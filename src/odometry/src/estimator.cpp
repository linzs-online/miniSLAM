#include "../inc/estimator.h"


using namespace std;
std::mutex m_process;
/**
 * @brief Construct a new Estimator:: Estimator object
 * 
 * @param _parametersPtr 
 */
Estimator::Estimator(Parameters::Ptr &_parametersPtr):td(_parametersPtr->TD), f_manager{Rs,_parametersPtr}{
    ROS_INFO("init begins");
    paramPtr = _parametersPtr;

    m_process.lock();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;

    // 初始位姿
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();

    for (int i = 0; i < windowSize + 1; i++){
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();

        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr){
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    // 根据Parameter类的数据初始化两个相机的外参
    for (int i = 0; i < 2; i++){
        t_ic[i] = paramPtr->TIC[i];
        R_ic[i] = paramPtr->RIC[i];
        cout << " exitrinsic cam " << i << endl  << t_ic[i] << endl << R_ic[i].transpose() << endl;
    }
    // 初始化特征管理类的 R_ic
    for (int i = 0; i < 2; i++){
        f_manager.R_ic[i] = R_ic[i];
    }
    td = paramPtr->TD;
    g = paramPtr->G;

    m_process.unlock();
}

/**
 * @brief 获得 初始帧 到 世界坐标系 下的变换矩阵 Rs[0]
 * 
 * @param accVector 
 */
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector){
    printf("init first imu pose\n");
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}



/**
 * @brief 主要业务
 * 
 * @param _featurePointMap 
 * @param header 
 */
void Estimator::poseEstimation(pair<double, FeaturePointMap> &t_featurePointMap){
    double header = t_featurePointMap.first;
    FeaturePointMap featurePointMap = t_featurePointMap.second;

    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("header = %lu", header);     // 当前帧的时间
    ROS_DEBUG("Adding feature points %lu", featurePointMap.size()); // 当前帧的特征点的数量

    // 检查两张图的视差，判断是否为关键帧，后面进行边缘化操作
    if (f_manager.addFeatureCheckParallax(frameCount, t_featurePointMap.second, td)){
        marginalization_flag = 0;
        //printf("keyframe\n");
    }
    else{
        marginalization_flag = 1;
        //printf("non-keyframe\n");
    }
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving the %d frame", frameCount);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frameCount] = header;

    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frameCount], Bgs[frameCount]};
    // 填充imageframe的容器以及更新临时预积分初始值
    ImageFrame imageframe(featurePointMap, header);
    imageframe.pre_integration = tmp_pre_integration;

    all_image_frame.insert(make_pair(header, imageframe)); //all_image_frame 中记录了当前帧的特征点、预积分
    
    // 是否估计外参
    if (paramPtr->ESTIMATE_EXTRINSIC == 2){
        cout << "calibrating extrinsic param, rotation movement is needed" << endl;
        if (frameCount != 0){
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frameCount - 1, frameCount);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frameCount]->delta_q, calib_ric)){
                R_ic[0] = calib_ric;
                RIC[0] = calib_ric;
                paramPtr->ESTIMATE_EXTRINSIC = 1; // 先进行了一次外参动态标定，之后在后端优化中还会优化该值
            }
        }
    }
    
    // stereo + IMU initilization
    if (solverFlag == 0){   
        f_manager.initFramePoseByPnP(frameCount, Ps, Rs, t_ic, R_ic);
        f_manager.triangulate(frameCount, Ps, Rs, t_ic, R_ic);
        if (frameCount == windowSize){
            map<double, ImageFrame>::iterator frame_it;
            int i = 0;
            for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++){
                frame_it->second.R = Rs[i];
                frame_it->second.T = Ps[i];
                i++;
            }
            solveGyroscopeBias(all_image_frame, Bgs);
            for (int i = 0; i <= windowSize; i++){
                pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
            }
            optimization();
            updateLatestStates();
            solverFlag = 1;   // 初始化完成
            slideWindow();
            ROS_INFO("Initialization finish!");
        }
        if(frameCount < windowSize)
        {
            frameCount++;
            int prev_frame = frameCount - 1;
            Ps[frameCount] = Ps[prev_frame];
            Vs[frameCount] = Vs[prev_frame];
            Rs[frameCount] = Rs[prev_frame];
            Bas[frameCount] = Bas[prev_frame];
            Bgs[frameCount] = Bgs[prev_frame];
        }
    }

    // 如果已经进行了初始化
    else{
        f_manager.triangulate(frameCount, Ps, Rs, t_ic, R_ic);
        optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
    }

    
}

// 【单目初始化】先对纯视觉SFM初始化相机位姿，再和IMU对齐
// 1. 纯视觉SFM估计滑动窗内相机位姿和路标点逆深度
// 2. 视觉惯性联合校准，SFM与IMU积分对齐
bool Estimator::initialStructure(){
    map<double, ImageFrame>::iterator frame_it;
    Vector3d sum_g;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        sum_g += tmp_g;
    }
    // 加速度观测值的平均值
    Vector3d aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
    {
        double dt = frame_it->second.pre_integration->sum_dt;
        Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        //cout << "frame g " << tmp_g.transpose() << endl;
    }
    // 加速度观测值的方差
    var = sqrt(var / ((int)all_image_frame.size() - 1));
    // 如果加速度计的标准差大于0.25，说明IMU得到了充分的运动，足够初始化
    if(var < 0.25)
    {
        ROS_INFO("IMU excitation not enouth!");
        //return false;
    }

    Quaterniond Q[frameCount + 1]; 
    Vector3d T[frameCount + 1];
    map<int, Vector3d> sfm_tracked_points;  // 用于存储SfM重建出的特征点的坐标
    vector<SFMFeature> sfm_f;   // 三角化状态、特征点ID、共视观测帧与像素坐标、3D坐标、深度

    // 将现有在f_manager中的所有feature,转存到 SfMFeature 对象中
    for (auto &featurePoint : f_manager.featurePointList){
        SFMFeature tmp_feature; // 初始化一个SfMFeature对象
        tmp_feature.state = false;  // 所有的特征点都还没进行三角化
        tmp_feature.id = featurePoint.feature_id; // 特征点的ID
        int commonViewID  = featurePoint.start_frame;
        for (auto &it_per_frame : featurePoint.feature_per_frame){
            Vector3d temp_2Dpts = it_per_frame.normPoint;
            tmp_feature.observation.push_back(make_pair(commonViewID++, Eigen::Vector2d{temp_2Dpts.x(), temp_2Dpts.y()}));
        }
        // 存入到sfm_f容器中，之后做纯视觉估计
        sfm_f.push_back(tmp_feature);
    }

    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    // 目的是找到滑动窗口中第一个和最后一帧有足够共同特征与视差的 参考帧l 的索引ID
    // 然后通过 solveRelativeRT 计算 最后帧 到 参考帧 的位姿变换
    if (!relativePose(relative_R, relative_T, l))
    {
        cout << "Not enough features or parallax; Move device around" << endl;
        return false;
    }
    
    // 对窗口里面 每个图像 帧求解SfM问题
    // 获得所有图像帧相对于参考帧l的位姿和坐标点sfm_tracked_points
    GlobalSFM sfm;
    if(!sfm.construct(frameCount + 1, Q, T, 
                l, relative_R, relative_T, // 参考帧的信息
              sfm_f, sfm_tracked_points))
    {
        // 求解失败，则边缘化最早的一帧，并滑动窗口
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }
    // 至此，第一部分纯视觉SFM结束，这里的Q 和 T 存储的是窗口里面每一帧相对于第 l 帧（参考帧）的相对位姿


    // 但是此时图像帧的数目可能会超过滑动窗口的大小，所以还需要对那些不被包含在滑动窗口里面图像帧进行位姿求解
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        cv::Mat r, rvec, t, D, tmp_r;
        // 对于在窗口里面的不需要继续求解了，直接用上面 sfm 的结果
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);  // eigen 转 cv::Mat
        cv::Rodrigues(tmp_r, rvec); // 罗德里格斯公式将旋转矩阵转换成旋转向量
        cv::eigen2cv(P_inital, t);
        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.featurePoints)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {   
                it = sfm_tracked_points.find(feature_id);   // 首先找到已经通过 sfm 恢复出运动的点
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));  // 赋值 3D点
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));   // 赋值 2D点
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6) // 起码要 5 个点才能 PnP
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))   // 3D-2D : PnP 
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        cv::Rodrigues(rvec, r);     // 罗德里格斯公式 旋转向量 转回 旋转矩阵，注意，这个函数支持双向转换，只要数据类型对得上就行
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }  
    // 视觉 - IMU 对齐
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }
}

/**
 * @brief 获得滑动窗口中与最后一帧满足视差的参考帧，并求得最后一帧到参考帧的变换
 * 
 * @param relative_R 
 * @param relative_T 
 * @param l     //用于获取参考帧Index
 * @return true 
 * @return false 
 */
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < windowSize; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, windowSize); // 获得当前帧与窗口里面最后一帧共视的特征角点
        // 1. 首先角点数目得足够
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {   
                Vector2d pts_0(corres[j].first(0), corres[j].first(1)); // 第 j 个角点在当前帧的 (x,y)
                Vector2d pts_1(corres[j].second(0), corres[j].second(1)); // 第 j 个角点在最后帧的 (x,y)
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax += parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            // 2. 前帧与窗口里面最后一帧特征点平均视差要大于30，并且能够正常求解出 R 和 T的
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                //ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    // 共视特征点不够，直接退出
    return false;
}


/**
 * @brief 求解尺度因子，优化重力矢量，所有状态变量对齐到世界坐标系
 * 
 * @return true 
 * @return false 
 */
bool Estimator::visualInitialAlign(){
    VectorXd x;
    // 求解尺度因子 s， 优化重力矢量 g
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }
    for (int i = 0; i <= frameCount; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    double s = (x.tail<1>())(0);
    for (int i = 0; i <= windowSize; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    // 将所有状态量对齐到第0帧IMU坐标系
    for (int i = frameCount; i >= 0; i--)
        // P_wi = sP_wc - R_wi * P_ic
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i; 
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    { // 对于关键帧，将之前求出来的速度矢量乘以 R_wi 转到世界系下：
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);  // g 是参考帧下的重力矢量，所以 R0 就是参考帧到世界坐标系的旋转
    double yaw = Utility::R2ypr(R0 * Rs[0]).x(); // Rs[0] 指滑窗的第0帧到参考帧的旋转
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; // 让第0帧的航向为0
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frameCount; i++)   // 将滑窗中的 PVQ 全部对齐到第 0 帧的重力方向
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    // 位姿对齐之后重新三角化路标点
    f_manager.clearDepth();
    f_manager.triangulate(frameCount, Ps, Rs, t_ic, R_ic);

    return true;
}


/**
 * @brief 位姿优化
 * 
 */
void Estimator::optimization(){
    // 1. 因为ceres用的是double类型的数组,所以要做vector到double类型的变换
    // 主要是 para_Pose 和 para_SpeedBias
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0); // 核函数

    // 2. 添加要优化的参数
    // 2.1 P Q V Ba Bg
    for (int i = 0; i < frameCount + 1; i++)
    {   // 自定义参数更新方式
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], 7, local_parameterization); //重构参数，优化时实际使用的是3维空间位置矢量+3维的等效旋转矢量
        problem.AddParameterBlock(para_SpeedBias[i], 9);
    }
    
    // 2.2 相机外参Ric，Tic
    for (int i = 0; i < 2; i++){
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], 7, local_parameterization);
        if ((paramPtr->ESTIMATE_EXTRINSIC && frameCount == windowSize && Vs[0].norm() > 0.2) || openExEstimation){
            //ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else{
            //ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    
    // 2.3 Td数组  滑窗内第一个时刻的相机到IMU的时钟差
    problem.AddParameterBlock(para_Td[0], 1);
    if (!paramPtr->ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);
    
    // 3. 添加残差模块
    // 边缘化残差
    if (last_marginalization_info && last_marginalization_info->valid){
        // 边缘化残差与雅克比计算方式
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 lastMarg_Res_parameter_blocks);
    }

    // IMU预积分残差
    for (int i = 0; i < frameCount; i++){
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        // IMU预积分残差与雅克比计算方式
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }

    // 视觉重投影残差
    int f_m_cnt = 0; //每个特征点,观测到它的相机的计数 visual measurement count
    int feature_index = -1;
    for (auto &fpts : f_manager.featurePointList)
    {
        fpts.obsCount = fpts.feature_per_frame.size();
        if (fpts.obsCount < 4)
            continue;
 
        ++feature_index;

        // imu_i该特征点第一次被观测到的帧 ,imu_j = imu_i - 1
        int imu_i = fpts.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = fpts.feature_per_frame[0].normPoint;
        for (auto &it_per_frame : fpts.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)//既,本次不是第一次观测到
            {
                Vector3d pts_j = it_per_frame.normPoint;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, fpts.feature_per_frame[0].velocity, it_per_frame.velocity,
                    fpts.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
                    /* 相关介绍:
                    1 只在视觉量测中用了核函数loss_function 用的是huber
                    2 参数包含了para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]
                    3 ProjectionTwoFrameOneCamFactor这个重投影并不是很懂 */
            }
   
            Vector3d pts_j_right = it_per_frame.normPointRight;
            if(imu_i != imu_j) //既,本次不是第一次观测到
            {
                ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, fpts.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    fpts.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
            }
            else//既,本次是第一次观测到
            {
                ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, fpts.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    fpts.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
            }
               
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);

    // ------------------------------------写下来配置优化选项,并进行求解-----------------------------------------
    {
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        //options.num_threads = 2;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = paramPtr->NUM_ITERATIONS;
        //options.use_explicit_schur_complement = true;
        //options.minimizer_progress_to_stdout = true;
        //options.use_nonmonotonic_steps = true;

        if (marginalization_flag == MARGIN_OLD)
            options.max_solver_time_in_seconds = paramPtr->SOLVER_TIME * 4.0 / 5.0;
        else
            options.max_solver_time_in_seconds = paramPtr->SOLVER_TIME;
        ceres::Solver::Summary summary;//优化信息
        ceres::Solve(options, &problem, &summary);
        //cout << summary.BriefReport() << endl;
        ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
        //printf("solver costs: %f \n", t_solver.toc());

        double2vector();
        //printf("frame_count: %d \n", frame_count);
        if(frameCount < windowSize)
            return;
    }
    
    // -----------------------------marginalization 实际上就是求解 H 矩阵的过程，把通过H矩阵和b反解出雅克比和残差，为下一次求解边缘化约束提供准备 ------------------------------------
    //如果需要marg掉最老的一帧
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo(); // 记录这帧边缘化信息
        vector2double();
        // 上一次边缘化信息传递到本次边缘化
        if (last_marginalization_info && last_marginalization_info->valid){
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(lastMarg_Res_parameter_blocks.size()); i++){   
                // lastMarg_Res_parameter_blocks 是上一次marg之后剩下的变量的地址
                if (lastMarg_Res_parameter_blocks[i] == para_Pose[0] ||
                    lastMarg_Res_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // 创建新的marg因子 construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                            lastMarg_Res_parameter_blocks,
                                                                            drop_set);//这一步添加了marg信息
            
            // 将上一步marginalization后的信息作为先验信息
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // 添加IMU的marg信息
        // 然后添加第0帧和第1帧之间的IMU预积分值以及第0帧和第1帧相关优化变量
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                // 这一步添加IMU的marg信息
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                    vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},//优化变量
                    vector<int>{0, 1});//这里是0,1的原因是0和1是para_Pose[0], para_SpeedBias[0]是需要marg的变量
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // 添加视觉的maeg信息
        {
            int feature_index = -1;

            //这里是遍历滑窗所有的特征点
            for (auto &it_per_id : f_manager.featurePointList)
            {
                it_per_id.obsCount = it_per_id.feature_per_frame.size();
                if (it_per_id.obsCount < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;//这里是从特征点的第一个观察帧开始
                if (imu_i != 0)//如果第一个观察帧不是第一帧就不进行考虑，因此后面用来构建marg矩阵的都是和第一帧有共视关系的滑窗帧
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].normPoint;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.normPoint;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},//优化变量
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    // 右目的
                    Vector3d pts_j_right = it_per_frame.normPointRight;
                    if(imu_i != imu_j)
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},//优化变量
                            vector<int>{0, 4});//为0和3的原因是，para_Pose[imu_i]是第一帧的位姿，需要marg掉，而3是para_Feature[feature_index]是和第一帧相关的特征点，需要marg掉 
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                        vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{2});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }

                }
            }
        }


        // 上面通过调用 addResidualBlockInfo() 已经确定优化变量的数量、存储位置、长度以及待优化变量的数量以及存储位置，
        //-------------------------- 下面就需要调用 preMarginalize() 进行预处理
        marginalization_info->preMarginalize();
        
        //------------------------调用 marginalize 正式开始边缘化，反解出 雅克比 和 残差
        marginalization_info->marginalize(); 

        //------------------------在optimization的最后会有一部滑窗预移动的操作
        // 值得注意的是，这里仅仅是相当于将指针进行了一次移动，指针对应的数据还是旧数据，因此需要结合后面调用的 slideWindow() 函数才能实现真正的滑窗移动
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= windowSize; i++)//从1开始，因为第零帧的状态不要了
        {
            //这一步的操作指的是第i的位置存放的的是i-1的内容，这就意味着窗口向前移动了一格
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];//因此para_Pose这些变量都是双指针变量，因此这一步是指针操作
        }
        for (int i = 0; i < 2; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> Res_parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;//删除掉上一次的marg相关的内容

        last_marginalization_info = marginalization_info;//marg相关内容的递归
        lastMarg_Res_parameter_blocks = Res_parameter_blocks;//优化变量的递归，这里面仅仅是指针
        
    }
    //扔掉次新帧
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(lastMarg_Res_parameter_blocks), std::end(lastMarg_Res_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(lastMarg_Res_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(lastMarg_Res_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (lastMarg_Res_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               lastMarg_Res_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }


            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();


            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();

            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < 2; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            lastMarg_Res_parameter_blocks = parameter_blocks;
            
        }
    }
}

void Estimator::vector2double(){
    for (int i = 0; i <= windowSize; i++)
    {
        // 七维位姿
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        // 九维 速度+加速度偏置+陀螺仪偏置
        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }

    for (int i = 0; i < 2; i++)
    {   // 相机外参
        para_Ex_Pose[i][0] = t_ic[i].x();
        para_Ex_Pose[i][1] = t_ic[i].y();
        para_Ex_Pose[i][2] = t_ic[i].z();
        Quaterniond q{t_ic[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }


    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                        para_Pose[0][3],
                                                        para_Pose[0][4],
                                                        para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                        para_Pose[0][3],
                                        para_Pose[0][4],
                                        para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= windowSize; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                                para_SpeedBias[i][4],
                                para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                para_SpeedBias[i][7],
                                para_SpeedBias[i][8]);
        
    }

    for (int i = 0; i < 2; i++)
    {
        t_ic[i] = Vector3d(para_Ex_Pose[i][0],
                            para_Ex_Pose[i][1],
                            para_Ex_Pose[i][2]);
        R_ic[i] = Quaterniond(para_Ex_Pose[i][6],
                                para_Ex_Pose[i][3],
                                para_Ex_Pose[i][4],
                                para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    td = para_Td[0][0];
}

/**
 * @brief 返回要保留下来的变量的地址
 * 
 * @param addr_shift 
 * @return std::vector<double *> 
 */
std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx)
    {
        if (it.second >= m)
        {
            keep_block_size.push_back(parameter_block_size[it.first]);
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}