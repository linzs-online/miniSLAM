#include "../inc/estimator.h"
#include <stdio.h>

using namespace std;
std::mutex m_process;

/**
 * @brief Construct a new Estimator:: Estimator object
 * 
 * @param _parametersPtr 初始位姿
 */
Estimator::Estimator(Parameters::Ptr &_parametersPtr):td(_parametersPtr->TD), f_manager{Rs,_parametersPtr}{
    ROS_INFO("Estimator Construct");
    solverFlag = INITIAL;
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
    frameCount = 0;
    m_process.unlock();
}
/**
 * @brief 预积分,获得Ps/Rs/Vs
 * 
 * @param accVector 
 * @param gyrVector 
 */
void Estimator::prevIntegrated(vector<pair<double, Eigen::Vector3d>> &accVector, vector<pair<double, Eigen::Vector3d>> &gyrVector){
    for(auto i = 0; i < accVector.size(); i++){
        double dt;
        if(i == 0)
            dt = accVector[i].first - prevTime;
        else if (i == accVector.size() - 1)
            dt = curTime - accVector[i - 1].first;
        else
            dt = accVector[i].first - accVector[i - 1].first;
        
        if (!first_imu)
        { // 第一个IMU数据处理方式
            first_imu = true;
            acc_0 = accVector[i].second; // 测量值
            gyr_0 = gyrVector[i].second;
        }
        if (!pre_integrations[frameCount])
        {   // 先创造一个预积分类
            pre_integrations[frameCount] = new IntegrationBase{acc_0, gyr_0, Bas[frameCount], Bgs[frameCount],
                                                               paramPtr->ACC_N, paramPtr->GYR_N, paramPtr->ACC_W, paramPtr->GYR_W};
        }
        if(frameCount != 0)
        {
            // 根据 dt 以及 每个时刻的IMU数据 传播误差，迭代残差雅克比、协方差矩阵
            pre_integrations[frameCount]->push_back(dt, accVector[i].second, gyrVector[i].second);
        
            Vector3d un_acc_0 = Rs[frameCount] * (acc_0 - Bas[frameCount]) - g; // 获得相机在世界坐标系下的真实加速度（含有白噪声）
            Vector3d un_gyr = 0.5 * (gyr_0 + gyrVector[i].second) - Bgs[frameCount]; // 获得相机的真实姿态角速度（含有白噪声）
            
            // 预积分 R 更新，更新旋转矩阵 
            Rs[frameCount] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            
            Vector3d un_acc_1 = Rs[frameCount] * (accVector[i].second - Bas[frameCount]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); // 中值加速度
            
            // 预积分 P 、 V 更新
            Ps[frameCount] += dt * Vs[frameCount] + 0.5 * dt * dt * un_acc;
            Vs[frameCount] += dt * un_acc;
        }
        acc_0 = accVector[i].second;
        gyr_0 = accVector[i].second; 
    }

}


/**
 * @brief 位姿优化
 * 
 */
void Estimator::poseEstimation(pair<double, FeaturePointMap> &t_featurePointMap){
    double header = t_featurePointMap.first;    // 当前帧的时间
    FeaturePointMap featurePointMap = t_featurePointMap.second;

    std::cout << "new image coming ------------------------------------------" << std::endl;
    std::cout <<"header =" << header << " feature points size: " << featurePointMap.size() << std::endl;     

    // 检查两张图的视差，判断是否为关键帧，后面进行边缘化操作
    if (f_manager.addFeatureCheckParallax(frameCount, t_featurePointMap.second, td)){
        marginalization_flag = 0;
        std::cout << "non key frame! " << std::endl;
    }
    else{
        marginalization_flag = 1;
        std::cout << "non key frame!" << std::endl;
    }

    std::cout << "solving the frame: " << frameCount <<  "  number of feature: " << f_manager.getFeatureCount() << std::endl;
    Headers[frameCount] = header;
    // 填充imageframe的容器以及更新临时预积分初始值
    ImageFrame imageframe(featurePointMap, header);
    imageframe.pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frameCount], Bgs[frameCount],
                                              paramPtr->ACC_N, paramPtr->GYR_N, paramPtr->ACC_W, paramPtr->GYR_W};
    all_image_frame.insert(make_pair(header, imageframe)); //all_image_frame 中记录了当前帧的特征点、预积分
    
    // 是否估计外参
    if (paramPtr->ESTIMATE_EXTRINSIC == 2){
        cout << "calibrating extrinsic param, rotation movement is needed" << endl;
        if (frameCount != 0){
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frameCount - 1, frameCount);
            Matrix3d calib_R_ic;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frameCount]->delta_q, calib_R_ic)){
                R_ic[0] = calib_R_ic;
                paramPtr->RIC[0] = calib_R_ic; // 校准写入参数文件
                paramPtr->ESTIMATE_EXTRINSIC = 1; // 先进行了一次外参动态标定，之后在后端优化中还会优化该值
            }
        }
    }
    
    // stereo + IMU initilization
    if (solverFlag == INITIAL){   
        // 第 0 帧不会参与到PnP解算中，第0帧的位姿已经通过一组加速度数据在初始化求得
        f_manager.initFramePoseByPnP(frameCount, Ps, Rs, t_ic, R_ic);   // 每个特征点有了深度就可以进行求解PNP了，这里得到的是视觉的Ps,Rs
        f_manager.triangulate(frameCount, Ps, Rs, t_ic, R_ic);  // 双目视差三角化测得特征点深度
        if (frameCount == windowSize){
            std::cout << "solver flag = initial" << std::endl;
            int i = 0;
            for (auto &frame_it : all_image_frame){
                frame_it.second.R = Rs[i];
                frame_it.second.T = Ps[i];
                i++;
            }
            solveGyroscopeBias(all_image_frame, Bgs); // 结合视觉求解陀螺仪偏置，然后重新进行一次误差传递
            // 根据视觉得到的新的陀螺仪的偏置对之前预积分得到的结果进行更新
            for (int i = 0; i <= windowSize; i++){
                pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
            }
            optimization();
            // updateLatestStates();
            solverFlag = NON_LINEAR;   // 初始化完成,系统进入紧耦合状态
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
        set<int> removeIndex = outliersRejection();
        f_manager.removeOutlier(removeIndex);
        slideWindow();
        f_manager.removeFailures();
        ROS_INFO("slideWindow finish!");
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
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        cv::Mat r, rvec, t, D, tmp_r;
        // 对于在窗口里面的不需要继续求解了，直接用上面 sfm 的结果
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * R_ic[0].transpose();
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
                auto it = sfm_tracked_points.find(feature_id);   // 首先找到已经通过 sfm 恢复出运动的点
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
        frame_it->second.R = R_pnp * R_ic[0].transpose();
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
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x, t_ic[0]);
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
        Ps[i] = s * Ps[i] - Rs[i] * t_ic[0] - (s * Ps[0] - Rs[0] * t_ic[0]);
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
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0); // 核函数，HuberLoss当预测偏差小于 δ 时，它采用平方误差,当预测偏差大于 δ 时，采用的线性误差。

    // 2. 显式添加要优化的参数，主要为了自定义参数的更新方式
    // 2.1 P Q V Ba Bg
    for (int i = 0; i < frameCount + 1; i++){  
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], 7, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], 9); // 实际上传进去的是指针
    }
    
    // 2.2 相机外参R_ic，t_ic
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
    
    /*************************************************************************************************/

    // 3. 添加残差模块
    // 边缘化残差约束
    if (last_marginalization_info && last_marginalization_info->valid){
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info); // 边缘化残差与雅克比更新计算方式
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
            if (imu_i != imu_j) // 左目第i帧投影到第j帧
            {
                Vector3d pts_j = it_per_frame.normPoint;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, 
                                                                                          fpts.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          fpts.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }
   
            Vector3d pts_j_right = it_per_frame.normPointRight;
            if(imu_i != imu_j)  // 右目第i帧投影到第j帧
            {
                ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, fpts.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                    fpts.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
            }
            else    // 第i帧从左目投影到右目
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
        options.trust_region_strategy_type = ceres::DOGLEG;  // 信赖域法
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
        //printf("frameCount: %d \n", frameCount);
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
            // 创建新的marg约束
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                            lastMarg_Res_parameter_blocks,
                                                                            drop_set);// 过渡类，这里把需要的信息集合起来，下面送给marginalization_info->addResidualBlockInfo()
            
            // 调用addResidualBlockInfo()函数将各个残差以及残差涉及的优化变量添加入优化变量中
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // 添加IMU的marg信息
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]); // IMU的CostFunction
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                    vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
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
                    if(imu_i != imu_j)  // 把左目第0帧的投影到窗口各帧，得到他们的视觉重投影误差
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
                    if(imu_i != imu_j)  // 把右目第0帧的投影到窗口各帧，得到他们的视觉重投影误差
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                            vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},//优化变量
                            vector<int>{0, 4});//为0和3的原因是，para_Pose[imu_i]是第一帧的位姿，需要marg掉，而3是para_Feature[feature_index]是和第一帧相关的特征点，需要marg掉 
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else  // 把左目的投影到右目，然后得到一个视觉重投影误差
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
        marginalization_info->preMarginalize();  
        marginalization_info->marginalize(); // 补充完整parameter_block_idx，构造信息矩阵，从信息矩阵中反解出雅可比矩阵 和 残差向量

        // 值得注意的是，这里仅仅是相当于将指针进行了一次移动，指针对应的数据还是旧数据，因此需要结合后面调用的 slideWindow() 函数才能实现真正的滑窗移动
        std::unordered_map<long, double *> addr_shift; 

        // 窗口中的各帧位姿
        for (int i = 1; i <= windowSize; i++)//从1开始，因为第零帧不要了
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];//因此para_Pose这些变量都是双指针变量，因此这一步是指针操作
        }
        for (int i = 0; i < 2; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];// 两个相机外参
        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];// 相机与IMU相差的时间

        vector<double *> Res_parameter_blocks = marginalization_info->getParameterBlocks(addr_shift); // 滑动窗口，记录要保留下来的参数地址等信息
        if (last_marginalization_info)
            delete last_marginalization_info;//删除掉上一次的marg相关的内容

        last_marginalization_info = marginalization_info;//marg相关内容的递归
        lastMarg_Res_parameter_blocks = Res_parameter_blocks;//优化变量的递归，这里面仅仅是指针
    }
    //扔掉次新帧
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(lastMarg_Res_parameter_blocks), std::end(lastMarg_Res_parameter_blocks), para_Pose[windowSize - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(lastMarg_Res_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(lastMarg_Res_parameter_blocks[i] != para_SpeedBias[windowSize - 1]);
                    if (lastMarg_Res_parameter_blocks[i] == para_Pose[windowSize - 1])
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
            for (int i = 0; i <= windowSize; i++)
            {
                if (i == windowSize - 1)
                    continue;
                else if (i == windowSize)
                { // 因为最新帧和次新帧很像，所以直接丢弃次新帧的位姿信息，保持IMU积分的连贯性
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
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
        Eigen::Quaterniond q{R_ic[i]};
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

void Estimator::slideWindow()
{
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0]; // 记录下滑掉的位姿
        back_P0 = Ps[0];
        if (frameCount == windowSize)
        {
            for (int i = 0; i < windowSize; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);//交换
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
                std::swap(pre_integrations[i], pre_integrations[i + 1]);//交换预积分值
                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
            }// 此时最旧帧已经被换到了最新帧的位置

            Headers[windowSize] = Headers[windowSize - 1];
            Ps[windowSize] = Ps[windowSize - 1];
            Rs[windowSize] = Rs[windowSize - 1];
            Vs[windowSize] = Vs[windowSize - 1];
            Bas[windowSize] = Bas[windowSize - 1];
            Bgs[windowSize] = Bgs[windowSize - 1];
            delete pre_integrations[windowSize];//讲预积分的最后一个值删除
            pre_integrations[windowSize] = new IntegrationBase{acc_0, gyr_0, Bas[windowSize], Bgs[windowSize], paramPtr->ACC_N, paramPtr->GYR_N, paramPtr->ACC_W, paramPtr->GYR_W};//在构造一个新的

            dt_buf[windowSize].clear();
            linear_acceleration_buf[windowSize].clear();
            angular_velocity_buf[windowSize].clear();

            if (true || solverFlag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);//找到第一个
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            
            bool shift_depth = solverFlag == NON_LINEAR ? true : false;//判断是否处于初始化
            if (shift_depth)//如果不是初始化
            {
                Matrix3d R0, R1;
                Vector3d P0, P1;
                R0 = back_R0 * R_ic[0];
                R1 = Rs[0] * R_ic[0];
                P0 = back_P0 + back_R0 * t_ic[0];
                P1 = Ps[0] + Rs[0] * t_ic[0];
                f_manager.removeBackShiftDepth(R0, P0, R1, P1); // 调整特征点的起始观测帧
            }
            else
                f_manager.removeBack();
        }
    }

    // marg掉倒数第二帧,也很简单,另倒数第二个等于新的一个就可以
    else
    {
        if (frameCount == windowSize)
        {
            Headers[frameCount - 1] = Headers[frameCount];
            Ps[frameCount - 1] = Ps[frameCount];
            Rs[frameCount - 1] = Rs[frameCount];
            for (unsigned int i = 0; i < dt_buf[frameCount].size(); i++)
            {
                double tmp_dt = dt_buf[frameCount][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frameCount][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frameCount][i];

                pre_integrations[frameCount - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frameCount - 1].push_back(tmp_dt);
                linear_acceleration_buf[frameCount - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frameCount - 1].push_back(tmp_angular_velocity);
            }

            Vs[frameCount - 1] = Vs[frameCount];
            Bas[frameCount - 1] = Bas[frameCount];
            Bgs[frameCount - 1] = Bgs[frameCount];

            delete pre_integrations[windowSize];
            pre_integrations[windowSize] = new IntegrationBase{acc_0, gyr_0, Bas[windowSize], Bgs[windowSize], paramPtr->ACC_N, paramPtr->GYR_N, paramPtr->ACC_W, paramPtr->GYR_W};

            dt_buf[windowSize].clear();
            linear_acceleration_buf[windowSize].clear();
            angular_velocity_buf[windowSize].clear();

            // 移除该帧特征点信息
            f_manager.removeFront(frameCount);
        }
    }
}

// 移除野点,返回野点的迭代器
// 移除重投影误差大于3个像素的
set<int> Estimator::outliersRejection()
{
    int feature_index = -1;
    set<int> removeIndex;
    for (auto &it_per_id : f_manager.featurePointList)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.obsCount = it_per_id.feature_per_frame.size();
        if (it_per_id.obsCount < 4)
            continue;//跳出本次循环
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].normPoint;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.normPoint;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], R_ic[0], t_ic[0], 
                                                    Rs[imu_j], Ps[imu_j], R_ic[0], t_ic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
  
            Vector3d pts_j_right = it_per_frame.normPointRight;
            if(imu_i != imu_j)
            {            
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], R_ic[0], t_ic[0], 
                                                    Rs[imu_j], Ps[imu_j], R_ic[1], t_ic[1],
                                                    depth, pts_i, pts_j_right);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            else
            {
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], R_ic[0], t_ic[0], 
                                                    Rs[imu_j], Ps[imu_j], R_ic[1], t_ic[1],
                                                    depth, pts_i, pts_j_right);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }       

        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)//误差大于三个像素
            removeIndex.insert(it_per_id.feature_id);
    }
    return removeIndex;
}

// 计算重投影误差
double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &R_ici, Vector3d &t_ici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &R_icj, Vector3d &t_icj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (R_ici * (depth * uvi) + t_ici) + Pi;
    Vector3d pts_cj = R_icj.transpose() * (Rj.transpose() * (pts_w - Pj) - t_icj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

Estimator::~Estimator(){

}