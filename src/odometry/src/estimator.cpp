#include "../inc/estimator.h"


using namespace std;


Estimator::Estimator(Parameters::Ptr &_parametersPtr):td(_parametersPtr->TD)
{
    paramPtr = _parametersPtr;
}

// 获得 初始帧 相对于 世界坐标系 下的变换矩阵 R0
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
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
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::processImage(const FeatureMap &featureMap, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", featureMap.size());
    // 检查两张图的视差，判断是否为关键帧，后面进行边缘化操作
    if (f_manager.addFeatureCheckParallax(frameCount, featureMap, td))
    {
        marginalization_flag = 0;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = 1;
        //printf("non-keyframe\n");
    }
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving the %d frame", frameCount);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frameCount] = header;

    // 填充imageframe的容器以及更新临时预积分初始值
    ImageFrame imageframe(featureMap, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frameCount], Bgs[frameCount]};

    // 是否估计外参
    if (paramPtr->ESTIMATE_EXTRINSIC == 2)
    {
        cout << "calibrating extrinsic param, rotation movement is needed" << endl;
        if (frameCount != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frameCount - 1, frameCount);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frameCount]->delta_q, calib_ric))
            {
                // ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                                                            //    << calib_ric);
                R_i2c[0] = calib_ric;
                RIC[0] = calib_ric;
                paramPtr->ESTIMATE_EXTRINSIC = 1; // 先进行了一次外参动态标定，之后在后端优化中还会优化该值
            }
        }
    }

    
    if (solverFlag == 0) // 初始化模式
    {
        // 确保有足够的Frame参与初始化
        if (frameCount == windowSize)
        {
            bool result = false;
            if (paramPtr->ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
            {
                // cout << "1 initialStructure" << endl;
                // 视觉惯导联合初始化
                result = initialStructure();
                initial_timestamp = header;
            }
            if (result)
            {
                //初始化成功，先进行一次滑动窗口非线性优化，得到当前帧与第一帧的位姿
                solverFlag = 1;

            }
        }
        else
            frameCount++;
    }
}

// 先对纯视觉SFM初始化相机位姿，再和IMU对齐
// 1. 纯视觉SFM估计滑动窗内相机位姿和路标点逆深度
// 2. 视觉惯性联合校准，SFM与IMU积分对齐
bool Estimator::initialStructure()
{
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
    for (auto &it_per_id : f_manager.featureList)
    {
        SFMFeature tmp_feature; // 初始化一个SfMFeature对象
        tmp_feature.state = false;  // 所有的特征点都还没进行三角化
        tmp_feature.id = it_per_id.feature_id; // 特征点的ID
        int commonViewID  = it_per_id.start_frame;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            Vector3d temp_2Dpts = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(commonViewID++, Eigen::Vector2d{temp_2Dpts.x(), temp_2Dpts.y()}));
        }
        // 存入到sfm_f容器中，之后做纯视觉估计
        sfm_f.push_back(tmp_feature);
    }

    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    // 目的是找到滑动窗口中第一个和最后一帧有足够共同特征与视差的 参考帧l 的索引ID
    // 然后通过 solveRelativeRT 计算【参考帧l帧】和【最后帧】的位姿估计
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

// 获得滑动窗口中第一个与它最近的一帧满足视差的帧，为l帧，以及对应的R 和 T，说明可以进行三角化
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

bool Estimator::visualInitialAlign()
{
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
    f_manager.triangulate(frameCount, Ps, Rs, t_i2c, R_i2c);

    return true;
}