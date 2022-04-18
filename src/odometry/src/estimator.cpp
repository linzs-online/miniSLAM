#include "../inc/estimator.h"


using namespace std;


Estimator::Estimator(Parameters::Ptr &_parametersPtr):td(_parametersPtr->TD)
{
    paramPtr = _parametersPtr;
}

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

    // 将现有在f_manager中的所有feature,转存到SfMFeature对象中
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
    
    // 对窗口里面每个图像帧求解SfM问题
    // 获得所有图像帧相对于参考帧l的位姿和坐标点sfm_tracked_points
    GlobalSFM sfm;
    if(!sfm.construct(frameCount + 1, Q, T, l,
              relative_R, relative_T,
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