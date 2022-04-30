#include "../inc/feature_manager.h"
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

FeatureManager::FeatureManager(Matrix3d _Rs[], Parameters::Ptr _paramPtr):Rs(_Rs),paramPtr(_paramPtr)
{
    for (auto i = 0; i < 2; i++)
        R_ic[i].setIdentity();
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]){
    if(frameCnt > 0){
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &_featurePoint : featurePointList){
            if (_featurePoint.estimated_depth > 0){
                int index = frameCnt - _featurePoint.start_frame;
                if((int)_featurePoint.feature_per_frame.size() >= index + 1){
                    Vector3d ptsInCam = ric[0] * (_featurePoint.feature_per_frame[0].normPoint * _featurePoint.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[_featurePoint.start_frame] * ptsInCam + Ps[_featurePoint.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(_featurePoint.feature_per_frame[index].normPoint.x(), _featurePoint.feature_per_frame[index].normPoint.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // 第0帧的 相机坐标系到世界坐标系的转换
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D){
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;                  

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }

    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
    //pnp_succ = solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

int FeatureManager::getFeatureCount(){
    int cnt = 0;
    for (auto &_pts : featurePointList){   
        _pts.obsCount = _pts.feature_per_frame.size();
        // 如果这个特征点在两帧上被观测到了，并且第一次观测到的帧数不是最后，说明这个特征点有效
        if (_pts.obsCount >= 4 && _pts.start_frame < windowSize - 2){
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frameCount, const FeaturePointMap &_featurepointMap, double td)
{
    ROS_DEBUG("input feature: %d", (int)_featurepointMap.size());  // 特征点的数量
    ROS_DEBUG("num of feature: %d", getFeatureCount());      // 目前为止有效的特征点个数

    double parallax_sum = 0;    // 所有特征点的视差总和
    int parallax_num = 0;   
    last_track_num = 0;
    new_feature_num = 0;
    long_track_num = 0;

    // 1. 把所有特征点放入到 featureList中
    for(auto &_pts : _featurepointMap){   
        // 特征点在每帧中的信息，归一化平面坐标、像素坐标、归一化平面上的速度
        FeaturePerFrame f_per_fra(_pts.second[0].second, td);
        // 赋值入右目的信息
        f_per_fra.rightObservation(_pts.second[1].second);

        int feature_id = _pts.first;
        // 自定义查找函数，返回在feature中的迭代器
        auto it = find_if(featurePointList.begin(), featurePointList.end(), 
                            [feature_id](const FeaturePoint &it){
                                return it.feature_id == feature_id;
                            });
        // 找不到，说明这个点还没记录在 feature 里面，现在把它添加进去，并统计数目
        if (it == featurePointList.end()){
            FeaturePoint temp_featurePoint(feature_id, frameCount); // 特征点ID 与 起始帧ID
            temp_featurePoint.feature_per_frame.push_back(f_per_fra);   // 记录这个特征点被追踪到的帧ID
            featurePointList.push_back(temp_featurePoint);
            new_feature_num++;  // 新特征点数目++
        }
        else if (it->feature_id == feature_id){  // 如果已经被记录过的，那就只记录下被追踪的帧ID
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;    
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;   // 追踪次数超过四次，说明该点长时间成功追踪
        }
    }
    // 2. 追踪次数小于20次或者长时间追踪的点的数量很少，那么说明这个也是关键帧
    if (frameCount < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true; // 说明当前帧是新的关键帧，直接返回

    // 3. 计算每个特征在次新帧中和次次新帧中的视差
    for (auto &it_per_id : featurePointList)
    {
        if (it_per_id.start_frame <= frameCount - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frameCount - 1){
            // 总视差：该特征点在两帧归一化平面上坐标点的距离
            parallax_sum += compensatedParallax2(it_per_id, frameCount);
            parallax_num++;
        }
    }
    // 第一次加进去的，是关键帧
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current average parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= paramPtr->MIN_PARALLAX;
    }
}

double FeatureManager::compensatedParallax2(const FeaturePoint &_featurePoint, int frame_count)
{
    const FeaturePerFrame &frame_i = _featurePoint.feature_per_frame[frame_count - 2 - _featurePoint.start_frame];
    const FeaturePerFrame &frame_j = _featurePoint.feature_per_frame[frame_count - 1 - _featurePoint.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.normPoint;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.normPoint;

    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    ans = max(ans, sqrt(du * du + dv * dv));
    return ans;
}

// 获得指定两帧的共视特征点3D坐标
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    // 遍历所有特征点
    for (auto &it : featurePointList)
    {   // 1. 首先保证跟踪到这个特征点的起始帧和最后一帧在我们指定的两帧范围之内
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {   // 2. 获得给定两帧对当前特征点的3D坐标
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].normPoint;

            b = it.feature_per_frame[idx_r].normPoint;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::clearDepth()
{
    for (auto &featurePoint : featurePointList)
        featurePoint.estimated_depth = -1;
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]){
    for (auto &_featurePoint : featurePointList){
        if (_featurePoint.estimated_depth > 0)
            continue;
        // 1. 三角化获取特征点的深度
        {
            int firstFrame = _featurePoint.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose; 
            // t0,R0 描述的是这个点到世界坐标系的变换
            Eigen::Vector3d t0 = Ps[firstFrame] + Rs[firstFrame] * tic[0];
            Eigen::Matrix3d R0 = Rs[firstFrame] * ric[0];
            // 这里得到的位姿是世界坐标系下的，可以认为是世界坐标系原点到这个featurePoint的变换，所以R 和 t 都要取逆变换
            leftPose.leftCols<3>() = R0.transpose();    // 正交对称群的转置描述了一个相反的旋转
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[firstFrame] + Rs[firstFrame] * tic[1];
            Eigen::Matrix3d R1 = Rs[firstFrame] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = _featurePoint.feature_per_frame[0].normPoint.head(2);
            point1 = _featurePoint.feature_per_frame[0].normPointRight.head(2);

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);

            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();

            if (depth > 0)
                _featurePoint.estimated_depth = depth;
            else
                _featurePoint.estimated_depth = paramPtr->INIT_DEPTH;

            continue;
        }
        // _featurePoint.obsCount = _featurePoint.feature_per_frame.size();
        // if (_featurePoint.obsCount < 4) // 追踪超过4次的点才会对它估计深度
        //     continue;
        // int imu_i = _featurePoint.start_frame, imu_j = imu_i - 1;
    }
    
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d){
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    // 每个等式的第三行都可以由第一行和第二行线性组合得到，所以这里只需要计算前面两行
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);

    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);

    // 方程没有零解，通过SVD求最小二乘解
    Eigen::Vector4d triangulated_point;
    triangulated_point =
            design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}