#include "../inc/feature_manager.h"
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

FeatureManager::FeatureManager(Matrix3d _Rs[], Parameters::Ptr _paramPtr):Rs(_Rs),paramPtr(_paramPtr)
{
    for (auto i = 0; i < 2; i++)
        R_ic[i].setIdentity();
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]){
    if(frameCnt >= 1){ // 第 0 帧不会参与到PnP解算中
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

        // 取前一帧的位姿作为先验位姿，传入 cv::solvePnP 中提高位姿计算效率
        Eigen::Matrix3d  R_wc = Rs[frameCnt - 1] * ric[0];
        Eigen::Vector3d  T_wc = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];
        Eigen::Matrix3d R_cw = R_wc.transpose();
        Eigen::Vector3d T_cw = -T_wc;

        if(solvePoseByPnP(R_cw, T_cw, pts2D, pts3D)){
            // Rs 里面存的是 R_wi , IMU 到世界坐标系的转换
            Rs[frameCnt] = R_cw.transpose() * ric[0].transpose(); 
            Ps[frameCnt] = - (Rs[frameCnt] * tic[0]) + (-T_cw);

            Eigen::Quaterniond Q(Rs[frameCnt]);
            //cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            //cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
        }
    }
}

bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &T, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D){
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }

    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(T, t);
    // 这里不需要求取畸变了，因为我们传进去的是归一化平面上的像素坐标，所以我们直接设置这个内参阵为单位矩阵即可
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
    // 另外这里传入了一个 rvec 旋转向量，这个是上一帧的位姿，这个先验位姿将作为初始化近似变换，最终会进一步优化他们，这样可以降低计算量
    bool pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
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

    // PnP得到的是世界坐标系到相机坐标系的转换
    R = R_pnp;
    T = T_pnp;

    return true;
}
/**
 * @brief 统计当前系统中记录的特征点的数量
 * 
 * @return int 
 */
int FeatureManager::getFeatureCount(){
    int cnt = 0;
    for (auto &_pts : featurePointList){   
        _pts.obsCount = _pts.feature_per_frame.size();
        if (_pts.obsCount >= 4){
            cnt++;
        }
    }
    return cnt;
}

/**
 * @brief 设置深度,这个在void Estimator::double2vector()中用了,如果失败,把solve_flag设置为2
 * 
 * @param x 
 */
void FeatureManager::setDepth(const Eigen::VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : featurePointList)
    {
        it_per_id.obsCount = it_per_id.feature_per_frame.size();
        if (it_per_id.obsCount < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        
        // 深度失败
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}
// 清除深度求解失败的点
void FeatureManager::removeFailures()
{
    for (auto it = featurePointList.begin(), it_next = featurePointList.begin(); it != featurePointList.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            featurePointList.erase(it);
    }
}

/**
 * @brief 把当前帧的特征点记录下来，并对当前帧与之前帧进行视差比较，判定采取的边缘化策略
 * 
 * @param frameCount 
 * @param _featurepointMap 
 * @param td 
 * @return true 说明当前帧是关键帧
 * @return false 
 */
bool FeatureManager::addFeatureCheckParallax(int frameCount, const FeaturePointMap &_featurepointMap, double td)
{
    ROS_DEBUG("input feature: %d", (int)_featurepointMap.size());  // 改帧上的特征点的数量
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
    for (auto &_featurePoint : featurePointList)
    {
        if (_featurePoint.start_frame <= frameCount - 2 &&
            _featurePoint.start_frame + int(_featurePoint.feature_per_frame.size()) - 1 >= frameCount - 1){
            // 总视差：该特征点在两帧归一化平面上坐标点的距离
            parallax_sum += compensatedParallax2(_featurePoint, frameCount);
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

/**
 * @brief 获得指定两帧的共视点的归一化坐标
 * 
 * @param frame_count_l 
 * @param frame_count_r 
 * @return vector<pair<Vector3d, Vector3d>> 
 */
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

/**
 * @brief 对所有的特征点进行三角化
 * 
 * @param frameCnt 
 * @param Ps 
 * @param Rs 
 * @param tic 
 * @param ric 
 */
void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]){
    for (auto &_featurePoint : featurePointList){
        if (_featurePoint.estimated_depth > 0)
            continue;
        // 1. 三角化获取特征点的深度
        {
            int firstFrame = _featurePoint.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose; 
            // t0,R0 描述的是相机坐标系到世界坐标系的变换
            Eigen::Vector3d t0 = Ps[firstFrame] + Rs[firstFrame] * tic[0];
            Eigen::Matrix3d R0 = Rs[firstFrame] * ric[0];
            // 这里得到的位姿是世界坐标系到相机坐标系的转换，可以认为是世界坐标系原点到这个featurePoint的变换，所以R 和 t 都要取逆变换
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
        }
    }
    
}

/**
 * @brief 子调用函数，用 SVD方法求解超定方程组，三角化获得特定点的深度
 * 
 * @param Pose0 
 * @param Pose1 
 * @param point0 
 * @param point1 
 * @param point_3d 结果存放
 */
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
/**
 * @brief 获取特征点的逆深度
 * 
 * @return Eigen::VectorXd 
 */
Eigen::VectorXd FeatureManager::getDepthVector()
{
    Eigen::VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : featurePointList)
    {
        it_per_id.obsCount = it_per_id.feature_per_frame.size();
        if (it_per_id.obsCount < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}
/**
 * @brief 调整观测到特征点的起始观测帧的信息
 * 
 * @param marg_R 
 * @param marg_P 
 * @param new_R 
 * @param new_P 
 */
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = featurePointList.begin(), it_next = featurePointList.begin(); it != featurePointList.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].normPoint;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2) // 直接删除仅在一帧被观测到的点
            {
                featurePointList.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = paramPtr->INIT_DEPTH;
            }
        }
    }
}

void FeatureManager::removeBack()
{
    for (auto it = featurePointList.begin(), it_next = featurePointList.begin(); it != featurePointList.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                featurePointList.erase(it);
        }
    }
}

// 在marg掉次新帧的时候会用
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = featurePointList.begin(), it_next = featurePointList.begin(); it != featurePointList.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = windowSize - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                featurePointList.erase(it);
        }
    }
}

// 移除野点
void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = featurePointList.begin(), it_next = featurePointList.begin(); it != featurePointList.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            featurePointList.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

//得到该特征点最后一次跟踪到的帧号
int FeaturePoint::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}