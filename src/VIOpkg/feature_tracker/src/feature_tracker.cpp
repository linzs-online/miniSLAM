#include "../inc/feature_tracker.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/types_c.h>
#include <ros/ros.h>
#include <stdio.h>
#include <queue>
#include <map>


using namespace std;

FeatureTracker::FeatureTracker(Parameters::Ptr Ptr):paramPtr(Ptr)
{
    n_id = 0;
    inputImageCnt = 0;
}



// 计算两个像素点的距离
double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

// 边界检测，在边缘的点返回false
bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x); // cvRound 四舍五入取整
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

// 根据 status 清除跟踪失败的点 
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


void FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img0, const cv::Mat &_img1)
{
    cur_time = _cur_time;
    cur_image = _img0;

    row = cur_image.rows;
    col = cur_image.cols;

    cv::Mat cur_rightImg = _img1;

    cur_pts.clear();
    if (prev_pts.size() > 0){
        vector<uchar> status; // 用于记录成功追踪的特征点，对prev_pts每个点都设置一个0/1变量表示是否追踪成功
        vector<float> err;
        
        cv::calcOpticalFlowPyrLK(prev_image, cur_image, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3); // 3层光流追踪
        
        // 反向光流,用当前帧的特征点去筛选上一帧中稳定的特征点
        if(paramPtr->FLOW_BACK){
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            cv::calcOpticalFlowPyrLK(cur_image, prev_image, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
                                        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), 
                                        cv::OPTFLOW_USE_INITIAL_FLOW);
            // 把稳定的特征点记录下来
            for (size_t i = 0; i < status.size(); i++){
                if (status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5){
                    // 在反向追踪成功的点中挑选出稳定的（距离小于0.5的）点
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
        {// 对成功追踪并反向追踪成功的点进行边界检查，在边缘的点视为追踪失败
            if (status[i] && !inBorder(cur_pts[i]))
                status[i] = 0;
        }
        
        reduceVector(prev_pts, status); // 只保留那些在连续两帧追踪稳定的点
        reduceVector(cur_pts, status);  
        reduceVector(ids, status);      
        reduceVector(track_cnt, status);
    }

    for (auto &n : track_cnt)
        n++;
    
    cv::Mat mask = setMask(); // 屏蔽稳定的特征点附近的区域
    int insufficientPts_cnt = paramPtr->MAX_CNT - cur_pts.size(); 
    vector<cv::Point2f> n_pts;  // 存放要提取的新特征点
    if (insufficientPts_cnt > 0)
    {
        if(mask.empty())
            cout << "mask is empty " << endl;
        if (mask.type() != CV_8UC1)
            cout << "mask type wrong " << endl;
        // 在没屏蔽的区域进行特征点提取，缺多少提取多少
        cv::goodFeaturesToTrack(cur_image, n_pts, insufficientPts_cnt, 0.01, paramPtr->MIN_DIST, mask);
    }
    else
        n_pts.clear();
    // 把新提取的特征点存起来
    for (auto &p : n_pts)
    {
        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
    }

    // 获得去畸变归一化坐标
    cv::undistortPoints(cur_pts, cur_un_pts, paramPtr->cameraMatrix[0], paramPtr->distCoeffs[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);


    // 使用右目辅助
    ids_right.clear();
    cur_right_pts.clear();
    cur_un_right_pts.clear();
    right_pts_velocity.clear();
    cur_un_right_pts_map.clear();

    if(!cur_pts.empty())
    {
        //printf("stereo image; track feature on right image\n");
        vector<cv::Point2f> reverseLeftPts;
        vector<uchar> status, statusRightLeft;
        vector<float> err;
        // 在右目中追踪出左目的特征点
        cv::calcOpticalFlowPyrLK(cur_image, cur_rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
        // 反向光流，同时在左目中追踪出右目的特征点，进一步筛选出稳定的特征点
        if(paramPtr->FLOW_BACK)
        {
            cv::calcOpticalFlowPyrLK(cur_rightImg, cur_image, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
            for(size_t i = 0; i < status.size(); i++)
            {
                if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                    status[i] = 1;
                else
                    status[i] = 0;
            }
        }

        ids_right = ids;
        reduceVector(cur_right_pts, status);
        reduceVector(ids_right, status);

        cv::undistortPoints(cur_right_pts, cur_un_right_pts, paramPtr->cameraMatrix[1], paramPtr->distCoeffs[1]);
        right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
    }
    prev_un_right_pts_map = cur_un_right_pts_map;

    if(paramPtr->SHOW_TRACK)
        drawTrack(cur_image, cur_rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_image = cur_image;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];
    featurePointMap.clear();
    // 存储左目的特征点
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        // 无畸变归一化坐标
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        // 像素坐标
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        // 特征点速度
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featurePointMap[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    // 存储右目的特征点
    if (!_img1.empty())
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featurePointMap[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }
}

cv::Mat FeatureTracker::setMask(){
    cv::Mat mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255)); // 先初始化一张全白的和原图一样大小的图片
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id; // < 追踪次数, <点坐标，ID> >
    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), // 自定义排序方式，按追踪次数降序排序
        [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });
    // 此时，这三个容器里面存的还是上一帧的信息，防止容器无限增长，先清零，之后重新载入当前帧的信息
    cur_pts.clear();
    ids.clear();  
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {// 类似非极大值抑制算法，在追踪次数很大的点附近不再记录
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first); // 当前帧的特征点的像素坐标
            ids.push_back(it.second.second);    // 当前帧中的特征点的ID
            track_cnt.push_back(it.first);      // 当前帧中的特征点的成功追踪次数
            cv::circle(mask, it.second.first, paramPtr->MIN_DIST, 0, -1);    // 以成功追踪次数最多的特征点为中心画圆
        }
    }
    return mask;
}


vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, 
                                            map<int, cv::Point2f> &prev_id_pts){
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear(); // ID-像素坐标
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]); // 先找到上一帧中对应当前点的迭代器
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else  // 找不到就会返回 end() 说明这个点是当前帧新增的，速度直接写 0 
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {  // 因为上一帧容器为空，所以当前帧应该是第一帧，速度直接写 0 
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}


/**
 * @brief 可视化追踪效果
 * 
 * @param imLeft 
 * @param imRight 
 * @param curLeftIds 
 * @param curLeftPts 
 * @param curRightPts 
 * @param prevLeftPtsMap 
 */
void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap){
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty())
        cv::hconcat(imLeft, imRight, imTrack); // 把左右两个图拼接到一个框框里面
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB); // 转换原始图为灰度图

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20); // 追踪次数大于 20 次的会绘制成蓝色的点，否则偏红色
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty())
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;  // 框框的右边显示右目的图像，并且把追踪到的点用绿色标记出来
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
        }
    } 
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {  // 画箭头，表示从上一帧追踪到当前帧的特征点的移动
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }
}