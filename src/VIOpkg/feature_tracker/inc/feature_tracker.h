#pragma once
#include "../../parameters/src/parameters.h"
#include "../../ros_node/inc/subscriber.h"
#include <opencv2/opencv.hpp>
#include "../../parameters/src/parameters.h"
using namespace std;
                         // feature ID        Camare ID        feature:      
using FeaturePointMap =  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>;

class FeatureTracker
{
public:
    using Ptr = shared_ptr<FeatureTracker>;
    Parameters::Ptr paramPtr;
    
    double cur_time, prev_time;
    cv::Mat cur_image, prev_image;
    int row, col;
    vector<cv::Point2f> cur_pts, prev_pts, cur_right_pts;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    int n_id; 
    vector<cv::Point2f> cur_un_pts, cur_un_right_pts, prev_un_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map, prevLeftPtsMap;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    cv::Mat imTrack;
    int inputImageCnt;
    FeaturePointMap featurePointMap; //map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>;

    // 这个特征类就是为了构建这个特征点队列
    queue<pair<double, FeaturePointMap>> t_featureQueue;

    FeatureTracker(Parameters::Ptr Ptr);
    FeatureTracker() = default;
    void trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    bool inBorder(const cv::Point2f &pt);
    cv::Mat setMask();
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap);
};

