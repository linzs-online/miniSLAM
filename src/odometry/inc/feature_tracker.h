#pragma once
#include "../../parameters/src/parameters.h"
#include "../inc/subscriber.h"
#include <opencv2/opencv.hpp>
#include "../../camera_models/include/camodocal/camera_models/CameraFactory.h"
#include "../../camera_models/include/camodocal/camera_models/CataCamera.h"
#include "../../camera_models/include/camodocal/camera_models/PinholeCamera.h"

using namespace std;
                    // feature ID        Camare ID        feature:      
using FeatureMap =  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>;

class FeatureTracker
{
public:
    using Ptr = shared_ptr<FeatureTracker>;
    Parameters::Ptr parameters;
    
    double cur_time, prev_time;
    cv::Mat cur_image, prev_image;
    int row, col;
    vector<cv::Point2f> cur_pts, prev_pts, cur_right_pts;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    cv::Mat mask;
    vector<cv::Point2f> n_pts;
    int n_id; 
    vector<cv::Point2f> cur_un_pts, cur_un_right_pts, prev_un_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<camodocal::CameraPtr> m_camera;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map, prevLeftPtsMap;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    cv::Mat imTrack;
    int inputImageCnt = 0;
    FeatureMap featureFrame;
    queue<pair<double, FeatureMap>> featureBuf;

    FeatureTracker(Parameters::Ptr Ptr);
    FeatureTracker() = delete;
    ~FeatureTracker();
    void IntrinsicParameter();
    FeatureMap trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    bool inBorder(const cv::Point2f &pt);
    void setMask();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap);
};

FeatureTracker::~FeatureTracker()
{
};

