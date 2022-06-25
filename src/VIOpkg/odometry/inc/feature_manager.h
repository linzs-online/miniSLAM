#pragma once
#include "../../feature_tracker/inc/feature_tracker.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;
// 存储每帧中的特征点信息
class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        // 无畸变归一化坐标系
        normPoint.x() = _point(0);
        normPoint.y() = _point(1);
        normPoint.z() = _point(2);
        // 像素坐标
        uv.x() = _point(3);
        uv.y() = _point(4);
        // 像素移动速度
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        normPointRight.x() = _point(0);
        normPointRight.y() = _point(1);
        normPointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
    }
    double cur_td;
    Vector3d normPoint, normPointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
};

// 根据特征点ID存储每个特征点的信息，包括跟踪到它的每帧特征点图
class FeaturePoint
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    int obsCount;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePoint(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          obsCount(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};



class FeatureManager{
public:
    const Matrix3d *Rs;
    Matrix3d R_ic[2];
    Parameters::Ptr paramPtr;
    list<FeaturePoint> featurePointList;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

    FeatureManager(Matrix3d _Rs[], Parameters::Ptr _paramPtr);
    FeatureManager() = default;
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool addFeatureCheckParallax(int frameCount, const FeaturePointMap &featureMap, double td);
    int getFeatureCount();
    double compensatedParallax2(const FeaturePoint &_featurePoint, int frame_count);
    void clearDepth();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    Eigen::VectorXd getDepthVector();
    // 获得两张图片的角点
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    void setDepth(const Eigen::VectorXd &x);
    void removeFailures();
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
    bool solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeFront(int frame_count);
    void removeBack();
    void removeOutlier(set<int> &outlierIndex);
    ~FeatureManager(){};
};

