#pragma once
#include "../../parameters/src/parameters.h"
#include "../inc/feature_tracker.h"

// 存储每帧中的特征点信息
class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        // 无畸变归一化坐标系
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
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
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
    }
    double cur_td;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;
    Vector2d velocity, velocityRight;
};

// 根据特征点ID存储每个特征点的信息，包括跟踪到它的每帧特征点图
class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    int used_num;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};



class FeatureManager
{
private:
    const Matrix3d *Rs;
    Matrix3d R_c2i[2];

public:
    Parameters::Ptr paramPtr;
    list<FeaturePerId> featureList;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;

    FeatureManager(Matrix3d _Rs[], Parameters::Ptr _paramPtr);
    FeatureManager() = default;
    bool addFeatureCheckParallax(int frameCount, const FeatureMap &featureMap, double td);
    int getFeatureCount();
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

    // 获得两张图片的角点
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    ~FeatureManager();
};

FeatureManager::~FeatureManager()
{
}
