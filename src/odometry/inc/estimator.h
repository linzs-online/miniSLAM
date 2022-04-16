#pragma once
#include "../../parameters/src/parameters.h"
#include "feature_manager.h"
#include "feature_tracker.h"
#include "utility.h"
#include "pre_integrated.h"
#include "initial_alignment.h"

#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>




class Estimator
{
private:
    
public:
    using Ptr = std::shared_ptr<Estimator>;

    
    double td;
    bool initFirstPoseFlag = false;

    Matrix3d R_c2i[2];
    Vector3d t_c2i[2];

    Vector3d Ps[(windowSize + 1)];
    Vector3d Vs[(windowSize + 1)];
    Matrix3d Rs[(windowSize + 1)];
    Vector3d Bas[(windowSize + 1)];
    Vector3d Bgs[(windowSize + 1)];
    Vector3d g;
    FeatureManager f_manager;
    bool marginalization_flag;
    int frameCount;

    double Headers[(windowSize + 1)];
    IntegrationBase *tmp_pre_integration;
    map<double, ImageFrame> all_image_frame;
    Vector3d acc_0, gyr_0;
    Parameters::Ptr paramPtr;

    Estimator(Parameters::Ptr &parametersPtr);
    // 获得初始位姿相对于世界坐标原点的旋转矩阵
    void initFirstIMUPose(std::vector<std::pair<double, Eigen::Vector3d>> &accVector);
    void processImage(const FeatureMap &image, const double header);

    ~Estimator();
};
