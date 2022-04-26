#pragma once
#include "../../parameters/src/parameters.h"
#include "feature_manager.h"
#include "feature_tracker.h"
#include "utility.h"
#include "pre_integrated.h"
#include "initial_alignment.h"
#include "initial_ex_rotation.h"
#include "initial_sfm.h"
#include "solve_5pts.h"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>




class Estimator
{
private:
    
public:
    using Ptr = std::shared_ptr<Estimator>;

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    double td;
    bool initFirstPoseFlag = false;
    int solverFlag = 0;
    double initial_timestamp = 0;
    Matrix3d R_i2c[2]; // 相机到IMU的外参矩阵
    Vector3d t_i2c[2];

    Vector3d Ps[(windowSize + 1)];
    Vector3d Vs[(windowSize + 1)];
    Matrix3d Rs[(windowSize + 1)];
    Vector3d Bas[(windowSize + 1)];
    Vector3d Bgs[(windowSize + 1)];
    Vector3d g;
    FeatureManager f_manager;
    MotionEstimator m_estimator;
    bool marginalization_flag;
    int frameCount;

    double Headers[(windowSize + 1)];
    IntegrationBase *pre_integrations[(windowSize + 1)];
    IntegrationBase *tmp_pre_integration;
    map<double, ImageFrame> all_image_frame;
    Vector3d acc_0, gyr_0;
    Parameters::Ptr paramPtr;
    InitialEXRotation initial_ex_rotation;

    Estimator(Parameters::Ptr &parametersPtr);
    // 获得初始位姿相对于世界坐标原点的旋转矩阵
    void initFirstIMUPose(std::vector<std::pair<double, Eigen::Vector3d>> &accVector);
    void processImage(const FeatureMap &image, const double header);
    bool initialStructure();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    ~Estimator();
};
