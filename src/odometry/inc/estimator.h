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
#include "pose_local_parameterization.h"
#include "imu_factor.h"

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
    Matrix3d R_ic[2]; // 相机到IMU的外参矩阵
    Vector3d t_ic[2];
    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;
    double prevTime, curTime;
    bool openExEstimation;

    // 都是 IMU 坐标系到世界坐标系的转换
    Vector3d Ps[(windowSize + 1)];
    Vector3d Vs[(windowSize + 1)];
    Matrix3d Rs[(windowSize + 1)];
    Vector3d Bas[(windowSize + 1)];
    Vector3d Bgs[(windowSize + 1)];

    vector<double> dt_buf[(windowSize + 1)];
    vector<Vector3d> linear_acceleration_buf[(windowSize + 1)];
    vector<Vector3d> angular_velocity_buf[(windowSize + 1)];

    Vector3d g;
    FeatureTracker featureTracker; // 提取特征的
    FeatureManager f_manager;   // 管理特征的
    MotionEstimator m_estimator;    // 求解参考帧的
    bool marginalization_flag;
    int frameCount;
    int inputImageCnt;

    double Headers[(windowSize + 1)];
    IntegrationBase *pre_integrations[(windowSize + 1)];    // 窗口中每帧都有一个预积分类
    IntegrationBase *tmp_pre_integration;
    map<double, ImageFrame> all_image_frame;
    Vector3d acc_0, gyr_0;
    Parameters::Ptr paramPtr;
    InitialEXRotation initial_ex_rotation;

    double para_Pose[windowSize + 1][7];
    double para_SpeedBias[windowSize + 1][9];
    double para_Feature[1000][1];
    double para_Ex_Pose[2][7];
    double para_Retrive_Pose[7];
    double para_Td[1][1];

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    Estimator(Parameters::Ptr &parametersPtr);
    // 获得初始位姿相对于世界坐标原点的旋转矩阵
    void initFirstIMUPose(std::vector<std::pair<double, Eigen::Vector3d>> &accVector);
    void poseEstimation(pair<double, FeaturePointMap> &t_featurePointMap);
    bool initialStructure();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    void optimization();
    void vector2double();
    ~Estimator();
};
