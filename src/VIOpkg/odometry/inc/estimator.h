#pragma once
#include "../../parameters/src/parameters.h"
#include "feature_manager.h"
#include "../../feature_tracker/inc/feature_tracker.h"
#include "utility.h"
#include "pre_integrated.h"
#include "initial_alignment.h"
#include "initial_ex_rotation.h"
#include "initial_sfm.h"
#include "solve_5pts.h"
#include "pose_local_parameterization.h"
#include "imu_factor.h"
#include "marginalization_factor.h"
#include "projectionOneFrameTwoCamFactor.h"
#include "projectionTwoFrameOneCamFactor.h"
#include "projectionTwoFrameTwoCamFactor.h"

#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


bool first_imu = false; // 中值积分需要使用，用于判断是否是第一个IMU时刻


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
    std::mutex mPropagate;
    std::mutex mBuf;
    int solverFlag = 0;
    // VINS系统的两种状态：
    enum SolverFlag
    {
        INITIAL,// 还未成功初始化
        NON_LINEAR // 已成功初始化，正处于紧耦合优化状态
    };
    double initial_timestamp = 0;
    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;
    double prevTime, curTime; // 图像帧时刻
    bool openExEstimation;

    double td;
    Matrix3d R_ic[2]; // 相机到IMU的外参矩阵
    Vector3d t_ic[2];
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
    FeatureTracker featureTracker; // 提取特征点
    FeatureManager f_manager;   // 管理特征类，存储着窗口中所有帧的特征点的信息
    MotionEstimator m_estimator;    // 求解参考帧�?
    bool marginalization_flag;
    int frameCount;     // 记录当前处理的是窗口中的第几帧
    int inputImageCnt;

    double Headers[(windowSize + 1)];
    IntegrationBase *pre_integrations[(windowSize + 1)];    // 窗口中每帧都有一个预积分�?
    map<double, ImageFrame> all_image_frame; // 记录所有帧的特征点、以及图像帧对应的预积分
    Vector3d acc_0, gyr_0;
    Parameters::Ptr paramPtr;
    InitialEXRotation initial_ex_rotation;

    double para_Pose[windowSize + 1][7];
    double para_SpeedBias[windowSize + 1][9];
    double para_Feature[1000][1];
    double para_Ex_Pose[2][7];
    double para_Retrive_Pose[7];
    double para_Td[1][1];

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    bool failure_occur;//检测是否发生了错误,在failureDetection中

    MarginalizationInfo *last_marginalization_info; //上一个H矩阵matg掉一部分后剩下的内容
    vector<double *> lastMarg_Res_parameter_blocks;

    Estimator(Parameters::Ptr &parametersPtr);
    void prevIntegrated(vector<pair<double, Eigen::Vector3d>> &accVector, 
                        vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void poseEstimation(pair<double, FeaturePointMap> &t_featurePointMap);
    bool initialStructure();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    void optimization();
    void slideWindow();
    void vector2double();
    void double2vector();
    set<int> outliersRejection();
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj);
    ~Estimator();

};
