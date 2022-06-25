#pragma once
#include "../../ros_node/inc/subscriber.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;

bool IMUAvailable(double &t, IMU_subscriber::Ptr &imu_subPtr);
bool getIMUInterVal(double t0, double t1, IMU_subscriber::Ptr &imu_subPtr, 
                                          vector<pair<double, Eigen::Vector3d>> &accVector, 
                                          vector<pair<double, Eigen::Vector3d>> &gyrVector);

Matrix3d initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);                             

class IntegrationBase
{
  public:
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf; // 这上一个相机帧到这一个相机帧时间段里面中每个IMU时刻的加速度计数据
    std::vector<Eigen::Vector3d> gyr_buf; // 这上一个相机帧到这一个相机帧时间段里面中每个IMU时刻的陀螺仪计数据

    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;

    Eigen::Matrix<double, 15, 15> jacobian, covariance; // F 矩阵 和 V 矩阵
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;

    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg,
                    double &ACC_N, double &GYR_N, double &ACC_W, double &GYR_W);
    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1);
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg);
    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian);
    Eigen::Matrix<double, 15, 1> IMU_residuals(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj);

};