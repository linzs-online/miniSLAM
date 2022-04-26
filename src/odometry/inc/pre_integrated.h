#pragma once
#include "subscriber.h"
#include "estimator.h"
#include "../../parameters/src/parameters.h"




class PreIntegrated{
private:


public:
    using Ptr = shared_ptr<PreIntegrated>;
    Parameters::Ptr ParametersPtr;
    IMU_subscriber::Ptr IMU_Ptr;
    Estimator::Ptr estimatorPtr;
    double prevTime = 0.0f, curTime = 0.0f;
    bool first_imu = false;
    Vector3d acc_0, gyr_0;
    IntegrationBase *pre_integrations[(windowSize + 1)];

    PreIntegrated(Parameters::Ptr &parametersPtr, IMU_subscriber::Ptr &IMU_sub_Ptr);
    bool IMUAvailable(double t);
    bool getIMUInterVal(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accBuf, 
                                vector<pair<double, Eigen::Vector3d>> &gyrBuf);
    void IMU_prevIntegrated(const int& frameCount, vector<pair<double, Eigen::Vector3d>> &accVector,
                            vector<pair<double, Eigen::Vector3d>> &gyrVector);
    
    ~PreIntegrated() = default;
};

class IntegrationBase
{
  public:
    Parameters::Ptr paramPtr;
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;

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
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg);
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