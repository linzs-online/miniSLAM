#include "../inc/pre_integrated.h"



bool IMUAvailable(double &t, IMU_subscriber::Ptr &imu_subPtr){
    if(!imu_subPtr->acc_buf.empty() && t <= imu_subPtr->acc_buf.back().first)
        return true;
    else
        return false;
}

/**
 * @brief 根据imu_subPtr和时间区间获取要积分的IMU数据
 * 
 * @param t0 
 * @param t1 
 * @param imu_subPtr 
 * @param accVector 
 * @param gyrVector 
 * @return true 
 * @return false 
 */
bool getIMUInterVal(double t0, double t1, IMU_subscriber::Ptr &imu_subPtr, 
                                                         vector<pair<double, Eigen::Vector3d>> &accVector, 
                                                         vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(imu_subPtr->acc_buf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    if(t1 <= imu_subPtr->acc_buf.back().first)
    {
        // 在t0 之前的IMU数据全部丢弃
        while (imu_subPtr->acc_buf.front().first <= t0)
        {
            imu_subPtr->acc_buf.pop();
            imu_subPtr->gyr_buf.pop();
        }
        while (imu_subPtr->acc_buf.front().first < t1)
        {
            accVector.push_back(imu_subPtr->acc_buf.front());
            imu_subPtr->acc_buf.pop();
            gyrVector.push_back(imu_subPtr->gyr_buf.front());
            imu_subPtr->gyr_buf.pop();
        }
        accVector.push_back(imu_subPtr->acc_buf.front());
        gyrVector.push_back(imu_subPtr->gyr_buf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}                                

/**
 * @brief 获得 初始帧 到 世界坐标系 下的变换矩阵 Rs[0]
 * 
 * @param accVector 
 */
Matrix3d initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector){
    ROS_INFO("init first imu pose\n");
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    ROS_INFO("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    cout << "init R0 " << endl << R0 << endl;
    return R0;
    //Vs[0] = Vector3d(5, 0, 0);
}




IntegrationBase::IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg,
                    double &ACC_N, double &GYR_N, double &ACC_W, double &GYR_W)
                : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
                  linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
                  jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
                 sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}
{
        //这个noise是预积分的噪声方差矩阵
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
}
/**
 * @brief 计算delta_p，delta_v，delta_q，残差雅可比，残差协方差矩阵
 * @param dt 
 * @param acc 
 * @param gyr 
 */
void IntegrationBase::push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    dt_buf.push_back(dt);
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    propagate(dt, acc, gyr);
}

void IntegrationBase::midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
{
    ROS_INFO("midpoint integration"); 
    // 前一时刻的加速度计偏置修正
    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    // 求 k 时刻 到 k+1 时刻的角速度中值
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    // 求q[b(i)][b(k+1)],因为微小转动量，这里做了泰勒展开，sin[(\phi)/2]  --> (\phi)/2
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
    // 求q[b(i)][b(k+1)]*{a[b(k+1)] - 当前帧的偏置}
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    // 求 k 时刻 到 k+1 时刻的加速度中值
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    // 位移与速度
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    // 假设两帧之间偏置不变
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg; 

    if(update_jacobian)
    {
        Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Vector3d a_0_x = _acc_0 - linearized_ba;
        Vector3d a_1_x = _acc_1 - linearized_ba;
        Matrix3d R_w_x, R_a_0_x, R_a_1_x;

        // 向量 转换为 反对称矩阵，为后面计算F矩阵做准备
        R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
        R_a_0_x<<0, -a_0_x(2), a_0_x(1),
            a_0_x(2), 0, -a_0_x(0),
            -a_0_x(1), a_0_x(0), 0;
        R_a_1_x<<0, -a_1_x(2), a_1_x(1),
            a_1_x(2), 0, -a_1_x(0),
            -a_1_x(1), a_1_x(0), 0;

        MatrixXd F = MatrixXd::Zero(15, 15);
        F.block<3, 3>(0, 0) = Matrix3d::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
                                -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
        F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
        F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
        F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
        F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
        F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
                                -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
        F.block<3, 3>(6, 6) = Matrix3d::Identity();
        F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
        F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
        F.block<3, 3>(9, 9) = Matrix3d::Identity();
        F.block<3, 3>(12, 12) = Matrix3d::Identity();
        //cout<<"A"<<endl<<A<<endl;

        MatrixXd V = MatrixXd::Zero(15,18);
        V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
        V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
        V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
        V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
        V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
        V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
        V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
        V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

        // 残差雅克比矩阵的更新，来一个时刻IMU数据，误差传递一次，残差雅克比迭代更新一次
        jacobian = F * jacobian;
        // 协方差矩阵，描述残差的不确定度，同样地来一个时刻IMU数据，不确定度也迭代更新一次
        covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }
}                        

void IntegrationBase::propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
{
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;  

    // 误差传递，计算残差雅可比和残差协方差矩阵 
    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);
    
    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1; 
}

void IntegrationBase::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
}

Eigen::Matrix<double, 15, 1> IntegrationBase::IMU_residuals(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                                            const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
{
    // 预积分残差
    Eigen::Matrix<double, 15, 1> residuals;

    // 预积分残差对优化变量【陀螺仪的偏置】和【加速度计的偏置】的雅克比，这个雅克比在我们求取F矩阵的时候已经求取过一部分，这里直接拿过
    Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(0, 9);
    Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(0, 12);

    Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(3, 12);

    Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(6, 9);
    Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(6, 12);

    // 偏置的增量
    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;

    // 补偿预积分测量值
    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

    // 残差求取
    residuals.block<3, 1>(0, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(3, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(6, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(9, 0) = Baj - Bai;
    residuals.block<3, 1>(12, 0) = Bgj - Bgi;
    return residuals;
}