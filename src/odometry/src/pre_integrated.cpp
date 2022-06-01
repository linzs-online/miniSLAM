#include "../inc/pre_integrated.h"


PreIntegrated::PreIntegrated(Parameters::Ptr &parametersPtr, IMU_subscriber::Ptr &IMU_sub_Ptr)
{
    IMU_Ptr = IMU_sub_Ptr;
    ParametersPtr = parametersPtr;
}


bool PreIntegrated::IMUAvailable(double t){
    if(!IMU_Ptr->acc_buf.empty() && t <= IMU_Ptr->acc_buf.back().first)
        return true;
    else
        return false;
}


bool PreIntegrated::getIMUInterVal(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(IMU_Ptr->acc_buf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    if(t1 <= IMU_Ptr->acc_buf.back().first)
    {
        // 在t0 之前的IMU数据全部丢弃
        while (IMU_Ptr->acc_buf.front().first <= t0)
        {
            IMU_Ptr->acc_buf.pop();
            IMU_Ptr->gyr_buf.pop();
        }
        while (IMU_Ptr->acc_buf.front().first < t1)
        {
            accVector.push_back(IMU_Ptr->acc_buf.front());
            IMU_Ptr->acc_buf.pop();
            gyrVector.push_back(IMU_Ptr->gyr_buf.front());
            IMU_Ptr->gyr_buf.pop();
        }
        accVector.push_back(IMU_Ptr->acc_buf.front());
        gyrVector.push_back(IMU_Ptr->gyr_buf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}                                


void PreIntegrated::prevIntegrated(const int& frameCount, vector<pair<double, Eigen::Vector3d>> &accVector,
                            vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    for(auto i = 0; i < accVector.size(); i++)
    {
        double dt;
        if(i == 0)
            dt = accVector[i].first - prevTime;
        else if (i == accVector.size() - 1)
            dt = curTime - accVector[i - 1].first;
        else
            dt = accVector[i].first - accVector[i - 1].first;
        
        if (!first_imu)
        { // 第一个IMU数据处理方式
            first_imu = true;
            acc_0 = accVector[i].second;
            gyr_0 = gyrVector[i].second;
        }
        if (!pre_integrations[frameCount])
        {   // 先创造一个预积分类
            pre_integrations[frameCount] = new IntegrationBase{acc_0, gyr_0, estimatorPtr->Bas[frameCount], estimatorPtr->Bgs[frameCount]};
        }
        if(frameCount != 0)
        {
            // 根据 dt 以及 每个时刻的IMU数据 传播误差，迭代残差雅克比、协方差矩阵
            pre_integrations[frameCount]->push_back(dt, accVector[i].second, gyrVector[i].second);
            estimatorPtr->tmp_pre_integration->push_back(dt, accVector[i].second, gyrVector[i].second);
            // 中值积分求测量值
            Vector3d un_acc_0 = estimatorPtr->Rs[frameCount] * (acc_0 - estimatorPtr->Bas[frameCount]) - estimatorPtr->g;
            
            // 移除了偏执之后的角速度
            Vector3d un_gyr = 0.5 * (gyr_0 + gyrVector[i].second) - estimatorPtr->Bgs[frameCount];
            
            // 预积分 R 更新，更新旋转矩阵 
            estimatorPtr->Rs[frameCount] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            
            Vector3d un_acc_1 = estimatorPtr->Rs[frameCount] * (accVector[i].second - estimatorPtr->Bas[frameCount]) - estimatorPtr->g;
            // 中值加速度
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            
            // 预积分 P 、 V 更新
            estimatorPtr->Ps[frameCount] += dt * estimatorPtr->Vs[frameCount] + 0.5 * dt * dt * un_acc;
            estimatorPtr->Vs[frameCount] += dt * un_acc;
        }
        acc_0 = accVector[i].second;
        gyr_0 = accVector[i].second; 
    }
}



IntegrationBase::IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
                : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0}
{

}

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