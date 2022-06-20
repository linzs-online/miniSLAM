/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "../inc/pose_local_parameterization.h"
/**
 * @brief 定义参数加法，x的维度是globalSize，delta的维度是localSize
 * 
 * @param x 
 * @param delta 
 * @param x_plus_delta 
 * @return true 
 * @return false 
 */
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x); // 位置
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3); // 姿态
    Eigen::Map<const Eigen::Vector3d> dp(delta);
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized(); // 四元数乘法，* 内部已经重载过了

    return true;
}

/**
 * @brief 定义参数更新方式 x' = x + jacobin * delta_x 中的雅可比矩阵
 * 
 * @param x 
 * @param jacobian 
 * @return true 
 * @return false 
 */
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}
