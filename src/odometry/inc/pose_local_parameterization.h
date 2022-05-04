/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "utility.h"

class PoseLocalParameterization : public ceres::LocalParameterization
{
    //重载的Plus函数给出了四元数的更新方法，接受参数分别为优化前的四元数【x】，用旋转矢量表示的增量【delta】，以及更新后的四元数【x_plus_delta】
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    // x对delta的雅克比矩阵
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    // 表示参数x的自由度（可能有冗余），比如四元数的自由度是4，旋转矩阵的自由度是9
    virtual int GlobalSize() const { return 7; };
    // 表示 Δ x  所在的正切空间（tangent space）的自由度，正切空间是流形（manifold）中概念
    virtual int LocalSize() const { return 6; };
};
