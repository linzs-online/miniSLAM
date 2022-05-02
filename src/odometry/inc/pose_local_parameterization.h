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
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    // 表示参数x的自由度（可能有冗余），比如四元数的自由度是4，旋转矩阵的自由度是9
    virtual int GlobalSize() const { return 7; };
    // 表示 Δ x  所在的正切空间（tangent space）的自由度，正切空间是流形（manifold）中概念
    virtual int LocalSize() const { return 6; };
};
