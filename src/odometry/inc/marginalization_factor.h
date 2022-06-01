/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "utility.h"


const int NUM_THREADS = 4;

struct ResidualBlockInfo
{
    // 记录 整个优化器的 优化参数、costFunction、lossFunction的地址，方便后面进行边缘化操作删除和修改
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}
    
    //用于计算当前时刻的残差和雅克比
    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks;
    // 待marg的变量的ID
    std::vector<int> drop_set;

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    Eigen::VectorXd residuals;

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};
/**
 * @brief 这个类保存了优化时上一步边缘化后保留下来的先验信息
 * 
 */
class MarginalizationInfo{
  public:
    MarginalizationInfo(){valid = true;};
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    // 添加残差块的信息（优化变量、待marg的变量）
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    // 计算每个残差的雅克比，并更新 parameter_block_data
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);
    //这里将参数块分为Xm,Xb,Xr；  Xm表示被marg掉的参数块，Xb表示与Xm相连接的参数块，Xr表示剩余的参数块
    std::vector<ResidualBlockInfo *> factors;
    // m 为要marg掉的变量的个数  n 为要保留下的优化变量的个数
    int m, n;
    std::unordered_map<long, int> parameter_block_size; //global size <优化变量内存地址，localSize>
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx; //local size
    std::unordered_map<long, double *> parameter_block_data;  // <优化变量内存地址，数据>

    //以下三个容器数组会在构建marginalization_factor的时候用到
    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    //指的是边缘化之后从信息矩阵恢复出来雅克比矩阵和残差向量
    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
    bool valid;

};



class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};
