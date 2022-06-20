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

/**
 * @brief 记录 优化参数、costFunction、lossFunction的地址，方便后面进行边缘化操作删除和修改
 * 
 */
struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}
    
    void Evaluate();  // 使用马氏距离函数，里面重新定义了 带归一化尺度的 残差和雅可比矩阵
    ceres::CostFunction *cost_function;  // 残差和雅可比矩阵的更新方式
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks;  // 与边缘化有关的变量块的地址
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
 * @brief 上一步边缘化后保留下来的先验信息，主要指从信息矩阵中恢复出来的雅可比矩阵和残差向量
 * 
 */
class MarginalizationInfo{
  public:
    MarginalizationInfo(){valid = true;};
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info); // 添加残差块的信息（优化变量、待marg的变量）
    void preMarginalize(); // 计算每个残差的雅克比，并更新 parameter_block_data
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);
    std::vector<ResidualBlockInfo *> factors;
    // m 为要marg掉的变量的个数  n 为要保留下的优化变量的个数
    int m, n;
    std::unordered_map<long, int> parameter_block_size; // 与边缘化有关的变量块的地址，以及该参数中参数的数量，以IMU为例就是[7,9,7,9]
    int sum_block_size; // 要保留下来的参数块的数量
    std::unordered_map<long, int> parameter_block_idx; // 与边缘化有关的变量块的地址，以及该变量块在信息矩阵中的起始位置
    std::unordered_map<long, double *> parameter_block_data;  // <指向与边缘化有关的变量块内存地址，变量块的内存地址> 这是后面preMarg会把相关变量地址拷贝到这个统一内存里面

    //以下三个容器数组会在构建marginalization_factor的时候用到
    std::vector<int> keep_block_size; // 要保留下来的参数块的每块中的参数的数量
    std::vector<int> keep_block_idx;  // 要保留下来的参数块在信息矩阵中的起始位置
    std::vector<double *> keep_block_data; // 要保留下来的参数块的每块实际数据地址

    
    Eigen::MatrixXd linearized_jacobians; //指的是边缘化之后从信息矩阵恢复出来雅克比矩阵
    Eigen::VectorXd linearized_residuals; //指的是边缘化之后从信息矩阵恢复出来的残差向量
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
