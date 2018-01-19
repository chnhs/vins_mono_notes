#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;


// hs: Marginalization 残差项构建 （上一次的残差+IMU+投影误差）
struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;         // hs: 计算函数
    ceres::LossFunction *loss_function;         // hs: 损失函数
    std::vector<double *> parameter_blocks;     // hs: 参数块
    std::vector<int> drop_set;

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

//
struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;                     // hs: sub_factor 残差模型
    Eigen::MatrixXd A;                                                // hs: information matrix 信息矩阵
    Eigen::VectorXd b;                                                // hs: information vector 信息向量
    std::unordered_map<long, int> parameter_block_size; //global size // hs: 不懂，什么global size 还是 local size
    std::unordered_map<long, int> parameter_block_idx; //local size   // hs: 不懂
};

class MarginalizationInfo
{
  public:
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    std::vector<ResidualBlockInfo *> factors;           // hs: 残差因子
    int m, n;
    std::unordered_map<long, int> parameter_block_size; //global size // hs: 保存参数的地址和长度
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx; //local size
    std::unordered_map<long, double *> parameter_block_data; // hs: 保存参数的地址和数据

    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;// hs: H
    Eigen::VectorXd linearized_residuals;// hs: b
    const double eps = 1e-8;

};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);                                  // hs： 构造函数
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;// hs: 运算函数

    MarginalizationInfo* marginalization_info;      // hs: 保存的边缘化信息
};
