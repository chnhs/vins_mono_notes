#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    // hs: 构造函数，输入的一个是已知的三维点，另外一个是经内参后的点
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    // hs: 计算residuals 和 jaconbians
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;                   // hs: pts_i, pts_j是指三维点
    Eigen::Matrix<double, 2, 3> tangent_base;       // hs: 切线空间
    static Eigen::Matrix2d sqrt_info;               // hs: sqrt_info是指L ( 计算公式：P=LLT )
    static double sum_t;                            // hs: 统计计算时间
};
