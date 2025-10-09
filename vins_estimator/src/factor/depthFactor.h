/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

// #include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "estimator/parameters.h"

namespace vins_multi{

class depthFactor : public ceres::SizedCostFunction<1, 1>
{
  public:
    depthFactor(const double _depth);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    // Eigen::Vector3d pts_i, pts_j;
    double depth;
    // double sqrt_depth;
    // Eigen::Matrix<double, 2, 3> tangent_base;
    static double depth_covar;
    double sqrt_info;
    static double sum_t;
};

}