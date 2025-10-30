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

class reprojectionDepthFactor : public ceres::SizedCostFunction<1, 7, 7, 7, 1, 1>
{
  public:
    reprojectionDepthFactor(const Eigen::Vector3d &_pts_i,
                                       const Eigen::Vector2d &_velocity_i,
                                       const double _td_i, const double _depth_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i;
    Eigen::Vector3d velocity_i;
    double td_i;
    double depth_j, sqrt_depth_j;
    static double sqrt_info;
    static double sum_t;
};
}
