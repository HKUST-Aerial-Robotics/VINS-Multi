/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

namespace vins_multi{

class ProjectionTwoFrameOneCamDepthFactor : public ceres::SizedCostFunction<3, 7, 7, 7, 1, 1>
{
  public:
    ProjectionTwoFrameOneCamDepthFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
    				   const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
    				   const double _td_i, const double _td_j, const double _depth_j, const double _row_i, const double _row_j, const int _rows, const double _tr);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    double tr_i, tr_j;
    double depth_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static double depth_covar;
    static Eigen::Matrix2d proj_sqrt_info;
    Eigen::Matrix3d sqrt_info;
    static double sum_t;
};

}