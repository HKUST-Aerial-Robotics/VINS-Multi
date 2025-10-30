/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "depthFactor.h"

namespace vins_multi{

// double depthFactor::sqrt_info;
double depthFactor::depth_covar;
double depthFactor::sum_t;

depthFactor::depthFactor(const double _depth) : depth(_depth), sqrt_info(1.0 / (depth_covar * depth))
{
};

bool depthFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    double dep_i = 1.0 / parameters[0][0];

    double reduce = sqrt_info;

    double &residual = residuals[0];

    residual = reduce * (dep_i - depth);

    if (jacobians)
    {
        if (jacobians[0])
        {
            double &jacobian_depth = jacobians[0][0];
            jacobian_depth = -1.0 * reduce * dep_i * dep_i;
        }

    }
    sum_t += tic_toc.toc();

    return true;
}

void depthFactor::check(double **parameters)
{
    double *res = new double[1];
    double **jaco = new double *[1];
    jaco[0] = new double[1];
    Evaluate(parameters, res, jaco);
    puts("check begins");

    puts("my");

    std::cout << Eigen::Map<Eigen::Matrix<double, 1, 1>>(res).transpose() << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 1, 1>>(jaco[0]) << std::endl
              << std::endl;
  
    double inv_dep_i = parameters[0][0];

    double reduce = sqrt_info * inv_dep_i;
    double residual = reduce * (1.0 / inv_dep_i - depth);

    puts("num");
    std::cout << residual << std::endl;

}

}