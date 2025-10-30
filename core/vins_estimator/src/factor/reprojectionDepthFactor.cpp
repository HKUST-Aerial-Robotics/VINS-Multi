/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "reprojectionDepthFactor.h"

namespace vins_multi{

double reprojectionDepthFactor::sqrt_info;
double reprojectionDepthFactor::sum_t;

reprojectionDepthFactor::reprojectionDepthFactor(const Eigen::Vector3d &_pts_i,
                                       const Eigen::Vector2d &_velocity_i,
                                       const double _td_i, const double _depth_j) : 
                                       pts_i(_pts_i), td_i(_td_i), depth_j(_depth_j)
{
    velocity_i.x() = _velocity_i.x();
    velocity_i.y() = _velocity_i.y();
    velocity_i.z() = 0;
};

bool reprojectionDepthFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double inv_dep_i = parameters[3][0];

    double td = parameters[4][0];

    Eigen::Vector3d pts_i_td;
    pts_i_td = pts_i - (td - td_i) * velocity_i;
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    double &residual = residuals[0];

    double dep_j = pts_camera_j.z();
    residual =  dep_j - depth_j;

    double reduce = sqrt_info;

    residual = reduce * residual;

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i.bottomRows<1>();
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j.bottomRows<1>();
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                     Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex.bottomRows<1>();
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            double &jacobian_feature = jacobians[3][0];
            jacobian_feature = reduce * (ric.transpose() * Rj.transpose() * Ri * ric * pts_i_td * -1.0 / (inv_dep_i * inv_dep_i)).z();
        }
        if (jacobians[4])
        {
            double &jacobian_td = jacobians[4][0];
            jacobian_td = reduce * (ric.transpose() * Rj.transpose() * Ri * ric * velocity_i / inv_dep_i).z() * -1.0 ;
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

void reprojectionDepthFactor::check(double **parameters)
{
    double *res = new double[2];
    double **jaco = new double *[5];
    jaco[0] = new double[2 * 7];
    jaco[1] = new double[2 * 7];
    jaco[2] = new double[2 * 7];
    jaco[3] = new double[2 * 1];
    jaco[4] = new double[2 * 1];
    Evaluate(parameters, res, jaco);
    puts("check begins");

    puts("my");

    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[4]) << std::endl
              << std::endl;

    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    double inv_dep_i = parameters[3][0];
    double td = parameters[4][0];

    Eigen::Vector3d pts_i_td;
    pts_i_td = pts_i - (td - td_i ) * velocity_i;
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    double residual;

    double dep_j = pts_camera_j.z();
    residual =  dep_j - depth_j;
    double reduce = sqrt_info / dep_j;
    residual = reduce * residual;

    puts("num");
    std::cout << residual << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 1, 20> num_jacobian;
    for (int k = 0; k < 20; k++)
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        double inv_dep_i = parameters[3][0];
        double td = parameters[4][0];


        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * Utility::deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * Utility::deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * Utility::deltaQ(delta);
        else if (a == 6 && b == 0)
            inv_dep_i += delta.x();
        else if (a == 6 && b == 1)
            td += delta.y();

        Eigen::Vector3d pts_i_td;
        pts_i_td = pts_i - (td - td_i) * velocity_i;
        Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;
        Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
        Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
        Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
        double tmp_residual;


        double dep_j = pts_camera_j.z();
        tmp_residual = dep_j - depth_j;
        tmp_residual = sqrt_info * sqrt_depth_j * tmp_residual;

        num_jacobian(k) = (tmp_residual - residual) / eps;
    }
    std::cout << num_jacobian.block<1, 6>(0, 0) << std::endl;
    std::cout << num_jacobian.block<1, 6>(0, 6) << std::endl;
    std::cout << num_jacobian.block<1, 6>(0, 12) << std::endl;
    std::cout << num_jacobian.block<1, 1>(0, 18) << std::endl;
    std::cout << num_jacobian.block<1, 1>(0, 19) << std::endl;
}

}