#pragma once
#include <Eigen/Dense>
#include <ceres/manifold.h>
#include "utility/utility.h"

namespace vins_multi {

class PoseManifold final : public ceres::Manifold {
public:
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
    bool PlusJacobian(const double* x, double* jacobian) const override;
    bool Minus(const double* y, const double* x, double* y_minus_x) const override;
    bool MinusJacobian(const double* x, double* jacobian) const override;
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }

private:
    static Eigen::Vector3d QuaternionToRotationVector(const Eigen::Quaterniond& q);
};

}  // namespace vins_multi