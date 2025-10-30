#include "pose_manifold.h"

namespace vins_multi {

bool PoseManifold::Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> p(x);
    Eigen::Map<const Eigen::Quaterniond> q(x + 3);
    Eigen::Map<const Eigen::Vector3d> dp(delta);
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);

    p_plus = p + dp;
    q_plus = (q * dq).normalized();
    return true;
}

bool PoseManifold::PlusJacobian(const double*, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
    J.setZero();
    J.topLeftCorner<6, 6>().setIdentity();
    return true;
}

bool PoseManifold::Minus(const double* y, const double* x, double* y_minus_x) const {
    Eigen::Map<const Eigen::Vector3d> py(y);
    Eigen::Map<const Eigen::Quaterniond> qy(y + 3);
    Eigen::Map<const Eigen::Vector3d> px(x);
    Eigen::Map<const Eigen::Quaterniond> qx(x + 3);

    Eigen::Map<Eigen::Vector3d> dp(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dtheta(y_minus_x + 3);

    dp = py - px;
    Eigen::Quaterniond q_rel = (qx.conjugate() * qy).normalized();
    if (q_rel.w() < 0.0) {
        q_rel.coeffs() *= -1.0;
    }
    dtheta = QuaternionToRotationVector(q_rel);
    return true;
}

bool PoseManifold::MinusJacobian(const double*, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobian);
    J.setZero();
    J.topLeftCorner<3, 3>().setIdentity();
    J.bottomRightCorner<3, 3>().setIdentity();
    return true;
}

Eigen::Vector3d PoseManifold::QuaternionToRotationVector(const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = q_in.normalized();
    const double norm_v = q.vec().norm();
    const double w = q.w();
    if (norm_v < 1e-12) {
        return 2.0 * q.vec();
    }
    const double angle = 2.0 * std::atan2(norm_v, w);
    return angle * q.vec() / norm_v;
}

}  // namespace vins_multi