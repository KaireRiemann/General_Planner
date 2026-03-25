#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>

namespace traj_opt_components
{
inline double accumulateTiltAnglePenalty(const Eigen::Vector4d &quaternion,
                                         const double max_tilt_angle,
                                         const double smooth_eps,
                                         const double weight,
                                         Eigen::Vector4d &grad_quaternion,
                                         double *max_violation = nullptr)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    double cos_theta = 1.0 - 2.0 * (quaternion(1) * quaternion(1) + quaternion(2) * quaternion(2));
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    const double violation = std::acos(cos_theta) - max_tilt_angle;
    if (max_violation != nullptr)
    {
        *max_violation = std::max(*max_violation, violation);
    }

    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!smoothedL1(violation, smooth_eps, penalty, penalty_grad))
    {
        return 0.0;
    }

    const double denom = std::sqrt(std::max(1.0 - cos_theta * cos_theta, 1e-12));
    grad_quaternion += weight * penalty_grad / denom * 4.0 *
                       Eigen::Vector4d(0.0, quaternion(1), quaternion(2), 0.0);
    return weight * penalty;
}
} // namespace traj_opt_components
