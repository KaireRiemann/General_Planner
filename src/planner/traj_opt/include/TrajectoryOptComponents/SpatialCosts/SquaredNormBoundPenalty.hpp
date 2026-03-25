#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"

#include <algorithm>

namespace traj_opt_components
{
template <typename VecT>
inline double accumulateSquaredNormBoundPenalty(const VecT &value,
                                                const double max_squared_norm,
                                                const double smooth_eps,
                                                const double weight,
                                                VecT &grad_value,
                                                double *max_violation = nullptr)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const double violation = value.squaredNorm() - max_squared_norm;
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

    grad_value += weight * penalty_grad * 2.0 * value;
    return weight * penalty;
}
} // namespace traj_opt_components
