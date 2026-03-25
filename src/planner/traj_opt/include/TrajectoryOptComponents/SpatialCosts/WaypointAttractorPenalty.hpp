#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"

#include <algorithm>

namespace traj_opt_components
{
template <typename PositionT, typename AttractorT, typename GradT>
inline double accumulateWaypointAttractorPenalty(const PositionT &position,
                                                 const AttractorT &attractor,
                                                 const double dead_distance,
                                                 const double smooth_eps,
                                                 const double weight,
                                                 GradT &grad_position,
                                                 double *max_violation = nullptr)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const auto delta = position - attractor;
    const double violation = delta.squaredNorm() - dead_distance * dead_distance;
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

    grad_position += weight * penalty_grad * 2.0 * delta;
    return weight * penalty;
}
} // namespace traj_opt_components
