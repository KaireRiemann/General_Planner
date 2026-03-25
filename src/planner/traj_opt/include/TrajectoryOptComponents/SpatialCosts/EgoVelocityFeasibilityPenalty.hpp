#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"
#include "TrajectoryOptAdapters/EgoPlanningTypesAdapter.hpp"

namespace traj_opt_components
{
using EgoTypes = traj_opt_adapters::EgoPlanningTypesAdapter;

inline double accumulateEgoVelocityFeasibilityPenalty(const EgoTypes::Vec3 &velocity,
                                                      const double max_velocity,
                                                      const double weight,
                                                      EgoTypes::Vec3 &grad_velocity)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const double violation = velocity.squaredNorm() - max_velocity * max_velocity;
    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!positivePartCubic(violation, penalty, penalty_grad))
    {
        return 0.0;
    }

    grad_velocity += weight * penalty_grad * 2.0 * velocity;
    return weight * penalty;
}
} // namespace traj_opt_components
