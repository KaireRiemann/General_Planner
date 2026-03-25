#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"
#include "TrajectoryOptAdapters/EgoPlanningTypesAdapter.hpp"

namespace traj_opt_components
{
using EgoTypes = traj_opt_adapters::EgoPlanningTypesAdapter;

inline double accumulateEgoObstacleHardPenalty(const double distance,
                                               const double clearance,
                                               const double weight,
                                               const EgoTypes::Vec3 &direction,
                                               EgoTypes::Vec3 &grad_position)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const double violation = clearance - distance;
    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!positivePartCubic(violation, penalty, penalty_grad))
    {
        return 0.0;
    }

    grad_position += -weight * penalty_grad * direction;
    return weight * penalty;
}
} // namespace traj_opt_components
