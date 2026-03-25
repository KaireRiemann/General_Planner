#pragma once

#include "TrajectoryOptAdapters/EgoPlanningTypesAdapter.hpp"
#include "TrajectoryOptComponents/SpatialCosts/EgoAccelerationFeasibilityPenalty.hpp"
#include "TrajectoryOptComponents/SpatialCosts/EgoJerkFeasibilityPenalty.hpp"
#include "TrajectoryOptComponents/SpatialCosts/EgoVelocityFeasibilityPenalty.hpp"

namespace traj_opt_components
{
using EgoTypes = traj_opt_adapters::EgoPlanningTypesAdapter;

inline double accumulateEgoFeasibilityPenalty(const EgoTypes::Vec3 &velocity,
                                              const EgoTypes::Vec3 &acceleration,
                                              const EgoTypes::Vec3 &jerk,
                                              const double max_vel,
                                              const double max_acc,
                                              const double max_jer,
                                              const double weight,
                                              EgoTypes::Vec3 &grad_velocity,
                                              EgoTypes::Vec3 &grad_acceleration,
                                              EgoTypes::Vec3 &grad_jerk)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    return accumulateEgoVelocityFeasibilityPenalty(velocity, max_vel, weight, grad_velocity) +
           accumulateEgoAccelerationFeasibilityPenalty(acceleration, max_acc, weight, grad_acceleration) +
           accumulateEgoJerkFeasibilityPenalty(jerk, max_jer, weight, grad_jerk);
}
} // namespace traj_opt_components
