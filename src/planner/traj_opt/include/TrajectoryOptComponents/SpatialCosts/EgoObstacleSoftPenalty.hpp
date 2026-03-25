#pragma once

#include "TrajectoryOptAdapters/EgoPlanningTypesAdapter.hpp"

#include <cmath>

namespace traj_opt_components
{
using EgoTypes = traj_opt_adapters::EgoPlanningTypesAdapter;

inline double accumulateEgoObstacleSoftPenalty(const double distance,
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
    if (violation <= 0.0)
    {
        return 0.0;
    }

    const double radius = 0.05;
    const double radius_sqr = radius * radius;
    const double term = std::sqrt(1.0 + violation * violation / radius_sqr);
    grad_position += -weight * violation / term * direction;
    return weight * radius_sqr * (term - 1.0);
}
} // namespace traj_opt_components
