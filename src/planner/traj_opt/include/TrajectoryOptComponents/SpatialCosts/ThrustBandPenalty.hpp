#pragma once

#include "TrajectoryOptComponents/SpatialCosts/ScalarBandPenalty.hpp"

#include <cmath>

namespace traj_opt_components
{
inline double accumulateThrustBandPenalty(const double thrust,
                                          const double min_thrust,
                                          const double max_thrust,
                                          const double smooth_eps,
                                          const double weight,
                                          double &grad_thrust,
                                          double *max_violation = nullptr)
{
    const double thrust_mean = 0.5 * (max_thrust + min_thrust);
    const double thrust_radius = 0.5 * std::abs(max_thrust - min_thrust);
    const double thrust_sqr_radius = thrust_radius * thrust_radius;
    return accumulateScalarBandPenalty(thrust,
                                       thrust_mean,
                                       thrust_sqr_radius,
                                       smooth_eps,
                                       weight,
                                       grad_thrust,
                                       max_violation);
}
} // namespace traj_opt_components
