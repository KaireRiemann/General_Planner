#pragma once

#include "TrajectoryOptComponents/SpatialCosts/SquaredNormBoundPenalty.hpp"

namespace traj_opt_components
{
template <typename VecT>
inline double accumulateVelocityBoundPenalty(const VecT &velocity,
                                             const double max_velocity,
                                             const double smooth_eps,
                                             const double weight,
                                             VecT &grad_velocity,
                                             double *max_violation = nullptr)
{
    return accumulateSquaredNormBoundPenalty(velocity,
                                             max_velocity * max_velocity,
                                             smooth_eps,
                                             weight,
                                             grad_velocity,
                                             max_violation);
}
} // namespace traj_opt_components
