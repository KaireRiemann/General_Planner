#pragma once

#include "TrajectoryOptComponents/SpatialCosts/SquaredNormBoundPenalty.hpp"

namespace traj_opt_components
{
template <typename VecT>
inline double accumulateAccelerationBoundPenalty(const VecT &acceleration,
                                                 const double max_acceleration,
                                                 const double smooth_eps,
                                                 const double weight,
                                                 VecT &grad_acceleration,
                                                 double *max_violation = nullptr)
{
    return accumulateSquaredNormBoundPenalty(acceleration,
                                             max_acceleration * max_acceleration,
                                             smooth_eps,
                                             weight,
                                             grad_acceleration,
                                             max_violation);
}
} // namespace traj_opt_components
