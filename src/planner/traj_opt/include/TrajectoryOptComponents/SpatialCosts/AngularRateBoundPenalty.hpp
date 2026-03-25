#pragma once

#include "TrajectoryOptComponents/SpatialCosts/SquaredNormBoundPenalty.hpp"

namespace traj_opt_components
{
template <typename VecT>
inline double accumulateAngularRateBoundPenalty(const VecT &angular_rate,
                                                const double max_angular_rate,
                                                const double smooth_eps,
                                                const double weight,
                                                VecT &grad_angular_rate,
                                                double *max_violation = nullptr)
{
    return accumulateSquaredNormBoundPenalty(angular_rate,
                                             max_angular_rate * max_angular_rate,
                                             smooth_eps,
                                             weight,
                                             grad_angular_rate,
                                             max_violation);
}
} // namespace traj_opt_components
