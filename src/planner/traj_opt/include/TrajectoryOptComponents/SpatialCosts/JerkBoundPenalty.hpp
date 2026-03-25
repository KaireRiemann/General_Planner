#pragma once

#include "TrajectoryOptComponents/SpatialCosts/SquaredNormBoundPenalty.hpp"

namespace traj_opt_components
{
template <typename VecT>
inline double accumulateJerkBoundPenalty(const VecT &jerk,
                                         const double max_jerk,
                                         const double smooth_eps,
                                         const double weight,
                                         VecT &grad_jerk,
                                         double *max_violation = nullptr)
{
    return accumulateSquaredNormBoundPenalty(jerk,
                                             max_jerk * max_jerk,
                                             smooth_eps,
                                             weight,
                                             grad_jerk,
                                             max_violation);
}
} // namespace traj_opt_components
