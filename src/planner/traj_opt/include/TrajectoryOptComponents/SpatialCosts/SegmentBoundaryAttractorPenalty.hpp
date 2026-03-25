#pragma once

#include "TrajectoryOptComponents/SpatialCosts/WaypointAttractorPenalty.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace traj_opt_components
{
template <typename PositionT, typename AttractorMatT, typename DeadDistanceVecT, typename GradT>
inline double accumulateSegmentBoundaryAttractorPenalty(const double local_time,
                                                        const int seg_idx,
                                                        const std::vector<double> *segment_times,
                                                        const AttractorMatT *waypoint_attractor,
                                                        const DeadDistanceVecT *waypoint_attractor_dead_d,
                                                        const PositionT &position,
                                                        const double smooth_eps,
                                                        const double weight,
                                                        GradT &grad_position,
                                                        double *max_violation = nullptr)
{
    if (weight <= 0.0 || segment_times == nullptr ||
        waypoint_attractor == nullptr || waypoint_attractor_dead_d == nullptr)
    {
        return 0.0;
    }

    const double seg_duration = (*segment_times)[seg_idx];
    const double eps = std::max(1e-9, 1e-7 * std::max(1.0, seg_duration));
    const bool is_start_node = (seg_idx != 0) && (std::abs(local_time) <= eps);
    const bool is_end_node =
        (seg_idx != static_cast<int>(segment_times->size()) - 1) && (std::abs(local_time - seg_duration) <= eps);

    if (!is_start_node && !is_end_node)
    {
        return 0.0;
    }

    const int idx = is_end_node ? seg_idx : seg_idx - 1;
    return accumulateWaypointAttractorPenalty(position,
                                              waypoint_attractor->col(idx),
                                              (*waypoint_attractor_dead_d)(idx),
                                              smooth_eps,
                                              weight,
                                              grad_position,
                                              max_violation);
}
} // namespace traj_opt_components
