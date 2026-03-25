#pragma once

#include "TrajectoryOptAdapters/EgoPlanningTypesAdapter.hpp"
#include "TrajectoryOptComponents/SpatialCosts/EgoObstacleHardPenalty.hpp"
#include "TrajectoryOptComponents/SpatialCosts/EgoObstacleSoftPenalty.hpp"

namespace traj_opt_components
{
using EgoTypes = traj_opt_adapters::EgoPlanningTypesAdapter;

inline double accumulateEgoObstaclePenalty(const int cp_idx,
                                           EgoTypes::ConstraintPoints *cps,
                                           const bool touch_goal,
                                           const EgoTypes::Vec3 &position,
                                           const double obs_clearance,
                                           const double obs_clearance_soft,
                                           const double weight_obs,
                                           const double weight_obs_soft,
                                           EgoTypes::Vec3 &grad_position)
{
    if (!cps || cp_idx == 0 || cp_idx >= cps->cp_size ||
        cp_idx > EgoTypes::ConstraintPoints::two_thirds_id(cps->points, touch_goal))
    {
        return 0.0;
    }

    double cost = 0.0;
    for (size_t k = 0; k < cps->direction[cp_idx].size(); ++k)
    {
        const EgoTypes::Vec3 ray = position - cps->base_point[cp_idx][k];
        const double dist = ray.dot(cps->direction[cp_idx][k]);
        const EgoTypes::Vec3 direction = cps->direction[cp_idx][k];

        cost += accumulateEgoObstacleHardPenalty(dist,
                                                 obs_clearance,
                                                 weight_obs,
                                                 direction,
                                                 grad_position);
        cost += accumulateEgoObstacleSoftPenalty(dist,
                                                 obs_clearance_soft,
                                                 weight_obs_soft,
                                                 direction,
                                                 grad_position);
    }

    return cost;
}
} // namespace traj_opt_components
