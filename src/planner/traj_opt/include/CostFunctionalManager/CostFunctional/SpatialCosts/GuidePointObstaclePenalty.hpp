#ifndef GUIDE_POINT_OBSTACLE_PENALTY_HPP
#define GUIDE_POINT_OBSTACLE_PENALTY_HPP

#include"CostFunctionalManager/PlanningTypesAdapter.hpp"
#include"CostFunctionalManager/CostFunctional/SpatialCosts/ObstacleSoftPenalty.hpp"
#include"CostFunctionalManager/CostFunctional/SpatialCosts/ObstacleHardPenalty.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateObstaclePenalty(const int cp_idx,
                                           Types::ConstraintPoints *cps,
                                           const bool touch_goal,
                                           const Types::Vec3 &position,
                                           const double obs_clearance,
                                           const double obs_clearance_soft,
                                           const double weight_obs,
                                           const double weight_obs_soft,
                                           Types::Vec3 &grad_position)
    {
        if (!cps || cp_idx == 0 || cp_idx >= cps->cp_size ||
            cp_idx > Types::ConstraintPoints::two_thirds_id(cps->points, touch_goal))
        {
            return 0.0;
        }

        double cost = 0.0;
        for (size_t k = 0; k < cps->direction[cp_idx].size(); ++k)
        {
            const Types::Vec3 ray = position - cps->base_point[cp_idx][k];
            const double dist = ray.dot(cps->direction[cp_idx][k]);
            const Types::Vec3 direction = cps->direction[cp_idx][k];

            cost += accumulateObstacleHardPenalty(dist,
                                                    obs_clearance,
                                                    weight_obs,
                                                    direction,
                                                    grad_position);
            cost += accumulateObstacleSoftPenalty(dist,
                                                    obs_clearance_soft,
                                                    weight_obs_soft,
                                                    direction,
                                                    grad_position);
        }

        return cost;
    }

}//namespace cost_functional

#endif