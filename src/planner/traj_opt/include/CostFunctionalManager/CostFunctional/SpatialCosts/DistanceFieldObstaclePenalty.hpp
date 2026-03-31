#ifndef DISTANCE_FIELD_OBSTACLE_PENALTY_HPP
#define DISTANCE_FIELD_OBSTACLE_PENALTY_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateDistanceFieldObstaclePenalty(const Types::Vec3 &p, const double sdf_value,
                                                         const Types::Vec3 &sdf_grad, const double safe_margin,
                                                         const double weight, Types::Vec3 &p_grad)
    {
        if(weight <= 0.0)
        {
            return 0.0;
        }

        const double violation = safe_margin - sdf_value;
        double penalty = 0.0, penalty_grad = 0.0;
        if (!positivePartCubic(violation, penalty, penalty_grad))
        {
            return 0.0;
        } 

        p_grad += weight * penalty_grad * (-sdf_grad);
        return weight * penalty; 
    }

}//namespace cost_functional

#endif
