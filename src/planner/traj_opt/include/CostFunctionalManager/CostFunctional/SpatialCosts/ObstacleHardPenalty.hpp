#ifndef OBSTACLE_HARD_PENALTY_HPP
#define OBSTACLE_HARD_PENALTY_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double ObstacleHardPenalty(const double distance,
                                      const double clearance,
                                      const double weight,
                                      const Types::Vec3 &direction,
                                      Types::Vec3 &grad_position)
    {
        if (weight <= 0.0)
        {
            return 0.0;
        }

        const double violation = clearance - distance;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!positivePartCubic(violation, penalty, penalty_grad))
        {
            return 0.0;
        }

        grad_position += -weight * penalty_grad * direction;
        return weight * penalty;
    }
}

#endif