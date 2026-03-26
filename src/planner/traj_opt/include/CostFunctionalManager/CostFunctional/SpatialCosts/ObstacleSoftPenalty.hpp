#ifndef OBSTACLE_SOFT_PENALTY_HPP
#define OBSTACLE_SOFT_PENALTY_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateObstacleSoftPenalty(const double distance,
                                      const double clearance,
                                      const double weight,
                                      const Types::Vec3 &direction,
                                      Types::Vec3 &grad_position)
    {
        if(weight <= 0.0)
        {
            return 0.0;
        }

        const double violation = clearance - distance;
        if(violation < 0.0)
        {
            return 0.0;
        }

        const double radius = 0.05;
        const double radius_sqr = radius * radius;
        const double term = std::sqrt(1.0 + violation * violation / radius_sqr);
        grad_position += -weight * violation / term * direction;
        return weight * radius_sqr * (term - 1.0);
    }
}

#endif