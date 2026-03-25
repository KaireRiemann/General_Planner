#ifndef EGO_VELOCITY_FEASIBILITY_PENALTY_HPP
#define EGO_VELOCITY_FEASIBILITY_PENALTY_HPP

#include "CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateVelocityPenalty(const Types::Vec3 &velocity, const double max_velocity,
                                            const double weight, Types::Vec3 &grad_velocity)
    {
        if(weight <= 0.0)
        {
            return 0.0;
        }

        const double violation = velocity.squaredNorm() - max_velocity * max_velocity;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!positivePartCubic(violation, penalty, penalty_grad))
        {
            return 0.0;
        }

        grad_velocity += weight * penalty_grad * 2.0 * velocity;
        return weight * penalty;
    }

}//namespace cost_functional

#endif