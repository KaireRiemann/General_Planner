#ifndef EGO_ACCELERATION_FEASIBILITY_PENALTY_HPP
#define EGO_ACCELERATION_FEASIBILITY_PENALTY_HPP

#include "CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateAccelerationPenalty(const Types::Vec3 &acceleration, const double max_acceleration,
                                            const double weight, Types::Vec3 &grad_acceleration)
    {
        if(weight <= 0.0)
        {
            return 0.0;
        }

        const double violation = acceleration.squaredNorm() - max_acceleration * max_acceleration;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!positivePartCubic(violation, penalty, penalty_grad))
        {
            return 0.0;
        }

        grad_acceleration += weight * penalty_grad * 2.0 * acceleration;
        return weight * penalty;
    }

}//namespace cost_functional

#endif