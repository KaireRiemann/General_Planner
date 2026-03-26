#ifndef EGO_JERK_FEASIBILITY_PENALTY_HPP
#define EGO_JERK_FEASIBILITY_PENALTY_HPP

#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateJerkPenalty(const Types::Vec3 &jerk, const double max_jerk,
                                            const double weight, Types::Vec3 &grad_jerk)
    {
        if(weight <= 0.0)
        {
            return 0.0;
        }

        const double violation = jerk.squaredNorm() - max_jerk * max_jerk;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!positivePartCubic(violation, penalty, penalty_grad))
        {
            return 0.0;
        }

        grad_jerk += weight * penalty_grad * 2.0 * jerk;
        return weight * penalty;
    }

}//namespace cost_functional

#endif