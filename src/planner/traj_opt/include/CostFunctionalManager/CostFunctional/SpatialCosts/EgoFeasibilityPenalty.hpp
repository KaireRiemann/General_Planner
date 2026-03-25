#ifndef EGO_FEASIBILITY_PENALTY_HPP
#define EGO_FEASIBILITY_PENALTY_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctional/SpatialCosts/EgoVelocityFeasibilityPenalty.hpp"
#include "CostFunctional/SpatialCosts/EgoAccelerationFeasibilityPenalty.hpp"
#include "CostFunctional/SpatialCosts/EgoJerkFeasibilityPenalty.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateFeasibilityPenalty(const Types::Vec3 &velocity, const Types::Vec3 &acceleration,
                                               const Types::Vec3 &jerk, const double max_vel, const double max_acc,
                                               const double max_jer, const double weight, Types::Vec3 &grad_vel,
                                               Types::Vec3 &grad_acc, Types::Vec3 &grad_jer)
    {
        if(weight <= 0.0)
        {
            return 0.0;
        }

        return accumulateVelocityPenalty(velocity,max_vel,weight,grad_vel) + accumulateAccelerationPenalty(acceleration,max_acc,weight,grad_acc) 
               + accumulateJerkPenalty(jerk,max_jer,weight,grad_jer);
    }

}//namespace cost_functional

#endif