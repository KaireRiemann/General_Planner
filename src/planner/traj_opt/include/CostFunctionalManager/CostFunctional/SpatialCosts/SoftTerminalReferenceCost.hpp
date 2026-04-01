#ifndef SOFT_TERMINAL_REFERENCE_COST_HPP
#define SOFT_TERMINAL_REFERENCE_COST_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateSoftTerminalReferenceCost(const Types::Vec3 &terminal_position,
                                                      const Types::Vec3 &terminal_velocity,
                                                      const Types::Vec3 &reference_position,
                                                      const Types::Vec3 &reference_velocity,
                                                      const double weight_pos,
                                                      const double weight_vel,
                                                      Types::Vec3 &grad_terminal_position,
                                                      Types::Vec3 &grad_terminal_velocity)
    {
        double cost = 0.0;

        if (weight_pos > 0.0)
        {
            const Types::Vec3 pos_err = terminal_position - reference_position;
            cost += 0.5 * weight_pos * pos_err.squaredNorm();
            grad_terminal_position += weight_pos * pos_err;
        }

        if (weight_vel > 0.0)
        {
            const Types::Vec3 vel_err = terminal_velocity - reference_velocity;
            cost += 0.5 * weight_vel * vel_err.squaredNorm();
            grad_terminal_velocity += weight_vel * vel_err;
        }

        return cost;
    }
} // namespace cost_functional

#endif
