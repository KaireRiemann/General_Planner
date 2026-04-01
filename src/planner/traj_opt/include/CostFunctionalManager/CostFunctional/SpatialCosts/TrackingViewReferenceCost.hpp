#ifndef TRACKING_VIEW_REFERENCE_COST_HPP
#define TRACKING_VIEW_REFERENCE_COST_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateTrackingViewReferenceCost(const Types::Vec3 &position,
                                                      const Types::Vec3 &viewpoint_position,
                                                      const double weight_xy,
                                                      const double weight_z,
                                                      Types::Vec3 &grad_position)
    {
        if (weight_xy <= 0.0 && weight_z <= 0.0)
        {
            return 0.0;
        }

        const Types::Vec3 delta = position - viewpoint_position;
        double cost = 0.0;

        if (weight_xy > 0.0)
        {
            const Eigen::Vector2d delta_xy = delta.head<2>();
            cost += 0.5 * weight_xy * delta_xy.squaredNorm();
            grad_position.head<2>() += weight_xy * delta_xy;
        }

        if (weight_z > 0.0)
        {
            cost += 0.5 * weight_z * delta.z() * delta.z();
            grad_position.z() += weight_z * delta.z();
        }

        return cost;
    }
}

#endif