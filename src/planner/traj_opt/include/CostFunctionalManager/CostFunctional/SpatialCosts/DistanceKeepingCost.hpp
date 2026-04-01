#ifndef DISTANCE_KEEPING_COST_HPP
#define DISTANCE_KEEPING_COST_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"

#include <algorithm>

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateDistanceKeepingCost(const Types::Vec3 &position,
                                                const Types::Vec3 &target_position,
                                                const double min_distance,
                                                const double max_distance,
                                                const double z_tolerance,
                                                const double smooth_eps,
                                                const double weight_near,
                                                const double weight_far,
                                                const double weight_vertical,
                                                Types::Vec3 &grad_position)
    {
        if (weight_near <= 0.0 && weight_far <= 0.0 && weight_vertical <= 0.0)
        {
            return 0.0;
        }

        const Types::Vec3 delta = position - target_position;
        const double eps = std::max(smooth_eps, 1.0e-6);
        Eigen::Vector2d delta_xy = delta.head<2>();
        double dist_xy = delta_xy.norm();
        Eigen::Vector2d dir_xy = Eigen::Vector2d::UnitX();
        if (dist_xy > 1.0e-6)
        {
            dir_xy = delta_xy / dist_xy;
        }
        dist_xy = std::max(dist_xy, 1.0e-6);

        double cost = 0.0;
        double penalty = 0.0;
        double penalty_grad = 0.0;

        if (weight_near > 0.0)
        {
            const double near_violation = min_distance - dist_xy;
            if (smoothedL1(near_violation, eps, penalty, penalty_grad))
            {
                cost += weight_near * penalty;
                grad_position.head<2>() += weight_near * penalty_grad * (-dir_xy);
            }
        }

        if (weight_far > 0.0)
        {
            const double far_violation = dist_xy - max_distance;
            if (smoothedL1(far_violation, eps, penalty, penalty_grad))
            {
                cost += weight_far * penalty;
                grad_position.head<2>() += weight_far * penalty_grad * dir_xy;
            }
        }

        if (weight_vertical > 0.0)
        {
            const double dz = delta.z();
            const double z_violation = std::abs(dz) - std::max(0.0, z_tolerance);
            if (smoothedL1(z_violation, eps, penalty, penalty_grad))
            {
                cost += weight_vertical * penalty;
                const double sign_z = (dz >= 0.0) ? 1.0 : -1.0;
                grad_position.z() += weight_vertical * penalty_grad * sign_z;
            }
        }

        return cost;
    }
} // namespace cost_functional

#endif
