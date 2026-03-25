#pragma once

#include "TrajectoryOptComponents/PenaltyUtils.hpp"

#include <algorithm>

namespace traj_opt_components
{
template <typename PolyhedronT, typename Vec3T>
inline double accumulatePolytopePositionPenalty(const PolyhedronT &poly,
                                                const Vec3T &position,
                                                const double smooth_eps,
                                                const double weight,
                                                Vec3T &grad_position,
                                                double *max_violation = nullptr)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    double cost = 0.0;
    for (int k = 0; k < poly.rows(); ++k)
    {
        const Vec3T outer_normal = poly.template block<1, 3>(k, 0).transpose();
        const double violation = outer_normal.dot(position) + poly(k, 3);
        if (max_violation != nullptr)
        {
            *max_violation = std::max(*max_violation, violation);
        }

        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (smoothedL1(violation, smooth_eps, penalty, penalty_grad))
        {
            grad_position += weight * penalty_grad * outer_normal;
            cost += weight * penalty;
        }
    }
    return cost;
}
} // namespace traj_opt_components
