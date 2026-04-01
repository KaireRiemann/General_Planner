#ifndef TRACKING_LOS_COST_HPP
#define TRACKING_LOS_COST_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"
#include <algorithm>
#include <cmath>

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline double accumulateTrackingLoSCost(const Types::Vec3 &drone_pos,
                                            const Types::Vec3 &target_pos,
                                            const Types::GridMapPtr &grid_map,
                                            const double clearance,
                                            const double weight_los,
                                            Types::Vec3 &grad_drone_pos)
    {
        if (weight_los <= 0.0 || grid_map == nullptr)
        {
            return 0.0;
        }

        const Types::Vec3 ray = target_pos - drone_pos;
        const double ray_len = ray.norm();
        if (ray_len < 1.0e-4)
        {
            return 0.0;
        }

        const double res = std::max(grid_map->getResolution(), 0.05);
        const int sample_num = std::max(4, static_cast<int>(std::ceil(ray_len / res)));
        const double eps = std::max(0.02, 0.5 * res);

        double cost = 0.0;
        const bool use_esdf = grid_map->esdfEnabled();

        for (int i = 1; i < sample_num; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
            const Types::Vec3 q = (1.0 - alpha) * drone_pos + alpha * target_pos;

            if (use_esdf)
            {
                Types::Vec3 sdf_grad = Types::Vec3::Zero();
                const double sdf = grid_map->getDistWithGradTrilinear(q, sdf_grad);

                double penalty = 0.0;
                double penalty_grad = 0.0;
                if (smoothedL1(clearance - sdf, eps, penalty, penalty_grad))
                {
                    cost += weight_los * penalty / static_cast<double>(sample_num);
                    const Types::Vec3 grad_q =
                        weight_los * penalty_grad * (-sdf_grad) / static_cast<double>(sample_num);
                    grad_drone_pos += (1.0 - alpha) * grad_q;
                }
                continue;
            }

            if (grid_map->getInflateOccupancy(q) != 0)
            {
                cost += 0.5 * weight_los / static_cast<double>(sample_num);

                Types::Vec3 occ_grad = Types::Vec3::Zero();
                for (int axis = 0; axis < 3; ++axis)
                {
                    Types::Vec3 step = Types::Vec3::Zero();
                    step(axis) = res;
                    const double occ_pos = (grid_map->getInflateOccupancy(q + step) != 0) ? 1.0 : 0.0;
                    const double occ_neg = (grid_map->getInflateOccupancy(q - step) != 0) ? 1.0 : 0.0;
                    occ_grad(axis) = occ_pos - occ_neg;
                }

                if (occ_grad.norm() < 1.0e-3)
                {
                    occ_grad = drone_pos - q;
                }
                if (occ_grad.norm() < 1.0e-3)
                {
                    occ_grad = q - target_pos;
                }
                if (occ_grad.norm() > 1.0e-6)
                {
                    occ_grad.normalize();
                    const Types::Vec3 grad_q =
                        weight_los * occ_grad / static_cast<double>(sample_num);
                    grad_drone_pos += (1.0 - alpha) * grad_q;
                }
            }
        }

        return cost;
    }
}

#endif
