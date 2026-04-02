#ifndef TRACKING_VISIBLE_REGION_COST_HPP
#define TRACKING_VISIBLE_REGION_COST_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/TrackingSemanticGuide.hpp"
#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"

#include <algorithm>
#include <cmath>

namespace cost_functional
{
    using Types = PlanningTypesAdapter;

    inline Eigen::Vector2d safeNormalizedXY(const Eigen::Vector2d &v,
                                            const Eigen::Vector2d &fallback = Eigen::Vector2d::UnitX())
    {
        const double n = v.norm();
        if (n > 1.0e-6)
        {
            return v / n;
        }
        return fallback.normalized();
    }

    inline double accumulateVisibleFanRegionViolationCost(const Types::Vec3 &position,
                                                          const VisibleFanRegion &region,
                                                          const double weight_visible_fan,
                                                          Types::Vec3 &grad_position)
    {
        if (weight_visible_fan <= 0.0 || !region.valid || !position.allFinite())
        {
            return 0.0;
        }

        const Types::Vec3 rel = position - region.target_position;
        const Eigen::Vector2d rel_xy = rel.head<2>();
        const Eigen::Vector2d pref_xy = (region.preferred_viewpoint - region.target_position).head<2>();
        const Eigen::Vector2d fallback_dir = safeNormalizedXY(pref_xy);
        const double dist_xy = rel_xy.norm();

        const double smooth_eps = 0.08;
        double cost = 0.0;

        if (region.min_tracking_distance > 1.0e-6)
        {
            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (smoothedL1(region.min_tracking_distance - dist_xy, smooth_eps, penalty, penalty_grad))
            {
                const Eigen::Vector2d grad_xy = -penalty_grad * safeNormalizedXY(rel_xy, fallback_dir);
                grad_position.head<2>() += weight_visible_fan * grad_xy;
                cost += weight_visible_fan * penalty;
            }
        }

        if (region.max_tracking_distance > 1.0e-6)
        {
            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (smoothedL1(dist_xy - region.max_tracking_distance, smooth_eps, penalty, penalty_grad))
            {
                const Eigen::Vector2d grad_xy = penalty_grad * safeNormalizedXY(rel_xy, fallback_dir);
                grad_position.head<2>() += weight_visible_fan * grad_xy;
                cost += weight_visible_fan * penalty;
            }
        }

        const double yaw = std::atan2(rel.y(), rel.x());
        const double yaw_err =
            semantic_guide::angleDiff(yaw, region.yaw_center);
        const double yaw_violation = std::abs(yaw_err) - region.yaw_half_span;
        if (yaw_violation > -smooth_eps)
        {
            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (smoothedL1(yaw_violation, smooth_eps, penalty, penalty_grad))
            {
                const double r2 = std::max(rel_xy.squaredNorm(), 1.0e-4);
                const double sign = (yaw_err >= 0.0) ? 1.0 : -1.0;
                const Eigen::Vector2d grad_yaw(-rel.y() / r2, rel.x() / r2);
                grad_position.head<2>() += weight_visible_fan * penalty_grad * sign * grad_yaw;
                cost += weight_visible_fan * penalty;
            }
        }

        const double z_err = std::abs(position.z() - region.z_center) - region.z_half_span;
        if (z_err > -smooth_eps)
        {
            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (smoothedL1(z_err, smooth_eps, penalty, penalty_grad))
            {
                const double sign = (position.z() >= region.z_center) ? 1.0 : -1.0;
                grad_position.z() += weight_visible_fan * penalty_grad * sign;
                cost += weight_visible_fan * penalty;
            }
        }

        return cost;
    }

    inline double accumulateTrackingViewDirectionSmoothnessCost(const Types::Vec3 &prev_position,
                                                               const Types::Vec3 &curr_position,
                                                               const VisibleFanRegion &prev_region,
                                                               const VisibleFanRegion &curr_region,
                                                               const double weight_view_dir_smooth,
                                                               Types::Vec3 &grad_prev_position,
                                                               Types::Vec3 &grad_curr_position) 
    {
        if (weight_view_dir_smooth <= 0.0 || !prev_region.valid || !curr_region.valid)
        {
            return 0.0;
        }

        const Eigen::Vector2d prev_rel = (prev_position - prev_region.target_position).head<2>();
        const Eigen::Vector2d curr_rel = (curr_position - curr_region.target_position).head<2>();
        if (prev_rel.norm() < 1.0e-4 || curr_rel.norm() < 1.0e-4)
        {
            return 0.0;
        }

        const double prev_yaw = std::atan2(prev_rel.y(), prev_rel.x());
        const double curr_yaw = std::atan2(curr_rel.y(), curr_rel.x());
        const double curr_delta = semantic_guide::angleDiff(curr_yaw, prev_yaw);
        const double ref_delta =
            semantic_guide::angleDiff(curr_region.yaw_center, prev_region.yaw_center);
        const double err = semantic_guide::angleDiff(curr_delta, ref_delta);

        const double cost = 0.5 * weight_view_dir_smooth * err * err;
        const double r2_prev = std::max(prev_rel.squaredNorm(), 1.0e-4);
        const double r2_curr = std::max(curr_rel.squaredNorm(), 1.0e-4);
        const Eigen::Vector2d grad_prev_yaw(-prev_rel.y() / r2_prev, prev_rel.x() / r2_prev);
        const Eigen::Vector2d grad_curr_yaw(-curr_rel.y() / r2_curr, curr_rel.x() / r2_curr);

        grad_prev_position.head<2>() -= weight_view_dir_smooth * err * grad_prev_yaw;
        grad_curr_position.head<2>() += weight_view_dir_smooth * err * grad_curr_yaw;
        return cost;
    }
}

#endif
