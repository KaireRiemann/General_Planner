#ifndef TRACKING_COST_MANAGER_HPP
#define TRACKING_COST_MANAGER_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/TrackingTypes.hpp"
#include "CostFunctionalManager/TrackingSemanticGuide.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/DistanceFieldObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/DistanceKeepingCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoFeasibilityPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoSwarmPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/GuidePointObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/SoftTerminalReferenceCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/TrackingViewReferenceCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/TrackingLosCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/TrackingVisibleRegionCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/VarianceSampleCost.hpp"

namespace cost_functional
{
    class TrackingCostFunctionalManager
    {
    public:
        using Types = cost_functional::PlanningTypesAdapter;
        using RefType = cost_functional::TrackingReference;

        enum SpatialMode
        {
            SPATIAL_PLAIN = 0,
            SPATIAL_ESDF
        };

        Types::GridMapPtr grid_map;
        Types::ConstraintPoints *cps;
        Types::SwarmTrajData *swarm_traj;
        const RefType *tracking_ref;
        const TrackingSemanticGuide *tracking_semantic_guide;

        SpatialMode spatial_mode;

        double wei_obs, wei_obs_soft, wei_dist, wei_swarm, wei_feas, wei_sqrvar;
        double obs_clearance, obs_clearance_soft, safe_margin, swarm_clearance;
        double max_vel, max_acc, max_jer;

        double track_d_min, track_d_max, track_z_tol, track_smooth_eps;
        double wei_track_near, wei_track_far, wei_track_vertical;

        // viewpoint reference cost
        double wei_track_view_xy, wei_track_view_z;

        // LoS cost
        double wei_track_los;
        double track_los_clearance;
        double wei_track_visible_fan;
        double wei_track_view_dir_smooth;

        // terminal
        double wei_terminal_pos, wei_terminal_vel;

        int drone_id;
        double t_now;
        bool touch_goal;
        int cps_per_piece;

        mutable std::vector<double> *min_ellip_dist2_ptr;
        mutable Eigen::VectorXd accumulated_costs;

        TrackingCostFunctionalManager()
            : grid_map(nullptr), cps(nullptr), swarm_traj(nullptr), tracking_ref(nullptr), tracking_semantic_guide(nullptr),
              spatial_mode(SPATIAL_PLAIN),
              wei_obs(0.0), wei_obs_soft(0.0), wei_dist(0.0), wei_swarm(0.0), wei_feas(0.0), wei_sqrvar(0.0),
              obs_clearance(0.0), obs_clearance_soft(0.0), safe_margin(0.0), swarm_clearance(0.0),
              max_vel(0.0), max_acc(0.0), max_jer(0.0),
              track_d_min(0.0), track_d_max(0.0), track_z_tol(0.4), track_smooth_eps(0.1),
              wei_track_near(0.0), wei_track_far(0.0), wei_track_vertical(0.0),
              wei_track_view_xy(0.0), wei_track_view_z(0.0),
              wei_track_los(0.0), track_los_clearance(0.2),
              wei_track_visible_fan(0.0), wei_track_view_dir_smooth(0.0),
              wei_terminal_pos(0.0), wei_terminal_vel(0.0),
              drone_id(-1), t_now(0.0), touch_goal(false), cps_per_piece(5),
              min_ellip_dist2_ptr(nullptr)
        {
            accumulated_costs.resize(10);
            accumulated_costs.setZero();
        }

        void setSpatialMode(const SpatialMode mode) { spatial_mode = mode; }
        void setTrackingReference(const RefType *ref) { tracking_ref = ref; }
        void setTrackingSemanticGuide(const TrackingSemanticGuide *guide) { tracking_semantic_guide = guide; }
        void resetAccumulation() const { accumulated_costs.setZero(); }

        double evaluateIntegral(const int cp_idx,
                                const double,
                                const double t_global,
                                const int,
                                const int,
                                const Types::Vec3 &position,
                                const Types::Vec3 &velocity,
                                const Types::Vec3 &acceleration,
                                const Types::Vec3 &jerk,
                                Types::Vec3 &grad_position,
                                Types::Vec3 &grad_velocity,
                                Types::Vec3 &grad_acceleration,
                                Types::Vec3 &grad_jerk,
                                double &grad_time) const
        {
            const double cost_feasibility =
                accumulateFeasibilityPenalty(velocity, acceleration, jerk,
                                             max_vel, max_acc, max_jer,
                                             wei_feas, grad_velocity, grad_acceleration, grad_jerk);

            double cost_spatial = 0.0;
            if (spatial_mode == SPATIAL_ESDF)
            {
                if (grid_map != nullptr && grid_map->esdfEnabled())
                {
                    Types::Vec3 sdf_grad = Types::Vec3::Zero();
                    const double sdf_value = grid_map->getDistWithGradTrilinear(position, sdf_grad);
                    cost_spatial =
                        accumulateDistanceFieldObstaclePenalty(position, sdf_value, sdf_grad,
                                                               safe_margin, wei_dist, grad_position);
                }
            }
            else
            {
                cost_spatial =
                    accumulateObstaclePenalty(cp_idx, cps, touch_goal, position,
                                              obs_clearance, obs_clearance_soft,
                                              wei_obs, wei_obs_soft, grad_position);
            }

            const double cost_swarm =
                accumulateSwarmPenalty(cp_idx, cps, touch_goal, swarm_traj, drone_id,
                                       t_now, t_global, swarm_clearance, wei_swarm,
                                       position, grad_position, grad_time, min_ellip_dist2_ptr);

            double cost_distance = 0.0;
            double cost_view_ref = 0.0;
            double cost_los = 0.0;
            double cost_visible_fan = 0.0;

            if (tracking_ref != nullptr)
            {
                Types::Vec3 ref_pos = Types::Vec3::Zero();
                Types::Vec3 ref_vel = Types::Vec3::Zero();

                if (sampleTrackingReference(*tracking_ref, t_global, ref_pos, ref_vel))
                {
                    Types::Vec3 grad_track_pos = Types::Vec3::Zero();
                    cost_distance =
                        accumulateDistanceKeepingCost(position, ref_pos,
                                                      track_d_min, track_d_max,
                                                      track_z_tol, track_smooth_eps,
                                                      wei_track_near, wei_track_far, wei_track_vertical,
                                                      grad_track_pos);
                    grad_position += grad_track_pos;
                    grad_time += -grad_track_pos.dot(ref_vel);

                    Types::Vec3 grad_los = Types::Vec3::Zero();
                    cost_los =
                        accumulateTrackingLoSCost(position, ref_pos, grid_map,
                                                  track_los_clearance, wei_track_los, grad_los);
                    grad_position += grad_los;
                    grad_time += -grad_los.dot(ref_vel);
                }

                Types::Vec3 view_pos = Types::Vec3::Zero();
                Types::Vec3 view_vel = Types::Vec3::Zero();
                if (tracking_ref->viewValid() &&
                    sampleTrackingViewReference(*tracking_ref, t_global, view_pos, view_vel))
                {
                    Types::Vec3 grad_view = Types::Vec3::Zero();
                    cost_view_ref =
                        accumulateTrackingViewReferenceCost(position, view_pos,
                                                            wei_track_view_xy, wei_track_view_z,
                                                            grad_view);
                    grad_position += grad_view;
                    grad_time += -grad_view.dot(view_vel);
                }
            }

            if (tracking_semantic_guide != nullptr)
            {
                VisibleFanRegion region;
                if (semantic_guide::sampleVisibleFanRegion(*tracking_semantic_guide, t_global, region))
                {
                    Types::Vec3 grad_visible = Types::Vec3::Zero();
                    cost_visible_fan =
                        accumulateVisibleFanRegionViolationCost(position,
                                                                region,
                                                                wei_track_visible_fan,
                                                                grad_visible);
                    grad_position += grad_visible;
                    grad_time += -grad_visible.dot(region.target_velocity);
                }
            }

            accumulated_costs(0) += cost_feasibility;
            accumulated_costs(1) += cost_spatial;
            accumulated_costs(2) += cost_swarm;
            accumulated_costs(4) += cost_distance;
            accumulated_costs(5) += cost_view_ref;
            accumulated_costs(6) += cost_los;
            accumulated_costs(7) += cost_visible_fan;

            return cost_feasibility + cost_spatial + cost_swarm +
                   cost_distance + cost_view_ref + cost_los + cost_visible_fan;
        }

        template <typename SamplesType>
        double evaluateSample(const SamplesType &samples,
                              Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_p,
                              Eigen::VectorXd &grad_t_global) const
        {
            grad_p.setZero(3, samples.size());
            grad_t_global.setZero(samples.size());

            double cost_smooth = 0.0;
            if (wei_sqrvar > 0.0 && !samples.empty())
            {
                const int point_num = samples.back().logical_idx + 1;
                if (point_num >= 2)
                {
                    Eigen::MatrixXd points(point_num, 3);
                    points.setZero();
                    std::vector<int> canonical_sample(point_num, -1);
                    for (int i = 0; i < static_cast<int>(samples.size()); ++i)
                    {
                        const int logical_idx = samples[i].logical_idx;
                        points.row(logical_idx) = samples[i].p.transpose();
                        if (canonical_sample[logical_idx] < 0)
                        {
                            canonical_sample[logical_idx] = i;
                        }
                    }

                    Eigen::MatrixXd grad_points = Eigen::MatrixXd::Zero(point_num, 3);
                    cost_smooth = accumulateVarianceSampleCost(points, wei_sqrvar, grad_points);

                    for (int logical_idx = 0; logical_idx < point_num; ++logical_idx)
                    {
                        const int sample_idx = canonical_sample[logical_idx];
                        if (sample_idx >= 0)
                        {
                            grad_p.col(sample_idx) += grad_points.row(logical_idx).transpose();
                        }
                    }
                }
            }

            double cost_view_dir_smooth = 0.0;
            if (tracking_semantic_guide != nullptr &&
                wei_track_view_dir_smooth > 0.0 &&
                samples.size() >= 2)
            {
                for (int i = 1; i < static_cast<int>(samples.size()); ++i)
                {
                    VisibleFanRegion prev_region, curr_region;
                    if (!semantic_guide::sampleVisibleFanRegion(*tracking_semantic_guide,
                                                                samples[i - 1].t_global,
                                                                prev_region) ||
                        !semantic_guide::sampleVisibleFanRegion(*tracking_semantic_guide,
                                                                samples[i].t_global,
                                                                curr_region))
                    {
                        continue;
                    }

                    Types::Vec3 grad_prev = Types::Vec3::Zero();
                    Types::Vec3 grad_curr = Types::Vec3::Zero();
                    cost_view_dir_smooth +=
                        accumulateTrackingViewDirectionSmoothnessCost(samples[i - 1].p,
                                                                      samples[i].p,
                                                                      prev_region,
                                                                      curr_region,
                                                                      wei_track_view_dir_smooth,
                                                                      grad_prev,
                                                                      grad_curr);
                    grad_p.col(i - 1) += grad_prev;
                    grad_p.col(i) += grad_curr;
                }
            }

            double cost_terminal = 0.0;
            if (tracking_ref != nullptr &&
                (wei_terminal_pos > 0.0 || wei_terminal_vel > 0.0) &&
                !samples.empty())
            {
                Types::Vec3 ref_terminal_pos = Types::Vec3::Zero();
                Types::Vec3 ref_terminal_vel = Types::Vec3::Zero();
                if (sampleTrackingTerminalReference(*tracking_ref, ref_terminal_pos, ref_terminal_vel))
                {
                    const int terminal_idx = static_cast<int>(samples.size()) - 1;

                    Types::Vec3 terminal_vel_est = Types::Vec3::Zero();
                    int prev_idx = -1;
                    double terminal_dt = 0.0;
                    if (samples.size() >= 2)
                    {
                        prev_idx = terminal_idx - 1;
                        terminal_dt = std::max(samples[terminal_idx].t_global - samples[prev_idx].t_global, 1.0e-3);
                        terminal_vel_est = (samples[terminal_idx].p - samples[prev_idx].p) / terminal_dt;
                    }

                    Types::Vec3 grad_terminal_pos = Types::Vec3::Zero();
                    Types::Vec3 grad_terminal_vel = Types::Vec3::Zero();
                    cost_terminal =
                        accumulateSoftTerminalReferenceCost(samples[terminal_idx].p,
                                                            terminal_vel_est,
                                                            ref_terminal_pos,
                                                            ref_terminal_vel,
                                                            wei_terminal_pos,
                                                            wei_terminal_vel,
                                                            grad_terminal_pos,
                                                            grad_terminal_vel);

                    grad_p.col(terminal_idx) += grad_terminal_pos;

                    if (prev_idx >= 0)
                    {
                        const Types::Vec3 grad_terminal_diff = grad_terminal_vel / terminal_dt;
                        grad_p.col(terminal_idx) += grad_terminal_diff;
                        grad_p.col(prev_idx) -= grad_terminal_diff;

                        const double grad_dt_term = grad_terminal_vel.dot(terminal_vel_est) / terminal_dt;
                        grad_t_global(terminal_idx) -= grad_dt_term;
                        grad_t_global(prev_idx) += grad_dt_term;
                    }
                }
            }

            accumulated_costs(3) += cost_smooth;
            accumulated_costs(8) += cost_view_dir_smooth;
            accumulated_costs(9) += cost_terminal;
            return cost_smooth + cost_view_dir_smooth + cost_terminal;
        }
    };
}

#endif
