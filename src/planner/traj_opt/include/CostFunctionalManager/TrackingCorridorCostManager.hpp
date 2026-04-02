#ifndef TRACKING_CORRIDOR_COST_MANAGER_HPP
#define TRACKING_CORRIDOR_COST_MANAGER_HPP

#include "CostFunctionalManager/CorridorCostManager.hpp"
#include "CostFunctionalManager/TrackingTypes.hpp"
#include "CostFunctionalManager/TrackingSemanticGuide.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/DistanceKeepingCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/SoftTerminalReferenceCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/TrackingViewReferenceCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/TrackingLosCost.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/TrackingVisibleRegionCost.hpp"

namespace cost_functional
{
    class TrackingCorridorCostFunctionalManager : public CorridorCostFunctionalManager
    {
    public:
        using Base = CorridorCostFunctionalManager;
        using RefType = TrackingReference;
        using Types = Base::Types;

        Types::GridMapPtr grid_map{nullptr};
        const RefType *tracking_ref{nullptr};
        const TrackingSemanticGuide *tracking_semantic_guide{nullptr};

        double track_d_min{0.0}, track_d_max{0.0}, track_z_tol{0.4}, track_smooth_eps{0.1};
        double wei_track_near{0.0}, wei_track_far{0.0}, wei_track_vertical{0.0};

        double wei_track_view_xy{0.0}, wei_track_view_z{0.0};

        double wei_track_los{0.0};
        double track_los_clearance{0.2};
        double wei_track_visible_fan{0.0};
        double wei_track_view_dir_smooth{0.0};

        double wei_terminal_pos{0.0}, wei_terminal_vel{0.0};

        TrackingCorridorCostFunctionalManager()
        {
            accumulated_costs.resize(11);
            accumulated_costs.setZero();
        }

        void setTrackingReference(const RefType *ref) { tracking_ref = ref; }
        void setTrackingSemanticGuide(const TrackingSemanticGuide *guide) { tracking_semantic_guide = guide; }
        void resetAccumulation() const { accumulated_costs.setZero(); }

        double evaluateIntegral(const int cp_idx,
                                const double t_local,
                                const double t_global,
                                const int seg_idx,
                                const int step_in_seg,
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
            const double base_cost =
                Base::evaluateIntegral(cp_idx, t_local, t_global, seg_idx, step_in_seg,
                                       position, velocity, acceleration, jerk,
                                       grad_position, grad_velocity, grad_acceleration, grad_jerk, grad_time);

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
                    Types::Vec3 grad_track = Types::Vec3::Zero();
                    cost_distance =
                        accumulateDistanceKeepingCost(position, ref_pos,
                                                      track_d_min, track_d_max,
                                                      track_z_tol, track_smooth_eps,
                                                      wei_track_near, wei_track_far, wei_track_vertical,
                                                      grad_track);
                    grad_position += grad_track;
                    grad_time += -grad_track.dot(ref_vel);

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

            accumulated_costs(5) += cost_distance;
            accumulated_costs(6) += cost_view_ref;
            accumulated_costs(7) += cost_los;
            accumulated_costs(8) += cost_visible_fan;
            return base_cost + cost_distance + cost_view_ref + cost_los + cost_visible_fan;
        }

        template <typename SamplesType>
        double evaluateSample(const SamplesType &samples,
                              Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_p,
                              Eigen::VectorXd &grad_t_global) const
        {
            double cost = Base::evaluateSample(samples, grad_p, grad_t_global);

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

            accumulated_costs(9) += cost_view_dir_smooth;
            accumulated_costs(10) += cost_terminal;
            return cost + cost_view_dir_smooth + cost_terminal;
        }
    };
}

#endif
