#ifndef CORRIDOR_COST_MANAGER_HPP
#define CORRIDOR_COST_MANAGER_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoFeasibilityPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoSwarmPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/VarianceSampleCost.hpp"
#include "CostFunctionalManager/CostFunctional/TemporalCosts/LinearTimeCost.hpp"
#include "SpatialMap/SFCCommonTypes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cost_functional
{
    class CorridorCostFunctionalManager
    {
    public:
        using Types = cost_functional::PlanningTypesAdapter;
        using PolyhedraH = spatial_map::PolyhedraH;

        const PolyhedraH *h_polys;
        const Eigen::VectorXi *segment_poly_idx;
        Types::ConstraintPoints *cps;
        Types::SwarmTrajData *swarm_traj;
        double wei_corridor, wei_swarm, wei_feas, wei_sqrvar;
        double corridor_clearance, corridor_smoothing, swarm_clearance;
        double max_vel, max_acc, max_jer;
        int drone_id;
        double t_now;
        bool touch_goal;
        mutable std::vector<double> *min_ellip_dist2_ptr;
        mutable Eigen::VectorXd accumulated_costs;

        CorridorCostFunctionalManager()
            : h_polys(nullptr), segment_poly_idx(nullptr),
              cps(nullptr), swarm_traj(nullptr),
              wei_corridor(0.0), wei_swarm(0.0), wei_feas(0.0), wei_sqrvar(0.0),
              corridor_clearance(0.0), corridor_smoothing(0.05), swarm_clearance(0.0),
              max_vel(0.0), max_acc(0.0), max_jer(0.0),
              drone_id(-1), t_now(0.0), touch_goal(false),
              min_ellip_dist2_ptr(nullptr)
        {
            accumulated_costs.resize(4);
            accumulated_costs.setZero();
        }

        void setCorridor(const PolyhedraH *polys, const Eigen::VectorXi *indices)
        {
            h_polys = polys;
            segment_poly_idx = indices;
        }

        void resetAccumulation() const
        {
            accumulated_costs.setZero();
        }

        double evaluateIntegral(const int cp_idx,
                                const double /*t_local*/,
                                const double t_global,
                                const int seg_idx,
                                const int /*step_in_seg*/,
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
                cost_functional::accumulateFeasibilityPenalty(
                    velocity, acceleration, jerk,
                    max_vel, max_acc, max_jer,
                    wei_feas, grad_velocity, grad_acceleration, grad_jerk);

            const double cost_corridor =
                accumulateCorridorPenalty(seg_idx, position, grad_position);

            const double cost_swarm =
                cost_functional::accumulateSwarmPenalty(
                    cp_idx, cps, touch_goal, swarm_traj, drone_id,
                    t_now, t_global, swarm_clearance, wei_swarm,
                    position, grad_position, grad_time, min_ellip_dist2_ptr);

            accumulated_costs(0) += cost_feasibility;
            accumulated_costs(1) += cost_corridor;
            accumulated_costs(2) += cost_swarm;

            return cost_feasibility + cost_corridor + cost_swarm;
        }

        template <typename SamplesType>
        double evaluateSample(const SamplesType &samples,
                              Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_p,
                              Eigen::VectorXd &grad_t_global) const
        {
            grad_p.setZero(3, samples.size());
            grad_t_global.setZero(samples.size());

            if (wei_sqrvar <= 0.0 || samples.empty())
            {
                return 0.0;
            }

            const int point_num = samples.back().logical_idx + 1;
            if (point_num < 2)
            {
                return 0.0;
            }

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
            const double cost_smooth =
                cost_functional::accumulateVarianceSampleCost(points, wei_sqrvar, grad_points);

            for (int logical_idx = 0; logical_idx < point_num; ++logical_idx)
            {
                const int sample_idx = canonical_sample[logical_idx];
                if (sample_idx >= 0)
                {
                    grad_p.col(sample_idx) += grad_points.row(logical_idx).transpose();
                }
            }

            accumulated_costs(3) += cost_smooth;
            return cost_smooth;
        }

    private:
        double accumulatePolyPenalty(const spatial_map::PolyhedronH &poly,
                                     const Types::Vec3 &position,
                                     Types::Vec3 &grad_position) const
        {
            const double smooth_eps = std::max(corridor_smoothing, 1.0e-6);

            double cost = 0.0;
            for (int i = 0; i < poly.rows(); ++i)
            {
                const Types::Vec3 outer_normal = poly.row(i).head<3>().transpose();
                if (outer_normal.squaredNorm() <= 1.0e-12)
                {
                    continue;
                }

                const double violation =
                    outer_normal.dot(position) + poly(i, 3) + corridor_clearance;

                double penalty = 0.0;
                double penalty_grad = 0.0;
                if (!cost_functional::smoothedL1(violation, smooth_eps, penalty, penalty_grad))
                {
                    continue;
                }

                grad_position += wei_corridor * penalty_grad * outer_normal;
                cost += wei_corridor * penalty;
            }

            return cost;
        }

        int resolvePolyId(const int seg_idx) const
        {
            if (!h_polys || h_polys->empty())
            {
                return -1;
            }

            if (!segment_poly_idx || segment_poly_idx->size() <= 0)
            {
                return std::min(std::max(seg_idx, 0), static_cast<int>(h_polys->size()) - 1);
            }

            const int idx_count = static_cast<int>(segment_poly_idx->size());
            const int idx_pos = std::min(std::max(seg_idx, 0), idx_count - 1);
            const int poly_id = (*segment_poly_idx)(idx_pos);
            return std::min(std::max(poly_id, 0), static_cast<int>(h_polys->size()) - 1);
        }

        double accumulateCorridorPenalty(const int seg_idx,
                                         const Types::Vec3 &position,
                                         Types::Vec3 &grad_position) const
        {
            if (wei_corridor <= 0.0)
            {
                return 0.0;
            }

            const int poly_id = resolvePolyId(seg_idx);
            if (poly_id < 0)
            {
                return 0.0;
            }

            return accumulatePolyPenalty((*h_polys)[poly_id], position, grad_position);
        }
    };

} // namespace cost_functional

#endif
