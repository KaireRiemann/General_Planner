#ifndef DISTANCE_FIELD_COST_MANAGER_HPP
#define DISTANCE_FIELD_COST_MANAGER_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/DistanceFieldObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoFeasibilityPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoSwarmPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/VarianceSampleCost.hpp"
#include <vector>

namespace cost_functional
{
    class DistanceFieldCostFunctionalManager
    {
    public:
        using Types = cost_functional::PlanningTypesAdapter;

        Types::GridMapPtr grid_map;
        Types::ConstraintPoints *cps;
        Types::SwarmTrajData *swarm_traj;
        double wei_dist, wei_swarm, wei_feas, wei_sqrvar;
        double safe_margin, swarm_clearance;
        double max_vel, max_acc, max_jer;
        int drone_id;
        double t_now;
        bool touch_goal;
        mutable std::vector<double> *min_ellip_dist2_ptr;
        mutable Eigen::VectorXd accumulated_costs;

        DistanceFieldCostFunctionalManager()
            : grid_map(nullptr), cps(nullptr), swarm_traj(nullptr),
              wei_dist(0.0), wei_swarm(0.0), wei_feas(0.0), wei_sqrvar(0.0),
              safe_margin(0.0), swarm_clearance(0.0),
              max_vel(0.0), max_acc(0.0), max_jer(0.0),
              drone_id(-1), t_now(0.0), touch_goal(false),
              min_ellip_dist2_ptr(nullptr)
        {
            accumulated_costs.resize(4);
            accumulated_costs.setZero();
        }

        void resetAccumulation() const
        {
            accumulated_costs.setZero();
        }

        double evaluateIntegral(const int cp_idx,
                                const double /*t_local*/,
                                const double t_global,
                                const int /*seg_idx*/,
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

            double cost_distance_field = 0.0;
            if (grid_map != nullptr && grid_map->esdfEnabled())
            {
                Types::Vec3 sdf_grad = Types::Vec3::Zero();
                const double sdf_value = grid_map->getDistWithGradTrilinear(position, sdf_grad);
                cost_distance_field =
                    cost_functional::accumulateDistanceFieldObstaclePenalty(
                        position, sdf_value, sdf_grad, safe_margin,
                        wei_dist, grad_position);
            }

            const double cost_swarm =
                cost_functional::accumulateSwarmPenalty(
                    cp_idx, cps, touch_goal, swarm_traj, drone_id,
                    t_now, t_global, swarm_clearance, wei_swarm,
                    position, grad_position, grad_time, min_ellip_dist2_ptr);

            accumulated_costs(0) += cost_feasibility;
            accumulated_costs(1) += cost_distance_field;
            accumulated_costs(2) += cost_swarm;

            return cost_feasibility + cost_distance_field + cost_swarm;
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
    };
} // namespace cost_functional

#endif
