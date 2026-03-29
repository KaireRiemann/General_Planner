#ifndef EGO_COST_MANAGER_HPP
#define EGO_COST_MANAGER_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoFeasibilityPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/GuidePointObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoSwarmPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/VarianceSampleCost.hpp"
#include "CostFunctionalManager/CostFunctional/TemporalCosts/LinearTimeCost.hpp"


namespace cost_functional
{
    class EgoCostFunctionalManager
    {
    public:
        using Types = cost_functional::PlanningTypesAdapter;

        Types::GridMapPtr grid_map;
        Types::ConstraintPoints *cps;
        Types::SwarmTrajData *swarm_traj;
        double wei_obs, wei_obs_soft, wei_swarm, wei_feas, wei_sqrvar;
        double obs_clearance, obs_clearance_soft, swarm_clearance;
        double max_vel, max_acc, max_jer;
        int drone_id;
        double t_now;
        bool touch_goal;
        int cps_per_piece;
        mutable std::vector<double> *min_ellip_dist2_ptr;
        mutable Eigen::VectorXd accumulated_costs;
        mutable std::vector<double> segment_dt_;

        EgoCostFunctionalManager() : grid_map(nullptr), cps(nullptr), swarm_traj(nullptr),
                                     wei_obs(0), wei_obs_soft(0), wei_swarm(0), wei_feas(0), wei_sqrvar(0),
                                     obs_clearance(0), obs_clearance_soft(0), swarm_clearance(0),
                                     max_vel(0), max_acc(0), max_jer(0),
                                     drone_id(-1), t_now(0), touch_goal(false), cps_per_piece(5),
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

            const double cost_obstacle =
                cost_functional::accumulateObstaclePenalty(
                    cp_idx, cps, touch_goal, position,
                    obs_clearance, obs_clearance_soft,
                    wei_obs, wei_obs_soft, grad_position);

            const double cost_swarm =
                cost_functional::accumulateSwarmPenalty(
                    cp_idx, cps, touch_goal, swarm_traj, drone_id,
                    t_now, t_global, swarm_clearance, wei_swarm,
                    position, grad_position, grad_time, min_ellip_dist2_ptr);

            accumulated_costs(0) += cost_feasibility;
            accumulated_costs(1) += cost_obstacle;
            accumulated_costs(2) += cost_swarm;

            return cost_feasibility + cost_obstacle + cost_swarm;
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

}//namespace cost_functional

#endif
