#ifndef EGO_SWARM_PENALTY_HPP
#define EGO_SWARM_PENALTY_HPP

#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    using Types = PlanningTypesAdapter;
    
    inline double accumulateSwarmPenalty(const int cp_idx,
                                        Types::ConstraintPoints *cps,
                                        const bool touch_goal,
                                        const Types::SwarmTrajData *swarm_trajs,
                                        const int drone_id,
                                        const double t_now,
                                        const double t_global,
                                        const double swarm_clearance,
                                        const double weight_swarm,
                                        const Types::Vec3 &position,
                                        Types::Vec3 &grad_position,
                                        double &grad_time,
                                        std::vector<double> *min_ellip_dist2_ptr = nullptr)
    {
        if(weight_swarm <= 0.0)
        {
            return 0.0;
        }

        if (!swarm_trajs || !cps || cp_idx <= 0 || cp_idx >= cps->cp_size ||
        cp_idx > Types::ConstraintPoints::two_thirds_id(cps->points, touch_goal))
        {
            return 0.0;
        }

        double cost = 0.0;
        constexpr double a_param = 2.0;
        constexpr double b_param = 1.0;
        constexpr double inv_a2 = 1.0 / (a_param * a_param);
        constexpr double inv_b2 = 1.0 / (b_param * b_param);

        for (size_t id = 0; id < swarm_trajs->size(); ++id)
        {
            if ((*swarm_trajs)[id].drone_id < 0 || (*swarm_trajs)[id].drone_id == drone_id)
            {
                continue;
            }

            const double traj_i_start_time = (*swarm_trajs)[id].start_time;
            const double point_time = (t_now - traj_i_start_time) + t_global;
            const double clearance = (swarm_clearance + (*swarm_trajs)[id].des_clearance) * 1.5;
            const double clearance2 = clearance * clearance;

            Types::Vec3 swarm_p;
            Types::Vec3 swarm_v;
            if (point_time < (*swarm_trajs)[id].duration)
            {
                swarm_p = (*swarm_trajs)[id].traj.evaluate(0.0 + point_time,0);
                swarm_v = (*swarm_trajs)[id].traj.evaluate(0.0 + point_time,1);
            }
            else
            {
                const double end_t = 0.0 + (*swarm_trajs)[id].duration;
                swarm_v = (*swarm_trajs)[id].traj.evaluate(end_t, 1);
                swarm_p = (*swarm_trajs)[id].traj.evaluate(end_t, 0) +
                        (point_time - (*swarm_trajs)[id].duration) * swarm_v;
            }

            const Types::Vec3 dist_vec = position - swarm_p;
            const double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 +
                                    (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
            const double dist2_err = clearance2 - ellip_dist2;

            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (positivePartCubic(dist2_err, penalty, penalty_grad))
            {
                cost += weight_swarm * penalty;
                const Types::Vec3 d_dist2_d_pos(inv_b2 * dist_vec(0),
                                                inv_b2 * dist_vec(1),
                                                inv_a2 * dist_vec(2));
                const Types::Vec3 d_cost_d_pos = weight_swarm * penalty_grad * (-2.0) * d_dist2_d_pos;
                grad_position += d_cost_d_pos;
                grad_time += d_cost_d_pos.dot(-swarm_v);
            }

            if (min_ellip_dist2_ptr && id < min_ellip_dist2_ptr->size() &&
                (*min_ellip_dist2_ptr)[id] > ellip_dist2)
            {
                (*min_ellip_dist2_ptr)[id] = ellip_dist2;
            }
        }

        return cost;
    }

}//namespace cost_functional

#endif