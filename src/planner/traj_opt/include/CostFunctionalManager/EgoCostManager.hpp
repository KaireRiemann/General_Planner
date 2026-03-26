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

        void operator()(const Eigen::MatrixXd &C, 
                        const Eigen::MatrixXd &V, 
                        const Eigen::MatrixXd &A, 
                        const Eigen::MatrixXd &J,
                        double &total_cost, 
                        Eigen::MatrixXd &gdC, 
                        Eigen::MatrixXd &gdV, 
                        Eigen::MatrixXd &gdA, 
                        Eigen::MatrixXd &gdJ) const
        {
            total_cost = 0.0;
            double cost_guide = 0.0;
            double cost_feasibility = 0.0;
            double cost_swarm = 0.0;
            double cost_smooth = 0.0;

            //feasibility
            // for(int i = 0; i <= V.rows(); i++)
            // {
            //     Types::Vec3 v_row = V.row(i).transpose();

            //     Types::Vec3 a_row = (i < A.rows()) ? Types::Vec3(A.row(i).transpose()) : Types::Vec3::Zero();
            //     Types::Vec3 j_row = (i < J.rows()) ? Types::Vec3(J.row(i).transpose()) : Types::Vec3::Zero();

            //     Types::Vec3 grad_v = Types::Vec3::Zero();
            //     Types::Vec3 grad_a = Types::Vec3::Zero();
            //     Types::Vec3 grad_j = Types::Vec3::Zero();

                   
            //     cost_feasibility += cost_functional::accumulateFeasibilityPenalty(
            //         v_row, a_row, j_row, 
            //         max_vel, max_acc, max_jer, 
            //         wei_feas, 
            //         grad_v, grad_a, grad_j
            //     );


            //     gdV.row(i) += grad_v.transpose();
            //     if (i < A.rows()) gdA.row(i) += grad_a.transpose();
            //     if (i < J.rows()) gdJ.row(i) += grad_j.transpose();
            // }
            // feasibility
            for(int i = 0; i < V.rows(); i++)
            {
                Types::Vec3 v_row = V.row(i).transpose();
                
                Types::Vec3 a_row = Types::Vec3::Zero();
                if (i < A.rows()) 
                {
                    a_row = A.row(i).transpose();
                }
                
                Types::Vec3 j_row = Types::Vec3::Zero();
                if (i < J.rows()) 
                {
                    j_row = J.row(i).transpose();
                }

                Types::Vec3 grad_v = Types::Vec3::Zero();
                Types::Vec3 grad_a = Types::Vec3::Zero();
                Types::Vec3 grad_j = Types::Vec3::Zero();
                    
                cost_feasibility += cost_functional::accumulateFeasibilityPenalty(
                    v_row, a_row, j_row, max_vel, max_acc, max_jer, wei_feas, grad_v, grad_a, grad_j
                );

                gdV.row(i) += grad_v.transpose();
                if (i < A.rows()) 
                {
                    gdA.row(i) += grad_a.transpose();
                }
                if (i < J.rows()) 
                {
                    gdJ.row(i) += grad_j.transpose();
                }
            }

            std::vector<double> time_accum(segment_dt_.size() + 1, 0.0);
            for (size_t k = 0; k < segment_dt_.size(); ++k) 
            {
                time_accum[k + 1] = time_accum[k] + segment_dt_[k];
            }

            //obstacle and swarm
            for(int i = 0; i < C.rows(); i++)
            {
                Types::Vec3 p_row = C.row(i).transpose();
                Types::Vec3 grad_p = Types::Vec3::Zero();
                cost_guide += cost_functional::accumulateObstaclePenalty(i,cps,touch_goal,p_row,obs_clearance,obs_clearance_soft,wei_obs,wei_obs_soft,grad_p);


                if (swarm_traj != nullptr)
                {
                    int seg_idx = i / cps_per_piece;
                    int step_in_seg = i % cps_per_piece;
                    if (seg_idx >= segment_dt_.size()) 
                    {
                        seg_idx = segment_dt_.size() - 1;
                        step_in_seg = cps_per_piece;
                    }
                    double dt_seg = segment_dt_[seg_idx];
                    

                    double t_global = time_accum[seg_idx] + (static_cast<double>(step_in_seg) / cps_per_piece) * dt_seg;
                    
                    double dummy_gt = 0.0; 

                    cost_swarm += cost_functional::accumulateSwarmPenalty(
                        i, cps, touch_goal, swarm_traj, drone_id, 
                        t_now, t_global, swarm_clearance, wei_swarm, 
                        p_row, grad_p, dummy_gt, min_ellip_dist2_ptr
                    );
                }

                gdC.row(i) += grad_p.transpose();
            }

            //sample
            cost_smooth += cost_functional::accumulateVarianceSampleCost(C,wei_sqrvar,gdC);

            accumulated_costs(0) += cost_feasibility;
            accumulated_costs(1) += cost_guide;
            accumulated_costs(2) += cost_swarm;
            accumulated_costs(3) += cost_smooth;

            total_cost = cost_feasibility + cost_guide + cost_swarm + cost_smooth;
        }

    };

}//namespace cost_functional

#endif