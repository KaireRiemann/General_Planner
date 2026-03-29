#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>

namespace ego_planner
{
  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    
    // Planner trajectory now uses MINCO
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts,
        MINCOTraj3D &initTraj, Eigen::MatrixXd &innerPts, Eigen::VectorXd &durations,
        Eigen::Matrix<double, 3, 3> &headState, Eigen::Matrix<double, 3, 3> &tailState);
        
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal);
        
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
        
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
        
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool checkCollision(int drone_id);
    
    bool setLocalTrajFromOpt(const MINCOTraj3D &traj, const bool touch_goal);
    
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    PlanningVisualization::Ptr visualization_;
    PolyTrajOptimizer::Ptr ploy_traj_opt_;

    int continous_failures_count_{0};

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
