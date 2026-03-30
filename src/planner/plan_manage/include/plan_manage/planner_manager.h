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
#include <SpatialMap/SFCCommonTypes.hpp>
#include <path_searching/dyn_a_star.h>

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
        MINCOBoundaryState3D &headState, MINCOBoundaryState3D &tailState);
        
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal,
        const bool force_plain = false);

    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const std::vector<Eigen::Vector3d> &guide_path,
        const spatial_map::PolyhedraH &corridor_hpolys, const bool touch_goal);

    bool prepareLocalGuideAndCorridor(const Eigen::Vector3d &start_pt,
                                      const Eigen::Vector3d &start_vel,
                                      const Eigen::Vector3d &goal_pt,
                                      std::vector<Eigen::Vector3d> &guide_path,
                                      spatial_map::PolyhedraH &corridor_hpolys,
                                      bool force_refresh = false);

    bool pointInsidePolytope(const Eigen::Vector3d& pt,
                         const spatial_map::PolyhedronH& hpoly,
                         double margin = 0.0) const;

    bool pointInsideCorridor(const Eigen::Vector3d& pt,
                            const spatial_map::PolyhedraH& corridor,
                            double margin = 0.0) const;

    Eigen::Vector3d computeLaunchPoint(const Eigen::Vector3d& start_pt,
                                      const Eigen::Vector3d& start_vel) const;

    bool buildWarmStartFromCurrentTraj(const Eigen::Vector3d& start_pt,
                                      const Eigen::Vector3d& goal_pt,
                                      Eigen::MatrixXd& inner_pts,
                                      Eigen::VectorXd& durations) const;
        
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
    bool searchLocalGuidePath(const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &goal_pt,
                              std::vector<Eigen::Vector3d> &guide_path);
    bool generateSafeFlightCorridor(const std::vector<Eigen::Vector3d> &guide_path,
                                    spatial_map::PolyhedraH &corridor_hpolys) const;
    bool buildGuideInitialGuess(const std::vector<Eigen::Vector3d> &guide_path,
                                Eigen::MatrixXd &inner_pts,
                                Eigen::VectorXd &durations) const;

    PlanningVisualization::Ptr visualization_;
    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    AStar::Ptr local_astar_;
    Eigen::Vector3i local_astar_pool_size_{Eigen::Vector3i::Zero()};
    Eigen::Vector3d corridor_seed_goal_;
    Eigen::Vector3d corridor_seed_start_;
    bool use_sfc_corridor_{false};
    double sfc_path_timeout_{0.2};
    double sfc_progress_{0.75};
    double sfc_range_{0.8};
    double sfc_launch_dist_{0.8};
    double sfc_reuse_goal_tol_{0.6};
    double sfc_corridor_margin_{0.05};
    double sfc_near_goal_radius_{0.8};
    int replan_seq_{0};

    int continous_failures_count_{0};

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
