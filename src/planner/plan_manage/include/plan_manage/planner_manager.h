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
#include <SpatialMap/CorridorInit.hpp>
#include <path_searching/simple_a_star.hpp>

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

    bool corridorModeEnabled();

    enum CorridorFailureType
    {
      FAIL_NONE = 0,
      FAIL_LOCAL_TARGET_INVALID,
      FAIL_GUIDE_PATH_TOO_CLOSE,
      FAIL_CORRIDOR_GENERATION,
      FAIL_CORRIDOR_INIT,
      FAIL_CORRIDOR_OPT
    };
    
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
    inline CorridorFailureType getLastCorridorFailureType(void) const { return last_corridor_failure_type_; }
    inline const std::string &getLastCorridorFailureTag(void) const { return last_corridor_failure_tag_; }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    bool sanitizeLocalTarget(const Eigen::Vector3d &raw_target,
                             Eigen::Vector3d &safe_target) const;
    bool sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
                           std::vector<Eigen::Vector3d> &sparse_path) const;
    bool buildInitStateFromGuidePath(const Eigen::Vector3d &start_pt,
                                     const Eigen::Vector3d &start_vel,
                                     const Eigen::Vector3d &start_acc,
                                     const Eigen::Vector3d &target_pt,
                                     const Eigen::Vector3d &target_vel,
                                     const std::vector<Eigen::Vector3d> &guide_path,
                                     MINCOTraj3D &init_traj,
                                     Eigen::MatrixXd &inner_pts,
                                     Eigen::VectorXd &durations,
                                     MINCOBoundaryState3D &head_state,
                                     MINCOBoundaryState3D &tail_state) const;
    bool generateSafeFlightCorridor(const std::vector<Eigen::Vector3d> &guide_path,
                                    spatial_map::PolyhedraH &corridor_hpolys) const;
    bool buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                        const Eigen::Vector3d &goal_pt,
                                        const spatial_map::PolyhedraH &corridor_hpolys,
                                        Eigen::MatrixXd &inner_pts,
                                        Eigen::VectorXd &durations,
                                        std::vector<Eigen::Vector3d> &transition_points,
                                        std::vector<double> &inner_clearances) const;
    bool assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                         Eigen::MatrixXd &inner_pts,
                                         Eigen::VectorXd &durations,
                                         std::vector<double> *inner_clearances = nullptr) const;

    int findFirstCorridorPolyContainingPoint(const Eigen::Vector3d &pt,
                                             const spatial_map::PolyhedraH &corridor_hpolys,
                                             double margin = 0.0) const;

    bool sampleWarmStartPrefixFromCurrentTraj(
        const Eigen::Vector3d &start_pt,
        const spatial_map::PolyhedraH &corridor_hpolys,
        std::vector<Eigen::Vector3d> &prefix_nodes,
        std::vector<double> &prefix_durations) const;

    bool mergePrefixAndTailInitialGuess(
        const std::vector<Eigen::Vector3d> &prefix_nodes,
        const std::vector<double> &prefix_durations,
        const Eigen::MatrixXd &tail_inner_pts,
        const Eigen::VectorXd &tail_durations,
        Eigen::MatrixXd &inner_pts,
        Eigen::VectorXd &durations,
        std::vector<double> *inner_clearances = nullptr) const;

    double estimateObstacleClearance(const Eigen::Vector3d &pt,
                                     double search_radius,
                                     Eigen::Vector3d *push_dir = nullptr) const;
    double computeTrajectoryMinSdf(const MINCOTraj3D &traj) const;
    void reportCorridorFailure(CorridorFailureType type,
                               const std::string &detail);

    bool prepareLocalAStarPath(const Eigen::Vector3d &start_pt,
                             const Eigen::Vector3d &goal_pt,
                             std::vector<Eigen::Vector3d> &dense_path,
                             Eigen::Vector3d &safe_goal) const;

    PlanningVisualization::Ptr visualization_;
    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    SimpleAStar::Ptr simple_astar_;
    bool use_sfc_corridor_{false};
    bool use_esdf_{false};
    double sfc_path_timeout_{0.2};
    double sfc_progress_{0.75};
    double sfc_range_{0.8};
    double sfc_corridor_margin_{0.05};
    double guide_min_clearance_{0.35};
    int guide_sparse_min_inner_{2};
    int guide_sparse_max_inner_{5};
    double guide_turn_angle_deg_{25.0};
    double warm_start_prefix_time_{0.45};
    int warm_start_prefix_max_points_{4};

    int replan_seq_{0};
    CorridorFailureType last_corridor_failure_type_{FAIL_NONE};
    std::string last_corridor_failure_tag_{"NONE"};

    int continous_failures_count_{0};

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
