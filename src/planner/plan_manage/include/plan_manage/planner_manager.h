#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <ros/ros.h>
#include <thread>
#include <memory>
#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <SpatialMap/SFCCommonTypes.hpp>
#include <SpatialMap/CorridorInit.hpp>
#include <CostFunctionalManager/TrackingTypes.hpp>
#include <CostFunctionalManager/TrackingSemanticGuide.hpp>
#include <path_searching/jps_a_star.hpp>
#include <path_searching/visible_region_graph.hpp>
#include <core/planning_context.hpp>
#include <core/task_spec.hpp>
#include <core/planning_solution.hpp>
#include <compiler/problem_compiler.hpp>
#include <optimization/backend_solver.hpp>
#include <optimization/problem_adapter.hpp>
#include "plan_manage/tracking_yaw_planner.hpp"

namespace ego_planner
{
  class EGOPlannerManager : public optimization::ProblemAdapter
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    bool corridorModeEnabled();

    // This system is organized as:
    // TaskSpec + PlanningContext -> ProblemCompiler -> PlanningProblem -> BackendSolver -> PlanningSolution.
    // Task-specific semantics belong in TaskSpec and ProblemCompiler, not in FSM or solver.
    bool solveTask(const core::PlanningContext &context,
                   const core::TaskSpec &task,
                   core::PlanningSolution &solution);

    bool solveCompatibility(const core::PlanningProblem &problem,
                            core::PlanningSolution &solution) override;

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

    // Legacy compatibility path. New callers should prefer solveTask(...).
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal,
        const bool force_plain = false,
        const cost_functional::TrackingReference *tracking_ref = nullptr,
        const std::vector<Eigen::Vector3d> *preferred_guide_path = nullptr,
        const cost_functional::TrackingSemanticGuide *tracking_semantic_guide = nullptr);

    // Legacy compatibility path. New callers should prefer solveTask(...).
    bool planTrackingTask(
        const cost_functional::TrackingReference &reference,
        const Eigen::Vector3d &start_pt,
        const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc,
        const bool flag_polyInit,
        const bool flag_randomPolyTraj,
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
    bool trackingSemanticHorizonValid(double t_cur, double horizon) const;
    
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
    bool mapWindowReady() const;
    bool sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
                           std::vector<Eigen::Vector3d> &sparse_path) const;
    bool buildTrackingAnchorCandidates(const cost_functional::TrackingReference &reference,
                                       const Eigen::Vector3d &start_pt,
                                       const Eigen::Vector3d &start_vel,
                                       std::vector<Eigen::Vector3d> &anchor_candidates,
                                       std::vector<Eigen::Vector3d> *anchor_target_vels = nullptr,
                                       std::vector<double> *anchor_times = nullptr) const;
    bool buildTrackingViewpointSeries(const cost_functional::TrackingReference &reference,
                                      const Eigen::Vector3d &start_pt,
                                      const Eigen::Vector3d &start_vel,
                                      std::vector<Eigen::Vector3d> &target_samples,
                                      std::vector<Eigen::Vector3d> &viewpoint_series,
                                      std::vector<Eigen::Vector3d> *viewpoint_target_vels = nullptr,
                                      std::vector<double> *viewpoint_times = nullptr) const;
    bool buildTrackingVisibleRegionGuide(const cost_functional::TrackingReference &reference,
                                         const Eigen::Vector3d &start_pt,
                                         const Eigen::Vector3d &start_vel,
                                         std::vector<Eigen::Vector3d> &target_samples,
                                         std::vector<Eigen::Vector3d> &viewpoint_series,
                                         std::vector<Eigen::Vector3d> &guide_path,
                                         std::vector<Eigen::Vector3d> *viewpoint_target_vels = nullptr,
                                         std::vector<double> *viewpoint_times = nullptr,
                                         std::vector<Eigen::Vector3d> *candidate_points = nullptr) const;
    bool buildTrackingSemanticGuideHypotheses(const cost_functional::TrackingReference &reference,
                                              const Eigen::Vector3d &start_pt,
                                              const Eigen::Vector3d &start_vel,
                                              std::vector<cost_functional::TrackingSemanticGuide> &hypotheses) const;
    bool buildTrackingVisibleFanRegions(cost_functional::TrackingSemanticGuide &semantic_guide) const;
    bool buildGuidePathFromWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                                     std::vector<Eigen::Vector3d> &guide_path) const;
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
    bool generateTrackingSafeFlightCorridor(const cost_functional::TrackingSemanticGuide &semantic_guide,
                                            spatial_map::PolyhedraH &corridor_hpolys,
                                            Eigen::VectorXi &corridor_piece_idx) const;
    bool buildTimeAlignedTrackingInitialGuess(const Eigen::Vector3d &start_pt,
                                              const Eigen::Vector3d &start_vel,
                                              const Eigen::Vector3d &goal_pt,
                                              const cost_functional::TrackingSemanticGuide &semantic_guide,
                                              const spatial_map::PolyhedraH &corridor_hpolys,
                                              Eigen::MatrixXd &inner_pts,
                                              Eigen::VectorXd &durations,
                                              Eigen::VectorXi &corridor_piece_idx,
                                              std::vector<double> *inner_clearances = nullptr) const;
    bool buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                        const Eigen::Vector3d &start_vel,
                                        const Eigen::Vector3d &goal_pt,
                                        const spatial_map::PolyhedraH &corridor_hpolys,
                                        Eigen::MatrixXd &inner_pts,
                                        Eigen::VectorXd &durations,
                                        Eigen::VectorXi &corridor_piece_idx,
                                        std::vector<Eigen::Vector3d> &transition_points,
                                        std::vector<double> &inner_clearances) const;
    bool assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                         Eigen::MatrixXd &inner_pts,
                                         Eigen::VectorXd &durations,
                                         std::vector<double> *inner_clearances = nullptr) const;

    bool applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                     Eigen::VectorXd &durations) const;

    double estimateObstacleClearance(const Eigen::Vector3d &pt,
                                     double search_radius,
                                     Eigen::Vector3d *push_dir = nullptr) const;
    bool lineOfSightFree(const Eigen::Vector3d &from,
                         const Eigen::Vector3d &to,
                         double max_dist = -1.0) const;
    double computeTrajectoryMinSdf(const MINCOTraj3D &traj) const;
    void reportCorridorFailure(CorridorFailureType type,
                               const std::string &detail);

    bool prepareLocalAStarPath(const Eigen::Vector3d &start_pt,
                             const Eigen::Vector3d &goal_pt,
                             std::vector<Eigen::Vector3d> &dense_path,
                             Eigen::Vector3d &safe_goal) const;

    PlanningVisualization::Ptr visualization_;
    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    JPSAStar::Ptr jps_astar_;
    mutable VisibleRegionGraph::Ptr tracking_vrg_;
    bool use_sfc_corridor_{false};
    bool use_esdf_{false};
    double sfc_path_timeout_{0.2};
    double sfc_progress_{0.75};
    double sfc_range_{0.8};
    double sfc_corridor_margin_{0.05};
    int jps_jump_max_cells_{6};
    int jps_near_obs_radius_{1};
    double guide_min_clearance_{0.35};
    int guide_sparse_min_inner_{2};
    int guide_sparse_max_inner_{5};
    double guide_turn_angle_deg_{25.0};
    double tracking_distance_min_{1.5};
    double tracking_distance_max_{4.0};
    double tracking_anchor_future_time_{1.0};
    double tracking_anchor_max_future_time_{2.0};
    double tracking_anchor_dir_hysteresis_{0.35};
    double tracking_anchor_side_angle_deg_{35.0};
    double tracking_viewpoint_dt_{0.6};
    int tracking_viewpoint_max_num_{5};
    double tracking_viewpoint_yaw_step_deg_{20.0};
    double tracking_viewpoint_connect_dist_{1.5};
    double tracking_viewpoint_clearance_{0.15};
    int tracking_hypothesis_topk_{3};
    double tracking_time_align_alpha_{0.55};
    double tracking_visible_yaw_half_span_deg_{35.0};
    double tracking_visible_z_half_span_{0.50};
    mutable bool have_tracking_anchor_dir_{false};
    mutable Eigen::Vector3d last_tracking_anchor_dir_{Eigen::Vector3d::UnitX()};
    mutable cost_functional::TrackingSemanticGuide active_tracking_semantic_guide_;
    mutable spatial_map::PolyhedraH active_tracking_corridor_;
    mutable bool have_active_tracking_semantic_guide_{false};

    std::unique_ptr<compiler::ProblemCompiler> problem_compiler_;
    std::unique_ptr<optimization::BackendSolver> backend_solver_;

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
