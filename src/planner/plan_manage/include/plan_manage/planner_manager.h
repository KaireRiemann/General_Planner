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
#include <CostFunctionalManager/TrackingTypes.hpp>
#include <CostFunctionalManager/TrackingSemanticGuide.hpp>
#include <solver/state_to_state_initializer.hpp>
#include <path_searching/jps_a_star.hpp>
#include <path_searching/visible_region_graph.hpp>

namespace ego_planner
{
  namespace engine
  {
    class PlannerEngine;
  }

  // planner_manager is now a resource/module host.
  // Unified task solving is owned by engine::PlannerEngine:
  // TaskDefinition + PlanningContext -> ProblemCompiler -> PlanningProblem -> solveProblem -> PlanningSolution.
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
    bool esdfModeEnabled();

    enum CorridorFailureType
    {
      FAIL_NONE = 0,
      FAIL_LOCAL_TARGET_INVALID,
      FAIL_GUIDE_PATH_TOO_CLOSE,
      FAIL_CORRIDOR_GENERATION,
      FAIL_CORRIDOR_INIT,
      FAIL_CORRIDOR_OPT
    };
    
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool checkCollision(int drone_id);
    
    bool setLocalTrajFromOpt(const MINCOTraj3D &traj, const bool touch_goal);
    bool setLocalTrajFromOpt(const SnapTraj3D &traj, const bool touch_goal);
    bool trackingSemanticHorizonValid(double t_cur, double horizon) const;
    
    inline double getSwarmClearance(void) { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    inline CorridorFailureType getLastCorridorFailureType(void) const { return last_corridor_failure_type_; }
    inline const std::string &getLastCorridorFailureTag(void) const { return last_corridor_failure_tag_; }
    inline JPSAStar *getJpsAstar(void) const { return jps_astar_.get(); }
    inline PolyTrajOptimizer *getOptimizer(void) const { return ploy_traj_opt_.get(); }
    inline PlanningVisualization::Ptr getVisualization(void) const { return visualization_; }
    inline int *getContinuousFailuresCountPtr(void) { return &continous_failures_count_; }
    inline bool managerPrefersCorridor(void) const { return use_sfc_corridor_; }
    inline bool managerPrefersEsdf(void) const { return use_esdf_; }
    solver::StateToStateInitResources makeStateToStateInitResources() const;
    inline bool hasActiveTrackingSemanticGuide(void) const { return have_active_tracking_semantic_guide_; }
    inline const cost_functional::TrackingSemanticGuide &getActiveTrackingSemanticGuide(void) const { return active_tracking_semantic_guide_; }
    void setActiveTrackingArtifacts(const cost_functional::TrackingSemanticGuide &semantic_guide,
                                   const spatial_map::PolyhedraH &corridor_hpolys);
    void clearActiveTrackingArtifacts();
    inline double getGuideMinClearance(void) const { return guide_min_clearance_; }
    inline int getGuideSparseMinInner(void) const { return guide_sparse_min_inner_; }
    inline int getGuideSparseMaxInner(void) const { return guide_sparse_max_inner_; }
    inline double getGuideTurnAngleDeg(void) const { return guide_turn_angle_deg_; }
    inline double getSfcProgress(void) const { return sfc_progress_; }
    inline double getSfcRange(void) const { return sfc_range_; }
    inline double getSfcCorridorMargin(void) const { return sfc_corridor_margin_; }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    TrajContainer traj_;

  private:
    // Legacy task bridges are retained only for non-migrated compatibility paths.
    // Unified task solving ownership lives in PlannerEngine.
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const bool touch_goal,
        const bool force_plain = false,
        const cost_functional::TrackingReference *tracking_ref = nullptr,
        const std::vector<Eigen::Vector3d> *preferred_guide_path = nullptr,
        const cost_functional::TrackingSemanticGuide *tracking_semantic_guide = nullptr);
    bool planTrackingTask(
        const cost_functional::TrackingReference &reference,
        const Eigen::Vector3d &start_pt,
        const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc,
        const bool flag_polyInit,
        const bool flag_randomPolyTraj,
        const bool force_plain = false);

    // Legacy frontend / seed helpers remain here only for compatibility flows.
    // The authoritative state-to-state initialization path lives in solver/state_to_state_initializer.*.
    bool pointInsidePolytope(const Eigen::Vector3d &pt,
                             const spatial_map::PolyhedronH &hpoly,
                             double margin = 0.0) const;
    bool pointInsideCorridor(const Eigen::Vector3d &pt,
                             const spatial_map::PolyhedraH &corridor,
                             double margin = 0.0) const;

    // Frontend / seed helpers that still back the legacy replan path.
    bool mapWindowReady() const;
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

    double estimateObstacleClearance(const Eigen::Vector3d &pt,
                                     double search_radius,
                                     Eigen::Vector3d *push_dir = nullptr) const;
    bool lineOfSightFree(const Eigen::Vector3d &from,
                         const Eigen::Vector3d &to,
                         double max_dist = -1.0) const;
    void reportCorridorFailure(CorridorFailureType type,
                               const std::string &detail);

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

    int replan_seq_{0};
    CorridorFailureType last_corridor_failure_type_{FAIL_NONE};
    std::string last_corridor_failure_tag_{"NONE"};

    int continous_failures_count_{0};

  public:
    typedef std::unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
    friend class engine::PlannerEngine;
  };
} // namespace ego_planner

#endif
