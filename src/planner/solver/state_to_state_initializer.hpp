#ifndef PLANNER_SOLVER_STATE_TO_STATE_INITIALIZER_HPP_
#define PLANNER_SOLVER_STATE_TO_STATE_INITIALIZER_HPP_

#include <core/planning_problem.hpp>
#include <path_searching/jps_a_star.hpp>
#include <plan_env/grid_map.h>
#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/minco_types.hpp>
#include <traj_utils/plan_container.hpp>

#include <SpatialMap/SFCCommonTypes.hpp>

#include <string>
#include <vector>

namespace ego_planner::solver
{

enum class StateToStateInitFailureType
{
  NONE = 0,
  LOCAL_TARGET_INVALID,
  GUIDE_PATH_TOO_CLOSE,
  CORRIDOR_GENERATION,
  CORRIDOR_INIT
};

struct StateToStateInitResources
{
  const PlanParameters *plan_params{nullptr};
  const TrajContainer *traj_container{nullptr};
  const int *continuous_failures_count{nullptr};
  GridMap::Ptr grid_map;
  JPSAStar *jps_astar{nullptr};
  PolyTrajOptimizer *optimizer{nullptr};

  double guide_min_clearance{0.35};
  int guide_sparse_min_inner{2};
  int guide_sparse_max_inner{5};
  double guide_turn_angle_deg{25.0};
  double sfc_progress{0.75};
  double sfc_range{0.8};
};

struct StateToStateInitializationResult
{
  bool success{false};
  core::ActiveSpaceModel active_mode{core::ActiveSpaceModel::PLAIN};
  std::string init_source{"stable_helper"};
  std::string failure_reason;
  StateToStateInitFailureType failure_type{StateToStateInitFailureType::NONE};

  bool compiler_hint_attempted{false};
  bool compiler_hint_succeeded{false};
  bool stable_helper_attempted{false};
  bool stable_helper_succeeded{false};
  bool corridor_warm_timing_used{false};
  bool corridor_time_scaling_feasible{false};
  bool init_collision_free{true};
  bool init_inside_corridor{true};
  double init_min_sdf{0.0};

  MINCOBoundaryState3D head_state;
  MINCOBoundaryState3D tail_state;
  Eigen::MatrixXd inner_points;
  Eigen::VectorXd durations;
  MINCOTraj3D init_traj;

  std::vector<Eigen::Vector3d> guide_path;
  std::vector<Eigen::Vector3d> dense_path;
  spatial_map::PolyhedraH corridor_hpolys;
  Eigen::VectorXi corridor_piece_idx;
};

// State-to-state initializer is the authoritative solver-side owner of
// guide/corridor/init construction. Compiler outputs are optional hints only.
class StateToStateInitializer
{
public:
  StateToStateInitializer() = default;
  explicit StateToStateInitializer(const StateToStateInitResources &resources)
      : resources_(resources)
  {
  }

  void setResources(const StateToStateInitResources &resources)
  {
    resources_ = resources;
  }

  bool initialize(const core::PlanningProblem &problem,
                  StateToStateInitializationResult &result) const;

  bool sanitizeLocalTarget(const Eigen::Vector3d &raw_target,
                           Eigen::Vector3d &safe_target) const;
  bool prepareLocalAStarPath(const Eigen::Vector3d &start_pt,
                             const Eigen::Vector3d &goal_pt,
                             std::vector<Eigen::Vector3d> &dense_path,
                             Eigen::Vector3d &safe_goal) const;
  bool prepareLocalGuideAndCorridor(const Eigen::Vector3d &start_pt,
                                    const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &goal_pt,
                                    std::vector<Eigen::Vector3d> &guide_path,
                                    spatial_map::PolyhedraH &corridor_hpolys,
                                    std::vector<Eigen::Vector3d> *dense_path = nullptr) const;
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
  bool buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                      const Eigen::Vector3d &start_vel,
                                      const Eigen::Vector3d &goal_pt,
                                      const spatial_map::PolyhedraH &corridor_hpolys,
                                      Eigen::MatrixXd &inner_pts,
                                      Eigen::VectorXd &durations,
                                      Eigen::VectorXi &corridor_piece_idx,
                                      std::vector<Eigen::Vector3d> &transition_points,
                                      std::vector<double> &inner_clearances) const;
  bool applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                   Eigen::VectorXd &durations) const;
  double computeTrajectoryMinSdf(const MINCOTraj3D &traj) const;

private:
  bool mapWindowReady() const;
  bool sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
                         std::vector<Eigen::Vector3d> &sparse_path) const;
  double estimateObstacleClearance(const Eigen::Vector3d &pt,
                                   double search_radius,
                                   Eigen::Vector3d *push_dir = nullptr) const;
  bool lineOfSightFree(const Eigen::Vector3d &from,
                       const Eigen::Vector3d &to,
                       double max_dist = -1.0) const;
  bool assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                       Eigen::MatrixXd &inner_pts,
                                       Eigen::VectorXd &durations,
                                       std::vector<double> *inner_clearances = nullptr) const;
  bool generateSafeFlightCorridor(const std::vector<Eigen::Vector3d> &guide_path,
                                  spatial_map::PolyhedraH &corridor_hpolys) const;
  bool computeInitState(const Eigen::Vector3d &start_pt,
                        const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &local_target_pt,
                        const Eigen::Vector3d &local_target_vel,
                        const bool flag_polyInit,
                        const bool flag_randomPolyTraj,
                        const double &ts,
                        MINCOTraj3D &initTraj,
                        Eigen::MatrixXd &outInnerPts,
                        Eigen::VectorXd &outDurations,
                        MINCOBoundaryState3D &headState,
                        MINCOBoundaryState3D &tailState,
                        const core::PlanningContext *planning_context = nullptr) const;
  bool improveCorridorSeedByTimeScaling(const MINCOBoundaryState3D &head_state,
                                        const MINCOBoundaryState3D &tail_state,
                                        const Eigen::MatrixXd &inner_pts,
                                        Eigen::VectorXd &durations,
                                        const spatial_map::PolyhedraH &corridor_hpolys,
                                        MINCOTraj3D &traj) const;
  bool initializePlain(const core::PlanningProblem &problem,
                       StateToStateInitializationResult &result) const;
  bool initializeEsdf(const core::PlanningProblem &problem,
                      StateToStateInitializationResult &result) const;
  bool initializeCorridor(const core::PlanningProblem &problem,
                          StateToStateInitializationResult &result) const;

private:
  StateToStateInitResources resources_;
};

} // namespace ego_planner::solver

#endif // PLANNER_SOLVER_STATE_TO_STATE_INITIALIZER_HPP_
