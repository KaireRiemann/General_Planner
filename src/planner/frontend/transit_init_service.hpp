#ifndef PLANNER_FRONTEND_TRANSIT_INIT_SERVICE_HPP_
#define PLANNER_FRONTEND_TRANSIT_INIT_SERVICE_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

#include <frontend/init_artifact.hpp>
#include <core/planning_problem.hpp>
#include <optimizer/poly_traj_optimizer.h>
#include <path_searching/jps_a_star.hpp>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>

namespace ego_planner::frontend
{

struct TransitInitRuntimeConfig
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

enum class TransitInitFailureType
{
  NONE = 0,
  LOCAL_TARGET_INVALID,
  GUIDE_PATH_TOO_CLOSE,
  CORRIDOR_GENERATION,
  CORRIDOR_INIT
};

struct TransitInitResult
{
  bool success{false};
  core::ActiveSpaceModel selected_mode{core::ActiveSpaceModel::PLAIN};
  std::string init_source{"stable_helper"};
  std::string message;
  std::string failure_reason;
  TransitInitFailureType failure_type{TransitInitFailureType::NONE};

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

// TransitInitService is the future owner for plain/ESDF/corridor seed
// construction. It is introduced without changing the active planning path.
class TransitInitService
{
public:
  TransitInitService() = default;

  static MINCOBoundaryState3D makeBoundaryState(const Eigen::Vector3d &pos,
                                                const Eigen::Vector3d &vel,
                                                const Eigen::Vector3d &acc);

  bool initializePlain(const TransitInitRuntimeConfig &config,
                       const core::PlanningProblem &problem,
                       TransitInitResult &result) const;

  bool initializeEsdf(const TransitInitRuntimeConfig &config,
                      const core::PlanningProblem &problem,
                      TransitInitResult &result) const;

  bool initializeCorridor(const TransitInitRuntimeConfig &config,
                          const core::PlanningProblem &problem,
                          TransitInitResult &result) const;

  bool buildFromAnchors(const Eigen::Vector3d &start_pt,
                        const Eigen::Vector3d &start_vel,
                        const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &target_pt,
                        const Eigen::Vector3d &target_vel,
                        const std::vector<Eigen::Vector3d> &anchors,
                        double piece_length,
                        double max_vel,
                        InitArtifact &artifact,
                        const std::string &source = "frontend_anchor_init") const;

  bool assembleInitialGuessFromAnchors(const TransitInitRuntimeConfig &config,
                                       const std::vector<Eigen::Vector3d> &anchors,
                                       Eigen::MatrixXd &inner_pts,
                                       Eigen::VectorXd &durations,
                                       std::vector<double> *inner_clearances = nullptr) const;

  bool applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                   Eigen::VectorXd &durations) const;

  bool computeInitialState(const TransitInitRuntimeConfig &config,
                           const Eigen::Vector3d &start_pt,
                           const Eigen::Vector3d &start_vel,
                           const Eigen::Vector3d &start_acc,
                           const Eigen::Vector3d &local_target_pt,
                           const Eigen::Vector3d &local_target_vel,
                           bool flag_poly_init,
                           bool flag_random_poly_traj,
                           const double &ts,
                           MINCOTraj3D &init_traj,
                           Eigen::MatrixXd &out_inner_pts,
                           Eigen::VectorXd &out_durations,
                           MINCOBoundaryState3D &head_state,
                           MINCOBoundaryState3D &tail_state,
                           const core::PlanningContext *planning_context = nullptr) const;

  bool prepareLocalGuideAndCorridor(const TransitInitRuntimeConfig &config,
                                    const Eigen::Vector3d &start_pt,
                                    const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &goal_pt,
                                    std::vector<Eigen::Vector3d> &guide_path,
                                    spatial_map::PolyhedraH &corridor_hpolys,
                                    std::vector<Eigen::Vector3d> *dense_path = nullptr) const;

  bool buildCorridorAwareInitialGuess(const TransitInitRuntimeConfig &config,
                                      const Eigen::Vector3d &start_pt,
                                      const Eigen::Vector3d &start_vel,
                                      const Eigen::Vector3d &goal_pt,
                                      const spatial_map::PolyhedraH &corridor_hpolys,
                                      Eigen::MatrixXd &inner_pts,
                                      Eigen::VectorXd &durations,
                                      Eigen::VectorXi &corridor_piece_idx,
                                      std::vector<Eigen::Vector3d> &transition_points,
                                      std::vector<double> &inner_clearances) const;

  bool improveCorridorSeedByTimeScaling(const TransitInitRuntimeConfig &config,
                                        const MINCOBoundaryState3D &head_state,
                                        const MINCOBoundaryState3D &tail_state,
                                        const Eigen::MatrixXd &inner_pts,
                                        Eigen::VectorXd &durations,
                                        const spatial_map::PolyhedraH &corridor_hpolys,
                                        MINCOTraj3D &traj) const;

  double computeTrajectoryMinSdf(const TransitInitRuntimeConfig &config,
                                 const MINCOTraj3D &traj) const;

private:
  bool assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                       double piece_length,
                                       double max_vel,
                                       Eigen::MatrixXd &inner_points,
                                       Eigen::VectorXd &durations) const;

  bool buildInitStateFromGuidePath(const TransitInitRuntimeConfig &config,
                                   const Eigen::Vector3d &start_pt,
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
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_TRANSIT_INIT_SERVICE_HPP_
