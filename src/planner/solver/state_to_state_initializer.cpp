#include <solver/state_to_state_initializer.hpp>

#include <frontend/corridor_service.hpp>
#include <frontend/guide_path_service.hpp>
#include <frontend/transit_init_service.hpp>

namespace ego_planner::solver
{

namespace
{

frontend::GuidePathRuntimeConfig makeGuideRuntimeConfig(const StateToStateInitResources &resources)
{
  frontend::GuidePathRuntimeConfig config;
  config.grid_map = resources.grid_map;
  config.jps_astar = resources.jps_astar;
  config.poly_piece_length = resources.plan_params ? resources.plan_params->polyTraj_piece_length : 0.2;
  config.guide_min_clearance = resources.guide_min_clearance;
  config.guide_sparse_min_inner = resources.guide_sparse_min_inner;
  config.guide_sparse_max_inner = resources.guide_sparse_max_inner;
  config.guide_turn_angle_deg = resources.guide_turn_angle_deg;
  config.sfc_range = resources.sfc_range;
  return config;
}

frontend::CorridorRuntimeConfig makeCorridorRuntimeConfig(const StateToStateInitResources &resources)
{
  frontend::CorridorRuntimeConfig config;
  config.grid_map = resources.grid_map;
  config.sfc_progress = resources.sfc_progress;
  config.sfc_range = resources.sfc_range;
  return config;
}

frontend::TransitInitRuntimeConfig makeTransitInitRuntimeConfig(const StateToStateInitResources &resources)
{
  frontend::TransitInitRuntimeConfig config;
  config.plan_params = resources.plan_params;
  config.traj_container = resources.traj_container;
  config.continuous_failures_count = resources.continuous_failures_count;
  config.grid_map = resources.grid_map;
  config.jps_astar = resources.jps_astar;
  config.optimizer = resources.optimizer;
  config.guide_min_clearance = resources.guide_min_clearance;
  config.guide_sparse_min_inner = resources.guide_sparse_min_inner;
  config.guide_sparse_max_inner = resources.guide_sparse_max_inner;
  config.guide_turn_angle_deg = resources.guide_turn_angle_deg;
  config.sfc_progress = resources.sfc_progress;
  config.sfc_range = resources.sfc_range;
  return config;
}

StateToStateInitFailureType mapFailureType(const frontend::TransitInitFailureType failure_type)
{
  switch (failure_type)
  {
  case frontend::TransitInitFailureType::LOCAL_TARGET_INVALID:
    return StateToStateInitFailureType::LOCAL_TARGET_INVALID;
  case frontend::TransitInitFailureType::GUIDE_PATH_TOO_CLOSE:
    return StateToStateInitFailureType::GUIDE_PATH_TOO_CLOSE;
  case frontend::TransitInitFailureType::CORRIDOR_GENERATION:
    return StateToStateInitFailureType::CORRIDOR_GENERATION;
  case frontend::TransitInitFailureType::CORRIDOR_INIT:
    return StateToStateInitFailureType::CORRIDOR_INIT;
  case frontend::TransitInitFailureType::NONE:
  default:
    return StateToStateInitFailureType::NONE;
  }
}

void copyTransitResult(const frontend::TransitInitResult &frontend_result,
                       StateToStateInitializationResult &result)
{
  result.success = frontend_result.success;
  result.selected_mode = frontend_result.selected_mode;
  result.message = frontend_result.message;
  result.failure_reason = frontend_result.failure_reason;
  result.failure_type = mapFailureType(frontend_result.failure_type);

  result.compiler_hint_attempted = frontend_result.compiler_hint_attempted;
  result.compiler_hint_succeeded = frontend_result.compiler_hint_succeeded;
  result.stable_helper_attempted = frontend_result.stable_helper_attempted;
  result.stable_helper_succeeded = frontend_result.stable_helper_succeeded;
  result.corridor_warm_timing_used = frontend_result.corridor_warm_timing_used;
  result.corridor_time_scaling_feasible = frontend_result.corridor_time_scaling_feasible;
  result.init_artifact = frontend_result.init_artifact;
}

} // namespace

bool StateToStateInitializer::sanitizeLocalTarget(const Eigen::Vector3d &raw_target,
                                                  Eigen::Vector3d &safe_target) const
{
  return frontend::GuidePathService{}.sanitizeLocalTarget(makeGuideRuntimeConfig(resources_),
                                                          raw_target,
                                                          safe_target);
}

bool StateToStateInitializer::sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
                                                std::vector<Eigen::Vector3d> &sparse_path) const
{
  return frontend::GuidePathService{}.sparsifyGuidePath(makeGuideRuntimeConfig(resources_),
                                                        dense_path,
                                                        sparse_path);
}

double StateToStateInitializer::computeTrajectoryMinSdf(const MINCOTraj3D &traj) const
{
  return frontend::TransitInitService{}.computeTrajectoryMinSdf(makeTransitInitRuntimeConfig(resources_),
                                                                traj);
}

bool StateToStateInitializer::assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                                              Eigen::MatrixXd &inner_pts,
                                                              Eigen::VectorXd &durations,
                                                              std::vector<double> *inner_clearances) const
{
  return frontend::TransitInitService{}.assembleInitialGuessFromAnchors(makeTransitInitRuntimeConfig(resources_),
                                                                        anchors,
                                                                        inner_pts,
                                                                        durations,
                                                                        inner_clearances);
}

bool StateToStateInitializer::buildInitStateFromGuidePath(const Eigen::Vector3d &start_pt,
                                                          const Eigen::Vector3d &start_vel,
                                                          const Eigen::Vector3d &start_acc,
                                                          const Eigen::Vector3d &target_pt,
                                                          const Eigen::Vector3d &target_vel,
                                                          const std::vector<Eigen::Vector3d> &guide_path,
                                                          MINCOTraj3D &init_traj,
                                                          Eigen::MatrixXd &inner_pts,
                                                          Eigen::VectorXd &durations,
                                                          MINCOBoundaryState3D &head_state,
                                                          MINCOBoundaryState3D &tail_state) const
{
  if (resources_.plan_params == nullptr)
  {
    return false;
  }
  frontend::InitArtifact artifact;
  if (!frontend::TransitInitService{}.buildFromAnchors(start_pt,
                                                       start_vel,
                                                       start_acc,
                                                       target_pt,
                                                       target_vel,
                                                       guide_path,
                                                       resources_.plan_params->polyTraj_piece_length,
                                                       resources_.plan_params->max_vel_,
                                                       artifact,
                                                       "guide_path_init"))
  {
    return false;
  }

  init_traj = artifact.init_traj;
  inner_pts = artifact.inner_points;
  durations = artifact.durations;
  head_state = artifact.head_state;
  tail_state = artifact.tail_state;
  return artifact.valid;
}

bool StateToStateInitializer::applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                                          Eigen::VectorXd &durations) const
{
  return frontend::TransitInitService{}.applyWarmStartTimingProfile(warm_durations, durations);
}

bool StateToStateInitializer::buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                                             const Eigen::Vector3d &start_vel,
                                                             const Eigen::Vector3d &goal_pt,
                                                             const spatial_map::PolyhedraH &corridor_hpolys,
                                                             Eigen::MatrixXd &inner_pts,
                                                             Eigen::VectorXd &durations,
                                                             Eigen::VectorXi &corridor_piece_idx,
                                                             std::vector<Eigen::Vector3d> &transition_points,
                                                             std::vector<double> &inner_clearances) const
{
  return frontend::TransitInitService{}.buildCorridorAwareInitialGuess(makeTransitInitRuntimeConfig(resources_),
                                                                       start_pt,
                                                                       start_vel,
                                                                       goal_pt,
                                                                       corridor_hpolys,
                                                                       inner_pts,
                                                                       durations,
                                                                       corridor_piece_idx,
                                                                       transition_points,
                                                                       inner_clearances);
}

bool StateToStateInitializer::generateSafeFlightCorridor(const std::vector<Eigen::Vector3d> &guide_path,
                                                         spatial_map::PolyhedraH &corridor_hpolys) const
{
  return frontend::CorridorService{}.generateSafeFlightCorridor(makeCorridorRuntimeConfig(resources_),
                                                                guide_path,
                                                                corridor_hpolys);
}

bool StateToStateInitializer::prepareLocalAStarPath(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Vector3d &goal_pt,
                                                    std::vector<Eigen::Vector3d> &dense_path,
                                                    Eigen::Vector3d &safe_goal) const
{
  return frontend::GuidePathService{}.prepareLocalAStarPath(makeGuideRuntimeConfig(resources_),
                                                            start_pt,
                                                            goal_pt,
                                                            dense_path,
                                                            safe_goal);
}

bool StateToStateInitializer::prepareLocalGuideAndCorridor(const Eigen::Vector3d &start_pt,
                                                           const Eigen::Vector3d &start_vel,
                                                           const Eigen::Vector3d &goal_pt,
                                                           std::vector<Eigen::Vector3d> &guide_path,
                                                           spatial_map::PolyhedraH &corridor_hpolys,
                                                           std::vector<Eigen::Vector3d> *dense_path) const
{
  return frontend::TransitInitService{}.prepareLocalGuideAndCorridor(makeTransitInitRuntimeConfig(resources_),
                                                                     start_pt,
                                                                     start_vel,
                                                                     goal_pt,
                                                                     guide_path,
                                                                     corridor_hpolys,
                                                                     dense_path);
}

bool StateToStateInitializer::computeInitState(const Eigen::Vector3d &start_pt,
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
                                               const core::PlanningContext *planning_context) const
{
  return frontend::TransitInitService{}.computeInitialState(makeTransitInitRuntimeConfig(resources_),
                                                            start_pt,
                                                            start_vel,
                                                            start_acc,
                                                            local_target_pt,
                                                            local_target_vel,
                                                            flag_polyInit,
                                                            flag_randomPolyTraj,
                                                            ts,
                                                            initTraj,
                                                            outInnerPts,
                                                            outDurations,
                                                            headState,
                                                            tailState,
                                                            planning_context);
}

bool StateToStateInitializer::initializePlain(const core::PlanningProblem &problem,
                                              StateToStateInitializationResult &result) const
{
  frontend::TransitInitResult frontend_result;
  frontend_result.selected_mode = result.selected_mode;
  const bool ok = frontend::TransitInitService{}.initializePlain(makeTransitInitRuntimeConfig(resources_),
                                                                 problem,
                                                                 frontend_result);
  copyTransitResult(frontend_result, result);
  return ok;
}

bool StateToStateInitializer::initializeEsdf(const core::PlanningProblem &problem,
                                             StateToStateInitializationResult &result) const
{
  frontend::TransitInitResult frontend_result;
  frontend_result.selected_mode = result.selected_mode;
  const bool ok = frontend::TransitInitService{}.initializeEsdf(makeTransitInitRuntimeConfig(resources_),
                                                                problem,
                                                                frontend_result);
  copyTransitResult(frontend_result, result);
  return ok;
}

bool StateToStateInitializer::initializeCorridor(const core::PlanningProblem &problem,
                                                 StateToStateInitializationResult &result) const
{
  frontend::TransitInitResult frontend_result;
  frontend_result.selected_mode = result.selected_mode;
  const bool ok = frontend::TransitInitService{}.initializeCorridor(makeTransitInitRuntimeConfig(resources_),
                                                                    problem,
                                                                    frontend_result);
  copyTransitResult(frontend_result, result);
  return ok;
}

bool StateToStateInitializer::initialize(const core::PlanningProblem &problem,
                                         StateToStateInitializationResult &result) const
{
  result = StateToStateInitializationResult{};
  result.selected_mode = problem.active_space_model;

  bool ok = false;

  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    ok = initializeCorridor(problem, result);
    break;
  case core::ActiveSpaceModel::ESDF:
    ok = initializeEsdf(problem, result);
    break;
  case core::ActiveSpaceModel::PLAIN:
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  default:
    ok = initializePlain(problem, result);
    break;
  }

  if (ok)
  {
    if (result.message.empty())
    {
      result.message = "initialized via " + result.init_artifact.source;
    }
  }
  else if (result.message.empty())
  {
    result.message = result.failure_reason;
  }

  return ok;
}

} // namespace ego_planner::solver
