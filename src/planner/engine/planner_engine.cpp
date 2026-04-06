#include <engine/planner_engine.hpp>

#include <plan_manage/planner_manager.h>
#include <plan_manage/tracking_yaw_planner.hpp>
#include <SFCGenerator/geo_utils.hpp>
#include <SFCGenerator/quickhull.hpp>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <set>

namespace
{

using ego_planner::EGOPlannerManager;
using ego_planner::MINCOTraj3D;

struct EdgeLess
{
  bool operator()(const std::pair<int, int> &lhs, const std::pair<int, int> &rhs) const
  {
    return lhs.first < rhs.first || (lhs.first == rhs.first && lhs.second < rhs.second);
  }
};

void appendCorridorVisualization(const Eigen::MatrixX4d &hpoly,
                                 std::vector<Eigen::Vector3d> &triangle_vertices,
                                 std::vector<Eigen::Vector3d> &edge_vertices)
{
  Eigen::Matrix3Xd vpoly;
  if (!geo_utils::enumerateVs(hpoly, vpoly) || vpoly.cols() < 4)
  {
    return;
  }

  quickhull::QuickHull<double> qh;
  const double qhull_eps = std::min(1.0e-6, quickhull::defaultEps<double>());
  const auto hull = qh.getConvexHull(vpoly.data(),
                                     static_cast<std::size_t>(vpoly.cols()),
                                     true,
                                     true,
                                     qhull_eps);
  const auto &indices = hull.getIndexBuffer();
  if (indices.size() < 3)
  {
    return;
  }

  std::set<std::pair<int, int>, EdgeLess> unique_edges;
  for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
  {
    const int ia = static_cast<int>(indices[i]);
    const int ib = static_cast<int>(indices[i + 1]);
    const int ic = static_cast<int>(indices[i + 2]);

    triangle_vertices.emplace_back(vpoly.col(ia));
    triangle_vertices.emplace_back(vpoly.col(ib));
    triangle_vertices.emplace_back(vpoly.col(ic));

    unique_edges.emplace(std::min(ia, ib), std::max(ia, ib));
    unique_edges.emplace(std::min(ib, ic), std::max(ib, ic));
    unique_edges.emplace(std::min(ic, ia), std::max(ic, ia));
  }

  for (const auto &edge : unique_edges)
  {
    edge_vertices.emplace_back(vpoly.col(edge.first));
    edge_vertices.emplace_back(vpoly.col(edge.second));
  }
}

void buildCorridorVisualization(const spatial_map::PolyhedraH &corridor_hpolys,
                                std::vector<Eigen::Vector3d> &triangle_vertices,
                                std::vector<Eigen::Vector3d> &edge_vertices)
{
  triangle_vertices.clear();
  edge_vertices.clear();
  for (const auto &hpoly : corridor_hpolys)
  {
    appendCorridorVisualization(hpoly, triangle_vertices, edge_vertices);
  }
}

Eigen::MatrixXd sampleTrajectoryForDisplay(const MINCOTraj3D &traj,
                                           const double dt)
{
  const double total_t = traj.getTotalDuration();
  const double clamped_dt = std::max(dt, 1.0e-3);
  const int sample_num =
      std::max(2, static_cast<int>(std::ceil(total_t / clamped_dt)) + 1);

  Eigen::MatrixXd pts(3, sample_num);
  for (int i = 0; i < sample_num; ++i)
  {
    const double ratio = (sample_num <= 1) ? 0.0
                                           : static_cast<double>(i) / static_cast<double>(sample_num - 1);
    pts.col(i) = traj.evaluate(ratio * total_t, 0);
  }
  return pts;
}

const char *managerDefaultModeString(const bool use_corridor, const bool use_esdf)
{
  if (use_corridor)
  {
    return "CORRIDOR";
  }
  if (use_esdf)
  {
    return "ESDF";
  }
  return "PLAIN";
}

const char *activeSpaceModelString(const ego_planner::core::ActiveSpaceModel mode)
{
  switch (mode)
  {
  case ego_planner::core::ActiveSpaceModel::ESDF:
    return "ESDF";
  case ego_planner::core::ActiveSpaceModel::CORRIDOR:
    return "CORRIDOR";
  case ego_planner::core::ActiveSpaceModel::VISIBLE_REGION:
    return "VISIBLE_REGION";
  case ego_planner::core::ActiveSpaceModel::TERMINAL_MANIFOLD:
    return "TERMINAL_MANIFOLD";
  case ego_planner::core::ActiveSpaceModel::PLAIN:
  default:
    return "PLAIN";
  }
}

const char *spaceModelPreferenceString(const ego_planner::core::SpaceModelPreference pref)
{
  switch (pref)
  {
  case ego_planner::core::SpaceModelPreference::PLAIN:
    return "PLAIN";
  case ego_planner::core::SpaceModelPreference::ESDF:
    return "ESDF";
  case ego_planner::core::SpaceModelPreference::CORRIDOR:
    return "CORRIDOR";
  case ego_planner::core::SpaceModelPreference::VISIBLE_REGION:
    return "VISIBLE_REGION";
  case ego_planner::core::SpaceModelPreference::TERMINAL_MANIFOLD:
    return "TERMINAL_MANIFOLD";
  case ego_planner::core::SpaceModelPreference::AUTO:
  default:
    return "AUTO";
  }
}

const char *seedKindString(const ego_planner::core::SeedSpec::Kind kind)
{
  switch (kind)
  {
  case ego_planner::core::SeedSpec::Kind::PLAIN_INIT:
    return "PLAIN_INIT";
  case ego_planner::core::SeedSpec::Kind::ESDF_INIT:
    return "ESDF_INIT";
  case ego_planner::core::SeedSpec::Kind::GUIDE_PATH_INIT:
    return "GUIDE_PATH_INIT";
  case ego_planner::core::SeedSpec::Kind::CORRIDOR_INIT:
    return "CORRIDOR_INIT";
  case ego_planner::core::SeedSpec::Kind::SEMANTIC_INIT:
    return "SEMANTIC_INIT";
  case ego_planner::core::SeedSpec::Kind::WARM_START:
    return "WARM_START";
  case ego_planner::core::SeedSpec::Kind::NONE:
  default:
    return "NONE";
  }
}

int countEnabledFeasibleSets(const ego_planner::core::PlanningProblem &problem,
                             const ego_planner::core::FeasibleSetType type)
{
  int count = 0;
  for (const auto &set : problem.feasible_sets)
  {
    if (set.enabled && set.type == type)
    {
      ++count;
    }
  }
  return count;
}

} // namespace

namespace ego_planner::engine
{

PlannerEngine::PlannerEngine(EGOPlannerManager *planner_manager)
    : planner_manager_(planner_manager)
{
  problem_compiler_.setProblemAdapter(this);
  backend_solver_.reset(new optimization::CompatibilityBackendSolver());
}

solver::StateToStateInitResources PlannerEngine::makeStateToStateInitResources() const
{
  solver::StateToStateInitResources resources;
  if (planner_manager_ == nullptr)
  {
    return resources;
  }

  resources.plan_params = &planner_manager_->pp_;
  resources.traj_container = &planner_manager_->traj_;
  resources.continuous_failures_count = &planner_manager_->continous_failures_count_;
  resources.grid_map = planner_manager_->grid_map_;
  resources.jps_astar = planner_manager_->jps_astar_.get();
  resources.optimizer = planner_manager_->ploy_traj_opt_.get();
  resources.guide_min_clearance = planner_manager_->guide_min_clearance_;
  resources.guide_sparse_min_inner = planner_manager_->guide_sparse_min_inner_;
  resources.guide_sparse_max_inner = planner_manager_->guide_sparse_max_inner_;
  resources.guide_turn_angle_deg = planner_manager_->guide_turn_angle_deg_;
  resources.sfc_progress = planner_manager_->sfc_progress_;
  resources.sfc_range = planner_manager_->sfc_range_;
  return resources;
}

bool PlannerEngine::solveTask(const core::PlanningContext &context,
                              const core::TaskDefinition &task_definition,
                              core::PlanningSolution &solution)
{
  core::PlanningProblem problem;
  if (!problem_compiler_.compile(context, task_definition, problem))
  {
    solution.success = false;
    solution.message = problem.compile_message.empty()
                           ? "problem compiler rejected task"
                           : problem.compile_message;
    return false;
  }

  const ros::Time solve_start = ros::Time::now();
  const bool ok = solveProblem(problem, solution);
  solution.solve_time_ms = (ros::Time::now() - solve_start).toSec() * 1.0e3;
  return ok;
}

bool PlannerEngine::solveTask(const core::PlanningContext &context,
                              const core::TaskSpec &task,
                              core::PlanningSolution &solution)
{
  return solveTask(context, core::TaskDefinition::fromTaskSpec(task), solution);
}

bool PlannerEngine::solveProblem(const core::PlanningProblem &problem,
                                 core::PlanningSolution &solution)
{
  return backend_solver_ && backend_solver_->solve(problem, solution);
}

bool PlannerEngine::solveCompatibility(const core::PlanningProblem &problem,
                                       core::PlanningSolution &solution)
{
  switch (problem.task.type)
  {
  case core::TaskType::STATE_TO_STATE:
    return solveStateToStateLegacy(problem.task, solution);
  case core::TaskType::TRACKING:
    return solveTrackingLegacyTask(problem.task, solution);
  case core::TaskType::PERCHING:
    return solvePerchingLegacyTask(problem.task, solution);
  case core::TaskType::UNKNOWN:
  default:
    solution.success = false;
    solution.used_legacy_adapter = true;
    solution.message = "legacy compatibility rejected unknown task type";
    return false;
  }
}

bool PlannerEngine::solveStateToStateLegacy(const core::TaskSpec &task,
                                            core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }

  const bool success = planner_manager_->reboundReplan(task.start_pt,
                                                       task.start_vel,
                                                       task.start_acc,
                                                       task.goal_pt,
                                                       task.goal_vel,
                                                       task.flag_poly_init,
                                                       task.flag_random_poly_traj,
                                                       task.touch_goal,
                                                       task.force_plain,
                                                       nullptr,
                                                       task.preferred_guide_path.empty() ? nullptr : &task.preferred_guide_path,
                                                       nullptr);
  solution.success = success;
  solution.used_legacy_adapter = true;
  solution.touch_goal = task.touch_goal;
  solution.message = success ? "legacy state-to-state solve success" : "legacy state-to-state solve failed";
  if (success)
  {
    solution.trajectory = planner_manager_->traj_.local_traj.traj;
    if (planner_manager_->traj_.local_traj.has_yaw_ref)
    {
      solution.has_yaw_ref = true;
      solution.yaw_time = planner_manager_->traj_.local_traj.yaw_time;
      solution.yaw_ref = planner_manager_->traj_.local_traj.yaw_ref;
    }
  }
  return success;
}

bool PlannerEngine::solveTrackingLegacyTask(const core::TaskSpec &task,
                                            core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }

  const bool success = planner_manager_->planTrackingTask(task.tracking_reference,
                                                          task.start_pt,
                                                          task.start_vel,
                                                          task.start_acc,
                                                          task.flag_poly_init,
                                                          task.flag_random_poly_traj,
                                                          task.force_plain);
  solution.success = success;
  solution.used_legacy_adapter = true;
  solution.touch_goal = false;
  solution.message = success ? "legacy tracking solve success" : "legacy tracking solve failed";
  if (success)
  {
    solution.trajectory = planner_manager_->traj_.local_traj.traj;
    if (planner_manager_->traj_.local_traj.has_yaw_ref)
    {
      solution.has_yaw_ref = true;
      solution.yaw_time = planner_manager_->traj_.local_traj.yaw_time;
      solution.yaw_ref = planner_manager_->traj_.local_traj.yaw_ref;
    }
    if (planner_manager_->have_active_tracking_semantic_guide_)
    {
      solution.has_tracking_semantic_guide = true;
      solution.tracking_semantic_guide = planner_manager_->active_tracking_semantic_guide_;
    }
  }
  return success;
}

bool PlannerEngine::solvePerchingLegacyTask(const core::TaskSpec &task,
                                            core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }

  const bool success = planner_manager_->reboundReplan(task.start_pt,
                                                       task.start_vel,
                                                       task.start_acc,
                                                       task.goal_pt,
                                                       task.goal_vel,
                                                       task.flag_poly_init,
                                                       task.flag_random_poly_traj,
                                                       true,
                                                       task.force_plain,
                                                       nullptr,
                                                       task.preferred_guide_path.empty() ? nullptr : &task.preferred_guide_path,
                                                       nullptr);
  solution.success = success;
  solution.used_legacy_adapter = true;
  solution.touch_goal = true;
  solution.message = success ? "legacy perching solve success" : "legacy perching solve failed";
  if (success)
  {
    solution.trajectory = planner_manager_->traj_.local_traj.traj;
  }
  return success;
}

bool PlannerEngine::solveStateToStateCompiledProblem(const core::PlanningProblem &problem,
                                                     core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }

  if (!planner_manager_->enable_compiled_state2state_)
  {
    return solveStateToStateLegacy(problem.task, solution);
  }

  const core::TaskDefinition &task_definition = problem.task_definition;
  const core::TaskSpec &task = problem.task;

  bool compiled_use_corridor = false;
  bool compiled_use_esdf = false;
  bool compiled_force_plain = task_definition.space_model_policy.force_plain || task.force_plain;
  const char *mode_str = "PLAIN";

  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    compiled_use_corridor = !task.force_plain;
    compiled_force_plain = task.force_plain;
    mode_str = compiled_use_corridor ? "CORRIDOR" : "PLAIN";
    break;
  case core::ActiveSpaceModel::ESDF:
    compiled_use_esdf = !task.force_plain;
    compiled_force_plain = task.force_plain;
    mode_str = compiled_use_esdf ? "ESDF" : "PLAIN";
    break;
  case core::ActiveSpaceModel::PLAIN:
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  default:
    compiled_force_plain = true;
    mode_str = "PLAIN";
    break;
  }

  const char *manager_default_mode =
      managerDefaultModeString(planner_manager_->use_sfc_corridor_, planner_manager_->use_esdf_);
  const char *compiled_active_mode =
      activeSpaceModelString(problem.active_space_model);
  const char *task_pref =
      spaceModelPreferenceString(task_definition.space_model_policy.preferred);
  const int corridor_set_count =
      countEnabledFeasibleSets(problem, core::FeasibleSetType::CORRIDOR_POLYTOPE);
  ROS_INFO("[CompiledS2S] task_pref=%s manager_default_mode=%s active_mode=%s selected_mode=%s force_plain=%s fallback_to_legacy=%s guide_pts=%zu corridor_sets=%d seed_kind=%s corridor_aware=%s feasible_sets=%zu",
           task_pref,
           manager_default_mode,
           compiled_active_mode,
           mode_str,
           compiled_force_plain ? "yes" : "no",
           planner_manager_->allow_compiled_state2state_legacy_fallback_ ? "enabled" : "disabled",
           problem.references.guide_path.size(),
           corridor_set_count,
           seedKindString(problem.seed.kind),
           problem.seed.corridor_aware ? "yes" : "no",
           problem.feasible_sets.size());

  bool fallback_attempted = false;
  const auto fillCompiledFailure = [&](const std::string &reason) -> bool
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.touch_goal = task_definition.runtime_policy.touch_goal;
    solution.message =
        std::string("compiled state-to-state solve failed; active_mode=") +
        mode_str +
        " fallback_attempted=" + (fallback_attempted ? "yes" : "no") +
        " reason=" + reason;
    ROS_WARN("[CompiledS2S] active_mode=%s fallback_to_legacy=%s reason=%s",
             compiled_active_mode,
             fallback_attempted ? "attempted" : "no",
             reason.c_str());
    return false;
  };

  const auto runModePinnedLegacyFallback =
      [&](const std::string &reason,
          const std::vector<Eigen::Vector3d> *preferred_guide_path) -> bool
  {
    if (!planner_manager_->allow_compiled_state2state_legacy_fallback_)
    {
      return fillCompiledFailure(reason);
    }

    fallback_attempted = true;
    ROS_WARN("[CompiledS2S] fallback_to_legacy=yes active_mode=%s reason=%s",
             compiled_active_mode,
             reason.c_str());

    const bool prev_use_corridor = planner_manager_->use_sfc_corridor_;
    const bool prev_use_esdf = planner_manager_->use_esdf_;
    planner_manager_->use_sfc_corridor_ = compiled_use_corridor;
    planner_manager_->use_esdf_ = compiled_use_esdf;
    const bool ok = planner_manager_->reboundReplan(task.start_pt,
                                                    task.start_vel,
                                                    task.start_acc,
                                                    task.goal_pt,
                                                    task.goal_vel,
                                                    task.flag_poly_init,
                                                    task.flag_random_poly_traj,
                                                    task.touch_goal,
                                                    compiled_force_plain,
                                                    nullptr,
                                                    preferred_guide_path,
                                                    nullptr);
    planner_manager_->use_sfc_corridor_ = prev_use_corridor;
    planner_manager_->use_esdf_ = prev_use_esdf;

    if (!ok)
    {
      return fillCompiledFailure(reason + "; mode-preserving legacy fallback also failed");
    }

    solution.success = true;
    solution.used_legacy_adapter = true;
    solution.touch_goal = task_definition.runtime_policy.touch_goal;
    solution.message =
        std::string("compiled state-to-state fallback success; active_mode=") +
        mode_str + " reason=" + reason;
    solution.trajectory = planner_manager_->traj_.local_traj.traj;
    if (planner_manager_->traj_.local_traj.has_yaw_ref)
    {
      solution.has_yaw_ref = true;
      solution.yaw_time = planner_manager_->traj_.local_traj.yaw_time;
      solution.yaw_ref = planner_manager_->traj_.local_traj.yaw_ref;
    }
    return true;
  };

  if (problem.representation != core::RepresentationKind::MINCO ||
      !problem.start_boundary.valid ||
      !problem.terminal_boundary.valid)
  {
    return runModePinnedLegacyFallback("compiled problem is missing valid MINCO boundaries", nullptr);
  }

  state_to_state_initializer_.setResources(makeStateToStateInitResources());
  solver::StateToStateInitializationResult init_result;
  if (!state_to_state_initializer_.initialize(problem, init_result))
  {
    if (compiled_use_corridor)
    {
      switch (init_result.failure_type)
      {
      case solver::StateToStateInitFailureType::LOCAL_TARGET_INVALID:
        planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_LOCAL_TARGET_INVALID, init_result.failure_reason);
        break;
      case solver::StateToStateInitFailureType::GUIDE_PATH_TOO_CLOSE:
        planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_GUIDE_PATH_TOO_CLOSE, init_result.failure_reason);
        break;
      case solver::StateToStateInitFailureType::CORRIDOR_GENERATION:
        planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_GENERATION, init_result.failure_reason);
        break;
      case solver::StateToStateInitFailureType::CORRIDOR_INIT:
        planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_INIT, init_result.failure_reason);
        break;
      case solver::StateToStateInitFailureType::NONE:
      default:
        break;
      }
    }
    const std::string init_failure =
        init_result.message.empty() ? init_result.failure_reason : init_result.message;
    return runModePinnedLegacyFallback("compiled stable initialization failed: " + init_failure,
                                       init_result.guide_path.size() >= 2 ? &init_result.guide_path : nullptr);
  }

  MINCOBoundaryState3D &headState = init_result.head_state;
  MINCOBoundaryState3D &tailState = init_result.tail_state;
  Eigen::MatrixXd &innerPts = init_result.inner_points;
  Eigen::VectorXd &durations = init_result.durations;
  MINCOTraj3D &initTraj = init_result.init_traj;
  spatial_map::PolyhedraH &corridor_hpolys = init_result.corridor_hpolys;
  Eigen::VectorXi &corridor_piece_idx = init_result.corridor_piece_idx;
  std::vector<Eigen::Vector3d> &active_guide_path = init_result.guide_path;
  const std::vector<Eigen::Vector3d> &display_path =
      init_result.dense_path.empty() ? init_result.guide_path : init_result.dense_path;
  const std::string &solver_init_source = init_result.init_source;
  const char *selected_mode_str = activeSpaceModelString(init_result.selected_mode);

  if (durations.size() <= 0 || !durations.allFinite())
  {
    return runModePinnedLegacyFallback("compiled stable initialization returned invalid durations",
                                       active_guide_path.size() >= 2 ? &active_guide_path : nullptr);
  }
  if (innerPts.cols() != std::max(0, static_cast<int>(durations.size()) - 1))
  {
    return runModePinnedLegacyFallback("compiled stable initialization returned invalid inner-point layout",
                                       active_guide_path.size() >= 2 ? &active_guide_path : nullptr);
  }

  if (compiled_use_corridor)
  {
    if (corridor_hpolys.empty())
    {
      planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_GENERATION,
                                              "compiled stable initialization did not produce corridor geometry");
      return runModePinnedLegacyFallback("compiled corridor mode has no corridor after stable initialization",
                                         active_guide_path.size() >= 2 ? &active_guide_path : nullptr);
    }
    if (corridor_piece_idx.size() != static_cast<int>(corridor_hpolys.size()) ||
        corridor_piece_idx.sum() != durations.size())
    {
      planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_INIT,
                                              "compiled stable initialization has invalid corridor piece allocation");
      return runModePinnedLegacyFallback("compiled stable corridor init has invalid piece layout",
                                         active_guide_path.size() >= 2 ? &active_guide_path : nullptr);
    }
  }

  ROS_INFO("[CompiledS2S] compiler_hint_guide_pts=%zu compiler_hint_corridor_sets=%d compiler_hint_seed_kind=%s",
           problem.references.guide_path.size(),
           corridor_set_count,
           seedKindString(problem.seed.kind));
  ROS_INFO("[CompiledS2SInit] requested_mode=%s selected_mode=%s solver_init_source=%s compiler_hint_attempted=%s compiler_hint_succeeded=%s stable_helper_attempted=%s stable_helper_succeeded=%s final_guide_pts=%zu final_corridor_polys=%zu final_init_pieces=%ld",
           mode_str,
           selected_mode_str,
           solver_init_source.c_str(),
           init_result.compiler_hint_attempted ? "yes" : "no",
           init_result.compiler_hint_succeeded ? "yes" : "no",
           init_result.stable_helper_attempted ? "yes" : "no",
           init_result.stable_helper_succeeded ? "yes" : "no",
           active_guide_path.size(),
           corridor_hpolys.size(),
           static_cast<long>(durations.size()));

  planner_manager_->ploy_traj_opt_->setIfTouchGoal(task_definition.runtime_policy.touch_goal);

  Eigen::MatrixXd cstr_pts =
      initTraj.getInitConstraintPoints(planner_manager_->ploy_traj_opt_->get_cps_num_prePiece_());
  std::vector<std::pair<int, int>> segments;
  if (!compiled_use_corridor && !compiled_use_esdf)
  {
    if (planner_manager_->ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initTraj, cstr_pts, true) ==
        PolyTrajOptimizer::CHK_RET::ERR)
    {
      return runModePinnedLegacyFallback("compiled plain seed failed initial collision checking",
                                         active_guide_path.size() >= 2 ? &active_guide_path : nullptr);
    }
  }

  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(static_cast<std::size_t>(cstr_pts.cols()));
  for (int i = 0; i < cstr_pts.cols(); ++i)
  {
    point_set.push_back(cstr_pts.col(i));
  }
  if (planner_manager_->visualization_)
  {
    if (compiled_use_esdf)
    {
      if (!display_path.empty())
      {
        planner_manager_->visualization_->displayGlobalPathList(display_path, 0.08, 1);
      }
      if (!active_guide_path.empty())
      {
        planner_manager_->visualization_->displayFrontendList(active_guide_path, 0.10, 1);
      }
    }
    else if (compiled_use_corridor)
    {
      if (!display_path.empty())
      {
        planner_manager_->visualization_->displayGlobalPathList(display_path, 0.08, 0);
      }
      if (!active_guide_path.empty())
      {
        planner_manager_->visualization_->displayFrontendList(active_guide_path, 0.12, 0);
      }
      if (!corridor_hpolys.empty())
      {
        std::vector<Eigen::Vector3d> tri, edges;
        buildCorridorVisualization(corridor_hpolys, tri, edges);
        planner_manager_->visualization_->displayCorridor(tri, edges, 0);
      }
    }
    planner_manager_->visualization_->displayInitPathList(point_set, 0.2, 0);
  }

  const ros::Time t_start = ros::Time::now();
  bool flag_success = false;
  std::vector<std::vector<Eigen::Vector3d>> vis_trajs;

  if (compiled_use_corridor)
  {
    double final_cost = 0.0;
    flag_success = planner_manager_->ploy_traj_opt_->optimizeTrajectory(headState,
                                                                        tailState,
                                                                        innerPts,
                                                                        durations,
                                                                        corridor_hpolys,
                                                                        &corridor_piece_idx,
                                                                        final_cost);

    if (flag_success)
    {
      const MINCOTraj3D opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
      ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f inside_corridor=%s",
               state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj),
               planner_manager_->ploy_traj_opt_->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
      planner_manager_->setLocalTrajFromOpt(opt_traj, task_definition.runtime_policy.touch_goal);
      planner_manager_->have_active_tracking_semantic_guide_ = false;
      planner_manager_->active_tracking_semantic_guide_.clear();
      planner_manager_->active_tracking_corridor_.clear();
      cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
      if (planner_manager_->visualization_)
      {
        planner_manager_->visualization_->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      const MINCOTraj3D &opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
      ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
               planner_manager_->ploy_traj_opt_->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
               state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj),
               planner_manager_->ploy_traj_opt_->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
      planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_OPT,
                                              "compiled corridor optimization rejected seed");
    }
  }
  else if (compiled_use_esdf)
  {
    double final_cost = 0.0;
    flag_success = planner_manager_->ploy_traj_opt_->optimizeTrajectoryWithDistanceField(headState,
                                                                                          tailState,
                                                                                          innerPts,
                                                                                          durations,
                                                                                          final_cost);
    if (flag_success)
    {
      const MINCOTraj3D opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      const double esdf_tol =
          planner_manager_->grid_map_ ? -std::max(0.02, 0.5 * planner_manager_->grid_map_->getResolution()) : 0.0;
      ROS_INFO("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f",
               min_sdf >= esdf_tol ? "yes" : "no",
               min_sdf);
      planner_manager_->setLocalTrajFromOpt(opt_traj, task_definition.runtime_policy.touch_goal);
      planner_manager_->have_active_tracking_semantic_guide_ = false;
      planner_manager_->active_tracking_semantic_guide_.clear();
      planner_manager_->active_tracking_corridor_.clear();
      cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
      if (planner_manager_->visualization_)
      {
        planner_manager_->visualization_->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      const MINCOTraj3D &opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f",
               min_sdf >= (planner_manager_->grid_map_ ? -std::max(0.02, 0.5 * planner_manager_->grid_map_->getResolution()) : 0.0) ? "yes" : "no",
               min_sdf);
    }
  }
  else if (planner_manager_->pp_.use_multitopology_trajs)
  {
    std::vector<Types::ConstraintPoints> trajs = planner_manager_->ploy_traj_opt_->distinctiveTrajs(segments);
    Eigen::VectorXi success = Eigen::VectorXi::Zero(static_cast<int>(trajs.size()));
    double final_cost = 0.0;
    double min_cost = 999999.0;
    MINCOTraj3D best_traj;

    for (int i = static_cast<int>(trajs.size()) - 1; i >= 0; --i)
    {
      planner_manager_->ploy_traj_opt_->setConstraintPoints(trajs[static_cast<std::size_t>(i)]);
      planner_manager_->ploy_traj_opt_->setUseMultitopologyTrajs(true);
      if (planner_manager_->ploy_traj_opt_->optimizeTrajectory(headState,
                                                               tailState,
                                                               innerPts,
                                                               durations,
                                                               final_cost))
      {
        success(i) = true;
        if (final_cost < min_cost)
        {
          min_cost = final_cost;
          best_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
          flag_success = true;
        }

        const MINCOTraj3D vis_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
        const Eigen::MatrixXd ctrl_pts_temp =
            vis_traj.getInitConstraintPoints(planner_manager_->ploy_traj_opt_->get_cps_num_prePiece_());
        std::vector<Eigen::Vector3d> vis_pts;
        vis_pts.reserve(static_cast<std::size_t>(ctrl_pts_temp.cols()));
        for (int j = 0; j < ctrl_pts_temp.cols(); ++j)
        {
          vis_pts.push_back(ctrl_pts_temp.col(j));
        }
        vis_trajs.push_back(vis_pts);
      }
    }

    if (flag_success)
    {
      planner_manager_->setLocalTrajFromOpt(best_traj, task_definition.runtime_policy.touch_goal);
      cstr_pts = sampleTrajectoryForDisplay(best_traj, 0.02);
      if (planner_manager_->visualization_)
      {
        planner_manager_->visualization_->displayOptimalList(cstr_pts, 0);
      }
    }
  }
  else
  {
    double final_cost = 0.0;
    flag_success = planner_manager_->ploy_traj_opt_->optimizeTrajectory(headState,
                                                                        tailState,
                                                                        innerPts,
                                                                        durations,
                                                                        final_cost);

    if (flag_success)
    {
      MINCOTraj3D opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      ROS_INFO("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f corridor_check=n/a",
               planner_manager_->ploy_traj_opt_->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
               min_sdf);
      planner_manager_->setLocalTrajFromOpt(opt_traj, task_definition.runtime_policy.touch_goal);
      cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
      if (planner_manager_->visualization_)
      {
        planner_manager_->visualization_->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      const MINCOTraj3D &opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f corridor_check=n/a",
               planner_manager_->ploy_traj_opt_->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
               min_sdf);
    }
  }

  planner_manager_->pp_.time_optimize_ = (ros::Time::now() - t_start).toSec();
  solution.success = flag_success;
  solution.used_legacy_adapter = false;
  solution.touch_goal = task_definition.runtime_policy.touch_goal;
  solution.message = flag_success
                         ? std::string("compiled state-to-state solve success; selected_mode=") +
                               selected_mode_str +
                               " init_source=" + solver_init_source
                         : std::string("compiled state-to-state solve failed after optimizer; selected_mode=") +
                               selected_mode_str +
                               " init_source=" + solver_init_source;
  if (flag_success)
  {
    solution.trajectory = planner_manager_->traj_.local_traj.traj;
    if (planner_manager_->traj_.local_traj.has_yaw_ref)
    {
      solution.has_yaw_ref = true;
      solution.yaw_time = planner_manager_->traj_.local_traj.yaw_time;
      solution.yaw_ref = planner_manager_->traj_.local_traj.yaw_ref;
    }
  }
  return flag_success;
}

bool PlannerEngine::solveTrackingCompiledProblem(const core::PlanningProblem &problem,
                                                 core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }

  if (problem.representation != core::RepresentationKind::MINCO ||
      !problem.start_boundary.valid ||
      !problem.terminal_boundary.valid)
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking problem has invalid MINCO boundaries";
    if (problem.prefer_legacy_fallback)
    {
      return solveTrackingLegacyTask(problem.task, solution);
    }
    return false;
  }

  ::cost_functional::TrackingReference tracking_ref_raw;
  if (problem.references.has_tracking_reference &&
      problem.references.tracking_reference.valid())
  {
    tracking_ref_raw = problem.references.tracking_reference;
  }
  else if (problem.task.tracking_reference.valid())
  {
    tracking_ref_raw = problem.task.tracking_reference;
  }
  else
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking solve missing valid tracking reference";
    if (problem.prefer_legacy_fallback)
    {
      return solveTrackingLegacyTask(problem.task, solution);
    }
    return false;
  }

  ::cost_functional::TrackingReference tracking_ref;
  std::string tracking_ref_reason;
  if (!::cost_functional::normalizeTrackingReference(tracking_ref_raw,
                                                     tracking_ref,
                                                     &tracking_ref_reason))
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking reference normalization failed: " + tracking_ref_reason;
    if (problem.prefer_legacy_fallback)
    {
      return solveTrackingLegacyTask(problem.task, solution);
    }
    return false;
  }

  const char *active_mode = activeSpaceModelString(problem.active_space_model);
  ROS_INFO("[CompiledTracking] active_mode=%s target_ref=%zu view_ref=%zu compiler_hint_guide=%zu seed_kind=%s",
           active_mode,
           tracking_ref.t_ref.size(),
           tracking_ref.t_view_ref.size(),
           problem.references.guide_path.size(),
           seedKindString(problem.seed.kind));

  state_to_state_initializer_.setResources(makeStateToStateInitResources());
  solver::StateToStateInitializationResult init_result;
  if (!state_to_state_initializer_.initialize(problem, init_result))
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking initialization failed: " + init_result.failure_reason;
    if (problem.prefer_legacy_fallback)
    {
      return solveTrackingLegacyTask(problem.task, solution);
    }
    return false;
  }

  ROS_INFO("[CompiledTrackingInit] active_mode=%s init_source=%s final_guide_pts=%zu final_corridor_polys=%zu final_init_pieces=%ld",
           active_mode,
           init_result.init_source.c_str(),
           init_result.guide_path.size(),
           init_result.corridor_hpolys.size(),
           static_cast<long>(init_result.durations.size()));

  planner_manager_->ploy_traj_opt_->setIfTouchGoal(false);

  if (planner_manager_->visualization_)
  {
    const std::vector<Eigen::Vector3d> &display_path =
        init_result.dense_path.empty() ? init_result.guide_path : init_result.dense_path;
    if (!display_path.empty())
    {
      planner_manager_->visualization_->displayGlobalPathList(display_path, 0.08, 3);
    }
    if (!init_result.guide_path.empty())
    {
      planner_manager_->visualization_->displayFrontendList(init_result.guide_path, 0.10, 3);
    }
    if (!init_result.corridor_hpolys.empty())
    {
      std::vector<Eigen::Vector3d> tri, edges;
      buildCorridorVisualization(init_result.corridor_hpolys, tri, edges);
      planner_manager_->visualization_->displayCorridor(tri, edges, 3);
    }
    const Eigen::MatrixXd init_display =
        init_result.init_traj.getInitConstraintPoints(planner_manager_->ploy_traj_opt_->get_cps_num_prePiece_());
    std::vector<Eigen::Vector3d> init_pts;
    init_pts.reserve(static_cast<std::size_t>(std::max<Eigen::Index>(0, init_display.cols())));
    for (int i = 0; i < init_display.cols(); ++i)
    {
      init_pts.push_back(init_display.col(i));
    }
    if (!init_pts.empty())
    {
      planner_manager_->visualization_->displayInitPathList(init_pts, 0.16, 3);
    }
  }

  bool ok = false;
  double final_cost = 0.0;
  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    if (init_result.corridor_hpolys.empty())
    {
      solution.success = false;
      solution.used_legacy_adapter = false;
      solution.message = "compiled tracking corridor mode missing corridor geometry";
      return false;
    }
    ok = planner_manager_->ploy_traj_opt_->optimizeTrackingTrajectory(
        init_result.head_state,
        init_result.tail_state,
        init_result.inner_points,
        init_result.durations,
        init_result.corridor_hpolys,
        &init_result.corridor_piece_idx,
        tracking_ref,
        final_cost);
    break;
  case core::ActiveSpaceModel::ESDF:
    ok = planner_manager_->ploy_traj_opt_->optimizeTrackingTrajectoryWithDistanceField(
        init_result.head_state,
        init_result.tail_state,
        init_result.inner_points,
        init_result.durations,
        tracking_ref,
        nullptr,
        final_cost);
    break;
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  case core::ActiveSpaceModel::PLAIN:
  default:
    ok = planner_manager_->ploy_traj_opt_->optimizeTrackingTrajectory(
        init_result.head_state,
        init_result.tail_state,
        init_result.inner_points,
        init_result.durations,
        tracking_ref,
        nullptr,
        final_cost);
    break;
  }

  solution.success = ok;
  solution.used_legacy_adapter = false;
  solution.touch_goal = false;
  solution.message = ok
                         ? std::string("compiled tracking solve success; active_mode=") + active_mode
                         : std::string("compiled tracking solve failed after optimizer; active_mode=") + active_mode;
  if (!ok)
  {
    return false;
  }

  const MINCOTraj3D opt_traj = planner_manager_->ploy_traj_opt_->getTrajectory();
  planner_manager_->setLocalTrajFromOpt(opt_traj, false);
  planner_manager_->have_active_tracking_semantic_guide_ = false;
  planner_manager_->active_tracking_semantic_guide_.clear();
  planner_manager_->active_tracking_corridor_.clear();

  const double yaw0 =
      (problem.start_boundary.velocity.head<2>().norm() > 1.0e-3)
          ? std::atan2(problem.start_boundary.velocity.y(), problem.start_boundary.velocity.x())
          : 0.0;
  auto yaw_plan = TrackingYawPlanner::planFacingTarget(opt_traj,
                                                       tracking_ref,
                                                       0.05,
                                                       1.2,
                                                       yaw0);
  planner_manager_->traj_.setLocalYawRef(yaw_plan.t, yaw_plan.yaw);

  solution.trajectory = planner_manager_->traj_.local_traj.traj;
  solution.has_yaw_ref = planner_manager_->traj_.local_traj.has_yaw_ref;
  solution.yaw_time = planner_manager_->traj_.local_traj.yaw_time;
  solution.yaw_ref = planner_manager_->traj_.local_traj.yaw_ref;

  if (planner_manager_->visualization_)
  {
    const Eigen::MatrixXd opt_display = sampleTrajectoryForDisplay(opt_traj, 0.02);
    planner_manager_->visualization_->displayOptimalList(opt_display, 3);
  }
  return true;
}

bool PlannerEngine::solveStateToStateCompiled(const core::PlanningProblem &problem,
                                              core::PlanningSolution &solution)
{
  return solveStateToStateCompiledProblem(problem, solution);
}

bool PlannerEngine::solveTrackingCompiled(const core::PlanningProblem &problem,
                                          core::PlanningSolution &solution)
{
  return solveTrackingCompiledProblem(problem, solution);
}

bool PlannerEngine::solveTrackingLegacy(const core::PlanningProblem &problem,
                                        core::PlanningSolution &solution)
{
  return solveTrackingLegacyTask(problem.task, solution);
}

bool PlannerEngine::solvePerchingLegacy(const core::PlanningProblem &problem,
                                        core::PlanningSolution &solution)
{
  return solvePerchingLegacyTask(problem.task, solution);
}

} // namespace ego_planner::engine
