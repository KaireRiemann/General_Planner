#include <engine/planner_engine.hpp>

#include <plan_manage/planner_manager.h>
#include <plan_manage/tracking_yaw_planner.hpp>
#include <MINCOTrajectory/terminal_mapping.hpp>
#include <frontend/perching_init_service.hpp>
#include <frontend/transit_init_service.hpp>
#include <optimization/backend_plugin_input.hpp>
#include <runtime/context_builder.hpp>
#include <SFCGenerator/geo_utils.hpp>
#include <SFCGenerator/quickhull.hpp>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>

namespace
{

using ego_planner::EGOPlannerManager;
using ego_planner::MINCOBoundaryState3D;
using ego_planner::MINCOTraj3D;
using ego_planner::SnapBoundaryState3D;
using ego_planner::SnapTraj3D;
using ego_planner::YawTraj1D;

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

template <typename TrajT>
Eigen::MatrixXd sampleTrajectoryForDisplay(const TrajT &traj,
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

void sampleYawTrajectoryForReference(const YawTraj1D &traj,
                                     const double dt,
                                     std::vector<double> &t_ref,
                                     std::vector<double> &yaw_ref)
{
  t_ref.clear();
  yaw_ref.clear();
  const double total_t = traj.getTotalDuration();
  if (total_t <= 1.0e-6)
  {
    return;
  }
  const double clamped_dt = std::max(dt, 1.0e-3);
  const int sample_num =
      std::max(2, static_cast<int>(std::ceil(total_t / clamped_dt)) + 1);
  t_ref.reserve(static_cast<std::size_t>(sample_num));
  yaw_ref.reserve(static_cast<std::size_t>(sample_num));
  for (int i = 0; i < sample_num; ++i)
  {
    const double ratio = (sample_num <= 1) ? 0.0
                                           : static_cast<double>(i) / static_cast<double>(sample_num - 1);
    const double t = ratio * total_t;
    t_ref.push_back(t);
    yaw_ref.push_back(traj.evaluate(t, 0)(0));
  }
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

bool perchingOnlyDebugEnabled()
{
  if (const char *env = std::getenv("PERCHING_ONLY_DEBUG"))
  {
    return env[0] != '\0' && env[0] != '0';
  }
  bool enabled = false;
  ros::param::get("/debug/perching_only", enabled);
  return enabled;
}

MINCOBoundaryState3D makeBoundaryState(const Eigen::Vector3d &pos,
                                       const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &acc)
{
  MINCOBoundaryState3D state = MINCOBoundaryState3D::Zero();
  if constexpr (MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM > 0)
  {
    state.col(0) = pos;
  }
  if constexpr (MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM > 1)
  {
    state.col(1) = vel;
  }
  if constexpr (MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM > 2)
  {
    state.col(2) = acc;
  }
  return state;
}

SnapBoundaryState3D makeSnapBoundaryState(const Eigen::Vector3d &pos,
                                          const Eigen::Vector3d &vel,
                                          const Eigen::Vector3d &acc,
                                          const Eigen::Vector3d &jer)
{
  SnapBoundaryState3D state = SnapBoundaryState3D::Zero();
  if constexpr (SnapTraj3D::BOUNDARY_DERIVATIVE_NUM > 0)
  {
    state.col(0) = pos;
  }
  if constexpr (SnapTraj3D::BOUNDARY_DERIVATIVE_NUM > 1)
  {
    state.col(1) = vel;
  }
  if constexpr (SnapTraj3D::BOUNDARY_DERIVATIVE_NUM > 2)
  {
    state.col(2) = acc;
  }
  if constexpr (SnapTraj3D::BOUNDARY_DERIVATIVE_NUM > 3)
  {
    state.col(3) = jer;
  }
  return state;
}

double estimateMaxPerchingBodyRate(const SnapTraj3D &traj)
{
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const double total_t = traj.getTotalDuration();
  if (total_t <= 1.0e-6)
  {
    return 0.0;
  }

  double max_omega = 0.0;
  const double dt = 0.01;
  const int samples = std::max(2, static_cast<int>(std::ceil(total_t / dt)) + 1);
  for (int i = 0; i < samples; ++i)
  {
    const double t = total_t * static_cast<double>(i) /
                     static_cast<double>(std::max(1, samples - 1));
    const Eigen::Vector3d acc = traj.evaluate(t, 2);
    const Eigen::Vector3d jerk = traj.evaluate(t, 3);
    const Eigen::Vector3d thrust = acc - gravity;
    const double thrust_norm = thrust.norm();
    if (thrust_norm < 1.0e-5)
    {
      return std::numeric_limits<double>::infinity();
    }
    const Eigen::Matrix3d d_normalized =
        (Eigen::Matrix3d::Identity() -
         thrust * thrust.transpose() / thrust.squaredNorm()) /
        thrust_norm;
    max_omega = std::max(max_omega, (d_normalized * jerk).norm());
  }
  return max_omega;
}

bool buildFastPerchingInitArtifact(
    const ego_planner::frontend::TransitInitRuntimeConfig &config,
    const ego_planner::core::PlanningProblem &problem,
    const ego_planner::frontend::PerchingInitArtifact &perching_init,
    ego_planner::frontend::InitArtifact &artifact)
{
  artifact = ego_planner::frontend::InitArtifact{};
  const auto &decoded = perching_init.decoded_contact_semantics;
  if (!decoded.valid || !problem.start_boundary.valid)
  {
    return false;
  }

  const double max_vel =
      config.plan_params != nullptr
          ? std::max(0.2, config.plan_params->max_vel_)
          : 1.5;
  int piece_num = 10;
  ros::param::param("/debug/perching_fast_piece_num", piece_num, piece_num);
  piece_num = std::max(3, std::min(20, piece_num));

  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const Eigen::Vector3d start_pos = problem.start_boundary.position;
  const Eigen::Vector3d start_vel = problem.start_boundary.velocity;
  const Eigen::Vector3d start_acc = problem.start_boundary.acceleration;
  const Eigen::Vector3d start_jerk = Eigen::Vector3d::Zero();

  const auto tailAt = [&](const double total_t,
                          MINCOBoundaryState3D &tail_state,
                          SnapBoundaryState3D &snap_tail_state)
  {
    const double dt_from_ref = total_t - decoded.reference_time;
    const Eigen::Vector3d tail_pos =
        decoded.plate_position_ref +
        decoded.plate_velocity * dt_from_ref +
        decoded.robot_l * decoded.surface_z;
    const Eigen::Vector3d tail_vel =
        decoded.plate_velocity -
        decoded.v_plus * decoded.surface_z;
    const double terminal_thrust =
        decoded.use_dynamics_terminal_accel
            ? decoded.thrust_nominal +
                  decoded.thrust_range * std::sin(decoded.tau_f_seed)
            : 0.0;
    const Eigen::Vector3d tail_acc =
        decoded.use_dynamics_terminal_accel
            ? terminal_thrust * decoded.surface_z + gravity
            : perching_init.predicted_contact_state.contact_acceleration;
    tail_state = makeBoundaryState(tail_pos, tail_vel, tail_acc);
    snap_tail_state =
        makeSnapBoundaryState(tail_pos, tail_vel, tail_acc, Eigen::Vector3d::Zero());
  };

  const Eigen::Vector3d nominal_contact =
      perching_init.predicted_contact_state.valid
          ? perching_init.predicted_contact_state.contact_position
          : decoded.plate_position_ref + decoded.robot_l * decoded.surface_z;
  double total_t = std::max(decoded.reference_time,
                            (nominal_contact - start_pos).norm() / max_vel);
  total_t = std::max(total_t, 0.15 * static_cast<double>(piece_num));

  const SnapBoundaryState3D snap_head =
      makeSnapBoundaryState(start_pos, start_vel, start_acc, start_jerk);
  const Eigen::MatrixXd no_inner(3, 0);
  Eigen::VectorXd one_piece_time(1);
  SnapTraj3D one_piece_bvp;
  MINCOBoundaryState3D tail_state = MINCOBoundaryState3D::Zero();
  SnapBoundaryState3D snap_tail_state = SnapBoundaryState3D::Zero();

  double max_omega = std::numeric_limits<double>::infinity();
  bool have_bvp = false;
  constexpr double kFastPerchingOmegaLimit = 4.5;
  for (int attempt = 0; attempt < 10; ++attempt)
  {
    tailAt(total_t, tail_state, snap_tail_state);
    one_piece_time(0) = total_t;
    if (one_piece_bvp.generate(no_inner, snap_head, snap_tail_state, one_piece_time))
    {
      max_omega = estimateMaxPerchingBodyRate(one_piece_bvp);
      have_bvp = std::isfinite(max_omega) && max_omega <= kFastPerchingOmegaLimit;
      if (have_bvp)
      {
        break;
      }
    }
    total_t += 0.5;
  }
  if (!have_bvp)
  {
    tailAt(total_t, tail_state, snap_tail_state);
    one_piece_time(0) = total_t;
    have_bvp = one_piece_bvp.generate(no_inner, snap_head, snap_tail_state, one_piece_time);
    max_omega = have_bvp ? estimateMaxPerchingBodyRate(one_piece_bvp)
                         : std::numeric_limits<double>::infinity();
  }
  if (!have_bvp)
  {
    return false;
  }

  artifact.head_state = makeBoundaryState(start_pos, start_vel, start_acc);
  artifact.tail_state = tail_state;
  artifact.durations = Eigen::VectorXd::Constant(piece_num,
                                                 total_t / static_cast<double>(piece_num));
  artifact.inner_points.resize(3, piece_num - 1);
  artifact.guide_path.clear();
  artifact.guide_path.reserve(static_cast<std::size_t>(piece_num + 1));
  artifact.guide_path.push_back(start_pos);
  for (int i = 1; i < piece_num; ++i)
  {
    const double t = total_t * static_cast<double>(i) /
                     static_cast<double>(piece_num);
    artifact.inner_points.col(i - 1) = one_piece_bvp.evaluate(t, 0);
    artifact.guide_path.push_back(artifact.inner_points.col(i - 1));
  }
  artifact.guide_path.push_back(tail_state.col(0));
  artifact.dense_path = artifact.guide_path;

  if (!artifact.init_traj.generate(artifact.inner_points,
                                   artifact.head_state,
                                   artifact.tail_state,
                                   artifact.durations))
  {
    return false;
  }

  artifact.source = "fast_perching_bvp_direct";
  artifact.valid = artifact.init_traj.getTotalDuration() > 1.0e-6;
  artifact.collision_free = true;
  artifact.inside_corridor = true;
  ROS_INFO("[FastPerchingInit] direct BVP seed pieces=%d T=%.2f dT=%.2f max_omega_seed=%.2f ref_t=%.2f start=[%.2f %.2f %.2f] tail=[%.2f %.2f %.2f]",
           piece_num,
           total_t,
           total_t / static_cast<double>(piece_num),
           max_omega,
           decoded.reference_time,
           start_pos.x(),
           start_pos.y(),
           start_pos.z(),
           tail_state.col(0).x(),
           tail_state.col(0).y(),
           tail_state.col(0).z());
  return artifact.valid;
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

void populateInitArtifacts(const ego_planner::frontend::InitArtifact &init_artifact,
                           ego_planner::core::PlanningSolution &solution)
{
  solution.has_init_artifacts = init_artifact.valid;
  solution.init_source = init_artifact.source;
  solution.guide_path = init_artifact.guide_path;
  solution.dense_path = init_artifact.dense_path;
  solution.corridor_hpolys = init_artifact.corridor_hpolys;
  solution.corridor_piece_idx = init_artifact.corridor_piece_idx;
}

ego_planner::optimization::BackendPluginInput makeBackendPluginInput(
    const ego_planner::core::PlanningProblem &problem,
    const ego_planner::frontend::InitArtifact &init_artifact,
    const cost_functional::TrackingReference *tracking_reference = nullptr,
    const cost_functional::TrackingSemanticGuide *tracking_semantic_guide = nullptr)
{
  ego_planner::optimization::BackendPluginInput input;
  input.problem = &problem;
  input.task_semantics = &problem.task_semantics;
  input.transit_init = &init_artifact;
  input.tracking_reference = tracking_reference;
  input.tracking_semantic_guide = tracking_semantic_guide;
  return input;
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
  const core::TaskType task_type =
      problem.task_definition.type != core::TaskType::UNKNOWN
          ? problem.task_definition.type
          : problem.task.type;

  switch (task_type)
  {
  case core::TaskType::STATE_TO_STATE:
    return solveStateToStateCompiledProblem(problem, solution);
  case core::TaskType::TRACKING:
    return solveTrackingCompiledProblem(problem, solution);
  case core::TaskType::PERCHING:
    return solvePerchingCompiledProblem(problem, solution);
  case core::TaskType::UNKNOWN:
  default:
    break;
  }

  if (backend_solver_ != nullptr)
  {
    return backend_solver_->solve(problem, solution);
  }

  solution.success = false;
  solution.message = "planner engine has no backend solver for unknown task";
  return false;
}

bool PlannerEngine::solveCompatibility(const core::PlanningProblem &problem,
                                       core::PlanningSolution &solution)
{
  switch (problem.task.type)
  {
  case core::TaskType::STATE_TO_STATE:
    return solveStateToStateCompiledProblem(problem, solution);
  case core::TaskType::TRACKING:
    return solveTrackingCompiledProblem(problem, solution);
  case core::TaskType::PERCHING:
    return solvePerchingCompiledProblem(problem, solution);
  case core::TaskType::UNKNOWN:
  default:
    solution.success = false;
    solution.used_legacy_adapter = true;
    solution.message = "legacy compatibility rejected unknown task type";
    return false;
  }
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

  runtime::ContextBuilder context_builder;
  const core::PlanningContext planning_context = context_builder.build(
      planner_manager_->pp_.drone_id,
      planner_manager_->grid_map_ != nullptr,
      planner_manager_->grid_map_ != nullptr,
      planner_manager_->grid_map_ != nullptr && planner_manager_->grid_map_->esdfEnabled(),
      planner_manager_->getJpsAstar() != nullptr,
      planner_manager_->grid_map_,
      planner_manager_->getJpsAstar(),
      task.start_pt,
      task.start_vel,
      task.start_acc,
      task.goal_pt,
      task.goal_pt,
      task.goal_vel,
      planner_manager_->pp_.max_vel_,
      planner_manager_->pp_.max_acc_,
      planner_manager_->pp_.polyTraj_piece_length,
      planner_manager_->getGuideMinClearance(),
      planner_manager_->getGuideSparseMinInner(),
      planner_manager_->getGuideSparseMaxInner(),
      planner_manager_->getGuideTurnAngleDeg(),
      planner_manager_->getSfcProgress(),
      planner_manager_->getSfcRange(),
      planner_manager_->getSfcCorridorMargin(),
      &planner_manager_->traj_.local_traj);

  const bool success = solveTask(planning_context, task, solution);
  solution.used_legacy_adapter = true;
  solution.touch_goal = true;
  if (solution.message.empty())
  {
    solution.message = success ? "legacy perching task solved through compiled transit path"
                               : "legacy perching task failed through compiled transit path";
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

  const core::TaskDefinition &task_definition = problem.task_definition;
  const core::TaskSpec &task = problem.task;
  auto *optimizer = planner_manager_->getOptimizer();
  const auto visualization = planner_manager_->getVisualization();

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
      managerDefaultModeString(planner_manager_->managerPrefersCorridor(),
                               planner_manager_->managerPrefersEsdf());
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
           "disabled",
           problem.references.guide_path.size(),
           corridor_set_count,
           seedKindString(problem.seed.kind),
           problem.seed.corridor_aware ? "yes" : "no",
           problem.feasible_sets.size());

  const auto fillCompiledFailure = [&](const std::string &reason) -> bool
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.touch_goal = task_definition.runtime_policy.touch_goal;
    solution.message =
        std::string("compiled state-to-state solve failed; active_mode=") +
        mode_str +
        " fallback_attempted=no reason=" + reason;
    ROS_WARN("[CompiledS2S] active_mode=%s fallback_to_legacy=no reason=%s",
             compiled_active_mode,
             reason.c_str());
    return false;
  };

  if (problem.representation != core::RepresentationKind::MINCO ||
      !problem.start_boundary.valid ||
      !problem.terminal_boundary.valid)
  {
  return fillCompiledFailure("compiled problem is missing valid MINCO boundaries");
  }

  state_to_state_initializer_.setResources(planner_manager_->makeStateToStateInitResources());
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
    return fillCompiledFailure("compiled stable initialization failed: " + init_failure);
  }
  const frontend::InitArtifact &init_artifact = init_result.init_artifact;
  populateInitArtifacts(init_artifact, solution);
  ego_planner::optimization::BackendPluginInput plugin_input = makeBackendPluginInput(problem, init_artifact);
  const frontend::InitArtifact &solver_input = *plugin_input.transit_init;
  solution.active_space_model = init_result.selected_mode;

  const MINCOBoundaryState3D &headState = solver_input.head_state;
  MINCOBoundaryState3D tailState = solver_input.tail_state;
  const Eigen::MatrixXd &innerPts = solver_input.inner_points;
  const Eigen::VectorXd &durations = solver_input.durations;
  const MINCOTraj3D &initTraj = solver_input.init_traj;
  const spatial_map::PolyhedraH &corridor_hpolys = solver_input.corridor_hpolys;
  const Eigen::VectorXi &corridor_piece_idx = solver_input.corridor_piece_idx;
  const std::vector<Eigen::Vector3d> &active_guide_path = solver_input.guide_path;
  const std::vector<Eigen::Vector3d> &display_path =
      solver_input.dense_path.empty() ? solver_input.guide_path : solver_input.dense_path;
  const std::string &solver_init_source = solver_input.source;
  const char *selected_mode_str = activeSpaceModelString(init_result.selected_mode);
  bool touch_goal =
      plugin_input.task_semantics != nullptr
          ? plugin_input.task_semantics->transit.touch_goal
          : task_definition.runtime_policy.touch_goal;

  if (!solver_input.hasValidTiming())
  {
    return fillCompiledFailure("compiled stable initialization returned invalid durations");
  }
  if (!solver_input.hasValidPieceLayout())
  {
    return fillCompiledFailure("compiled stable initialization returned invalid inner-point layout");
  }

  if (compiled_use_corridor)
  {
    if (corridor_hpolys.empty())
    {
      planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_GENERATION,
                                              "compiled stable initialization did not produce corridor geometry");
      return fillCompiledFailure("compiled corridor mode has no corridor after stable initialization");
    }
    if (!solver_input.hasValidCorridorAllocation())
    {
      planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_INIT,
                                              "compiled stable initialization has invalid corridor piece allocation");
      return fillCompiledFailure("compiled stable corridor init has invalid piece layout");
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

  optimizer->setIfTouchGoal(touch_goal);

  Eigen::MatrixXd cstr_pts =
      initTraj.getInitConstraintPoints(optimizer->get_cps_num_prePiece_());
  std::vector<std::pair<int, int>> segments;
  if (!compiled_use_corridor && !compiled_use_esdf)
  {
    if (optimizer->finelyCheckAndSetConstraintPoints(segments, initTraj, cstr_pts, true) ==
        PolyTrajOptimizer::CHK_RET::ERR)
    {
      return fillCompiledFailure("compiled plain seed failed initial collision checking");
    }
  }

  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(static_cast<std::size_t>(cstr_pts.cols()));
  for (int i = 0; i < cstr_pts.cols(); ++i)
  {
    point_set.push_back(cstr_pts.col(i));
  }
  if (visualization)
  {
    if (compiled_use_esdf)
    {
      if (!display_path.empty())
      {
        visualization->displayGlobalPathList(display_path, 0.08, 1);
      }
      if (!active_guide_path.empty())
      {
        visualization->displayFrontendList(active_guide_path, 0.10, 1);
      }
    }
    else if (compiled_use_corridor)
    {
      if (!display_path.empty())
      {
        visualization->displayGlobalPathList(display_path, 0.08, 0);
      }
      if (!active_guide_path.empty())
      {
        visualization->displayFrontendList(active_guide_path, 0.12, 0);
      }
      if (!corridor_hpolys.empty())
      {
        std::vector<Eigen::Vector3d> tri, edges;
        buildCorridorVisualization(corridor_hpolys, tri, edges);
        visualization->displayCorridor(tri, edges, 0);
      }
    }
    visualization->displayInitPathList(point_set, 0.2, 0);
  }

  const ros::Time t_start = ros::Time::now();
  bool flag_success = false;
  std::vector<std::vector<Eigen::Vector3d>> vis_trajs;

  if (compiled_use_corridor)
  {
    double final_cost = 0.0;
    flag_success = optimizer->optimizeTrajectory(headState,
                                                 tailState,
                                                 innerPts,
                                                 durations,
                                                 corridor_hpolys,
                                                 &corridor_piece_idx,
                                                 final_cost);

    if (flag_success)
    {
      const MINCOTraj3D opt_traj = optimizer->getTrajectory();
      ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f inside_corridor=%s",
               state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj),
               optimizer->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
      planner_manager_->setLocalTrajFromOpt(opt_traj, touch_goal);
      planner_manager_->clearActiveTrackingArtifacts();
      cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
      if (visualization)
      {
        visualization->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      const MINCOTraj3D &opt_traj = optimizer->getTrajectory();
      ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
               optimizer->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
               state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj),
               optimizer->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
      planner_manager_->reportCorridorFailure(EGOPlannerManager::FAIL_CORRIDOR_OPT,
                                              "compiled corridor optimization rejected seed");
    }
  }
  else if (compiled_use_esdf)
  {
    double final_cost = 0.0;
    flag_success = optimizer->optimizeTrajectoryWithDistanceField(headState,
                                                                  tailState,
                                                                  innerPts,
                                                                  durations,
                                                                  final_cost);
    if (flag_success)
    {
      const MINCOTraj3D opt_traj = optimizer->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      const double esdf_tol = optimizer->getDistanceFieldCollisionTolerance();
      ROS_INFO("OPT_TRAJ_CHECK: clearance_ok=%s min_sdf=%.3f required_clearance=%.3f",
               min_sdf >= esdf_tol ? "yes" : "no",
               min_sdf,
               esdf_tol);
      planner_manager_->setLocalTrajFromOpt(opt_traj, touch_goal);
      planner_manager_->clearActiveTrackingArtifacts();
      cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
      if (visualization)
      {
        visualization->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      const MINCOTraj3D &opt_traj = optimizer->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      const double esdf_tol = optimizer->getDistanceFieldCollisionTolerance();
      ROS_WARN("OPT_TRAJ_CHECK: clearance_ok=%s min_sdf=%.3f required_clearance=%.3f",
               min_sdf >= esdf_tol ? "yes" : "no",
               min_sdf,
               esdf_tol);
    }
  }
  else if (planner_manager_->pp_.use_multitopology_trajs)
  {
    std::vector<Types::ConstraintPoints> trajs = optimizer->distinctiveTrajs(segments);
    Eigen::VectorXi success = Eigen::VectorXi::Zero(static_cast<int>(trajs.size()));
    double final_cost = 0.0;
    double min_cost = 999999.0;
    MINCOTraj3D best_traj;

    for (int i = static_cast<int>(trajs.size()) - 1; i >= 0; --i)
    {
      optimizer->setConstraintPoints(trajs[static_cast<std::size_t>(i)]);
      optimizer->setUseMultitopologyTrajs(true);
      if (optimizer->optimizeTrajectory(headState,
                                        tailState,
                                        innerPts,
                                        durations,
                                        final_cost))
      {
        success(i) = true;
        if (final_cost < min_cost)
        {
          min_cost = final_cost;
          best_traj = optimizer->getTrajectory();
          flag_success = true;
        }

        const MINCOTraj3D vis_traj = optimizer->getTrajectory();
        const Eigen::MatrixXd ctrl_pts_temp =
            vis_traj.getInitConstraintPoints(optimizer->get_cps_num_prePiece_());
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
      planner_manager_->setLocalTrajFromOpt(best_traj, touch_goal);
      cstr_pts = sampleTrajectoryForDisplay(best_traj, 0.02);
      if (visualization)
      {
        visualization->displayOptimalList(cstr_pts, 0);
      }
    }
  }
  else
  {
    double final_cost = 0.0;
    flag_success = optimizer->optimizeTrajectory(headState,
                                                 tailState,
                                                 innerPts,
                                                 durations,
                                                 final_cost);

    if (flag_success)
    {
      MINCOTraj3D opt_traj = optimizer->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      ROS_INFO("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f corridor_check=n/a",
               optimizer->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
               min_sdf);
      planner_manager_->setLocalTrajFromOpt(opt_traj, touch_goal);
      cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
      if (visualization)
      {
        visualization->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      const MINCOTraj3D &opt_traj = optimizer->getTrajectory();
      const double min_sdf = state_to_state_initializer_.computeTrajectoryMinSdf(opt_traj);
      ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f corridor_check=n/a",
               optimizer->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
               min_sdf);
    }
  }

  planner_manager_->pp_.time_optimize_ = (ros::Time::now() - t_start).toSec();
  solution.success = flag_success;
  solution.used_legacy_adapter = false;
  solution.touch_goal = touch_goal;
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

bool PlannerEngine::solvePerchingCompiledProblem(const core::PlanningProblem &problem,
                                                 core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }

  auto *optimizer = planner_manager_->getOptimizer();
  const auto visualization = planner_manager_->getVisualization();
  const char *compiled_active_mode = activeSpaceModelString(problem.active_space_model);

  if (problem.representation != core::RepresentationKind::MINCO ||
      !problem.start_boundary.valid ||
      !problem.terminal_boundary.valid)
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled perching problem has invalid MINCO boundaries";
    return false;
  }

  const auto init_resources = planner_manager_->makeStateToStateInitResources();
  frontend::TransitInitRuntimeConfig perching_init_config;
  perching_init_config.plan_params = init_resources.plan_params;
  perching_init_config.traj_container = init_resources.traj_container;
  perching_init_config.continuous_failures_count = init_resources.continuous_failures_count;
  perching_init_config.grid_map = init_resources.grid_map;
  perching_init_config.jps_astar = init_resources.jps_astar;
  perching_init_config.optimizer = init_resources.optimizer;
  perching_init_config.guide_min_clearance = init_resources.guide_min_clearance;
  perching_init_config.guide_sparse_min_inner = init_resources.guide_sparse_min_inner;
  perching_init_config.guide_sparse_max_inner = init_resources.guide_sparse_max_inner;
  perching_init_config.guide_turn_angle_deg = init_resources.guide_turn_angle_deg;
  perching_init_config.sfc_progress = init_resources.sfc_progress;
  perching_init_config.sfc_range = init_resources.sfc_range;

  frontend::PerchingInitArtifact perching_init;
  if (!frontend::PerchingInitService{}.initialize(perching_init_config, problem, perching_init))
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled perching initialization failed: " + perching_init.message;
    ROS_WARN("[CompiledPerchingInit] active_mode=%s result=failed reason=%s",
             compiled_active_mode,
             perching_init.message.c_str());
    return false;
  }

  if (problem.task_definition.runtime_policy.use_fast_perching_init)
  {
    frontend::InitArtifact fast_init;
    if (buildFastPerchingInitArtifact(perching_init_config,
                                      problem,
                                      perching_init,
                                      fast_init))
    {
      perching_init.transit_init = fast_init;
      perching_init.selected_mode = core::ActiveSpaceModel::PLAIN;
      perching_init.message = "fast perching direct BVP init ready";
      ROS_INFO("[CompiledPerchingInit] use_fast_perching_init=yes source=%s force_selected_mode=PLAIN",
               fast_init.source.c_str());
    }
    else
    {
      ROS_WARN("[CompiledPerchingInit] use_fast_perching_init requested but BVP seed failed; fallback to %s",
               perching_init.transit_init.source.c_str());
    }
  }

  const frontend::InitArtifact &init_artifact = perching_init.transit_init;
  populateInitArtifacts(init_artifact, solution);
  const frontend::InitArtifact &solver_input = init_artifact;
  solution.active_space_model = perching_init.selected_mode;

  const bool have_phase0_goal =
      problem.phase_specs.size() > 0 &&
      problem.phase_specs.front().terminal_is_set &&
      problem.phase_specs.front().goal_state.valid;
  const bool have_phase1_manifold =
      problem.phase_specs.size() > 1 &&
      problem.phase_specs[1].terminal_is_manifold &&
      problem.phase_specs[1].manifold_params.size() > 0;
  ROS_INFO("[CompiledPerchingPhase] phase0_goal_source=%s phase1_manifold_source=%s approach_anchor_source=%s final_mapping_source=%s",
           have_phase0_goal ? "phase_ir_phase0" : "fallback",
           have_phase1_manifold ? "phase_ir_phase1" : "fallback",
           perching_init.approach_anchor_source.c_str(),
           perching_init.final_manifold_source.c_str());

  const MINCOBoundaryState3D &headState = solver_input.head_state;
  MINCOBoundaryState3D tailState = solver_input.tail_state;
  const Eigen::MatrixXd &innerPts = solver_input.inner_points;
  const Eigen::VectorXd &durations = solver_input.durations;
  const MINCOTraj3D &initTraj = solver_input.init_traj;
  const spatial_map::PolyhedraH &corridor_hpolys = solver_input.corridor_hpolys;
  const Eigen::VectorXi &corridor_piece_idx = solver_input.corridor_piece_idx;
  const std::vector<Eigen::Vector3d> &active_guide_path = solver_input.guide_path;
  const std::vector<Eigen::Vector3d> &display_path =
      solver_input.dense_path.empty() ? solver_input.guide_path : solver_input.dense_path;
  const char *selected_mode_str = activeSpaceModelString(perching_init.selected_mode);

  if (!solver_input.hasValidTiming() || !solver_input.hasValidPieceLayout())
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled perching initialization returned invalid timing/layout";
    return false;
  }

  minco::PerchingTerminalMapping<3, ego_planner::SNAP_TRAJ_S> perching_mapping;
  const auto &decoded = perching_init.decoded_contact_semantics;
  const auto &predicted_contact = perching_init.predicted_contact_state;
  const auto &anchor = perching_init.pre_contact_anchor_state;
  // Phase-aware single perching solve:
  //   phase 0 -> approach_anchor drives transit/perching init
  //   phase 1 -> contact_final manifold drives terminal mapping
  // This PR keeps a single optimize call, but makes the two semantic roles
  // explicit in the initialization and final-terminal configuration below.
  typename minco::PerchingTerminalMapping<3, ego_planner::SNAP_TRAJ_S>::PerchingSemanticConfig
      perching_semantics;
  perching_semantics.plate_position = decoded.plate_position_ref;
  perching_semantics.plate_velocity = decoded.plate_velocity;
  perching_semantics.reference_time = decoded.reference_time;
  perching_semantics.surface_x = decoded.surface_x;
  perching_semantics.surface_y = decoded.surface_y;
  perching_semantics.surface_z = decoded.surface_z;
  perching_semantics.robot_l = decoded.robot_l;
  perching_semantics.v_plus = decoded.v_plus;
  perching_semantics.thrust_nominal = decoded.thrust_nominal;
  perching_semantics.thrust_range = decoded.thrust_range;
  perching_semantics.use_dynamics_terminal_accel = decoded.use_dynamics_terminal_accel;
  perching_semantics.nu_seed = decoded.nu_seed;
  perching_semantics.tau_f_seed = decoded.tau_f_seed;
  perching_semantics.pre_contact_distance = anchor.pre_contact_distance;
  // Fast-Perching uses a very strong rhoVt regularizer so the optimizer lands
  // with almost no tangential slip. Matching that scale here reduces the
  // "touch then slide away" behavior in the integrated planner.
  perching_semantics.weight_nu = 1.0e5;
  perching_semantics.weight_tau_f = decoded.use_dynamics_terminal_accel ? 1.0e-1 : 0.0;
  perching_mapping.configure(perching_semantics);
  ROS_INFO("[CompiledPerchingInit] active_mode=%s selected_mode=%s init_source=%s guide_pts=%zu corridor_polys=%zu",
           compiled_active_mode,
           selected_mode_str,
           solver_input.source.c_str(),
           active_guide_path.size(),
           corridor_hpolys.size());
  ROS_INFO("[CompiledPerchingInit] predicted_contact pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] pred_t=%.2f plate_ref=[%.2f %.2f %.2f] plate_vel=[%.2f %.2f %.2f] normal=[%.2f %.2f %.2f]",
           predicted_contact.contact_position.x(),
           predicted_contact.contact_position.y(),
           predicted_contact.contact_position.z(),
           predicted_contact.contact_velocity.x(),
           predicted_contact.contact_velocity.y(),
           predicted_contact.contact_velocity.z(),
           predicted_contact.contact_acceleration.x(),
           predicted_contact.contact_acceleration.y(),
           predicted_contact.contact_acceleration.z(),
           predicted_contact.prediction_time,
           decoded.plate_position_ref.x(),
           decoded.plate_position_ref.y(),
           decoded.plate_position_ref.z(),
           decoded.plate_velocity.x(),
           decoded.plate_velocity.y(),
           decoded.plate_velocity.z(),
           decoded.surface_z.x(),
           decoded.surface_z.y(),
           decoded.surface_z.z());
  ROS_INFO("[CompiledPerchingInit] phase0_approach_source=%s phase1_manifold_source=%s",
           perching_init.approach_anchor_source.c_str(),
           perching_init.final_manifold_source.c_str());
  ROS_INFO("[CompiledPerchingInit] pre_contact_anchor pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] distance=%.2f",
           anchor.position.x(),
           anchor.position.y(),
           anchor.position.z(),
           anchor.velocity.x(),
           anchor.velocity.y(),
           anchor.velocity.z(),
           anchor.acceleration.x(),
           anchor.acceleration.y(),
           anchor.acceleration.z(),
           anchor.pre_contact_distance);

  const Eigen::Vector3d optimizer_init_tail_acceleration =
      decoded.use_dynamics_terminal_accel
          ? Eigen::Vector3d::Zero()
          : predicted_contact.contact_acceleration;
  tailState = makeBoundaryState(predicted_contact.contact_position,
                                predicted_contact.contact_velocity,
                                optimizer_init_tail_acceleration);
  ROS_INFO("[CompiledPerchingInit] optimizer_nominal_contact_tail pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] source=%s",
           tailState.col(0).x(),
           tailState.col(0).y(),
           tailState.col(0).z(),
           tailState.col(1).x(),
           tailState.col(1).y(),
           tailState.col(1).z(),
           tailState.col(2).x(),
           tailState.col(2).y(),
           tailState.col(2).z(),
           decoded.use_dynamics_terminal_accel ? "phase0_soft_init_tail" : "predicted_contact");
  if (perchingOnlyDebugEnabled())
  {
    ROS_INFO("[PerchingOnlyDebug][Engine] compiled_mode=%s selected_mode=%s phase0_source=%s phase1_source=%s init_source=%s guide_preserved=%s feasible_preserved=%s seed_preserved=%s guide_pts=%zu->%zu feasible_sets=%zu->%zu seed_pts=%zu->%zu",
             compiled_active_mode,
             selected_mode_str,
             perching_init.approach_anchor_source.c_str(),
             perching_init.final_manifold_source.c_str(),
             solver_input.source.c_str(),
             perching_init.handoff_debug.guide_path_preserved ? "yes" : "no",
             perching_init.handoff_debug.feasible_sets_preserved ? "yes" : "no",
             perching_init.handoff_debug.seed_hint_preserved ? "yes" : "no",
             perching_init.handoff_debug.guide_points_before,
             perching_init.handoff_debug.guide_points_after,
             perching_init.handoff_debug.feasible_sets_before,
             perching_init.handoff_debug.feasible_sets_after,
             perching_init.handoff_debug.seed_anchor_points_before,
             perching_init.handoff_debug.seed_anchor_points_after);
    ROS_INFO("[PerchingOnlyDebug][Engine] solve_input head=[%.2f %.2f %.2f] anchor=[%.2f %.2f %.2f] contact=[%.2f %.2f %.2f] plate_ref=[%.2f %.2f %.2f] plate_vel=[%.2f %.2f %.2f] nu_seed=[%.2f %.2f] tau_f=%.2f",
             headState.col(0).x(),
             headState.col(0).y(),
             headState.col(0).z(),
             anchor.position.x(),
             anchor.position.y(),
             anchor.position.z(),
             predicted_contact.contact_position.x(),
             predicted_contact.contact_position.y(),
             predicted_contact.contact_position.z(),
             decoded.plate_position_ref.x(),
             decoded.plate_position_ref.y(),
             decoded.plate_position_ref.z(),
             decoded.plate_velocity.x(),
             decoded.plate_velocity.y(),
             decoded.plate_velocity.z(),
             decoded.nu_seed.x(),
             decoded.nu_seed.y(),
             decoded.tau_f_seed);
  }

  optimizer->setIfTouchGoal(true);

  Eigen::MatrixXd cstr_pts =
      initTraj.getInitConstraintPoints(optimizer->get_cps_num_prePiece_());
  std::vector<std::pair<int, int>> segments;
  const bool use_corridor = perching_init.selected_mode == core::ActiveSpaceModel::CORRIDOR;
  const bool use_esdf = perching_init.selected_mode == core::ActiveSpaceModel::ESDF;
  if (!use_corridor && !use_esdf)
  {
    if (optimizer->finelyCheckAndSetConstraintPoints(segments, initTraj, cstr_pts, true) ==
        PolyTrajOptimizer::CHK_RET::ERR)
    {
      solution.success = false;
      solution.used_legacy_adapter = false;
      solution.message = "compiled perching seed failed initial collision checking";
      return false;
    }
  }

  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(static_cast<std::size_t>(cstr_pts.cols()));
  for (int i = 0; i < cstr_pts.cols(); ++i)
  {
    point_set.push_back(cstr_pts.col(i));
  }
  if (visualization)
  {
    if (!display_path.empty())
    {
      visualization->displayGlobalPathList(display_path, 0.08, 2);
    }
    if (!active_guide_path.empty())
    {
      visualization->displayFrontendList(active_guide_path, 0.10, 2);
    }
    if (use_corridor && !corridor_hpolys.empty())
    {
      std::vector<Eigen::Vector3d> tri, edges;
      buildCorridorVisualization(corridor_hpolys, tri, edges);
      visualization->displayCorridor(tri, edges, 2);
    }
    visualization->displayInitPathList(point_set, 0.2, 2);
  }

  double final_cost = 0.0;
  bool flag_success = false;
  if (use_corridor)
  {
    flag_success = optimizer->optimizePerchingTrajectory(headState,
                                                         tailState,
                                                         innerPts,
                                                         durations,
                                                         corridor_hpolys,
                                                         &corridor_piece_idx,
                                                         perching_mapping,
                                                         final_cost);
  }
  else if (use_esdf)
  {
    flag_success = optimizer->optimizePerchingTrajectoryWithDistanceField(headState,
                                                                          tailState,
                                                                          innerPts,
                                                                          durations,
                                                                          perching_mapping,
                                                                          final_cost);
  }
  else
  {
    flag_success = optimizer->optimizePerchingTrajectory(headState,
                                                         tailState,
                                                         innerPts,
                                                         durations,
                                                         perching_mapping,
                                                         final_cost);
  }

  solution.success = flag_success;
  solution.used_legacy_adapter = false;
  solution.touch_goal = true;
  solution.message = flag_success
                         ? std::string("compiled perching solve success; selected_mode=") + selected_mode_str
                         : std::string("compiled perching solve failed after optimizer; selected_mode=") + selected_mode_str;

  if (!flag_success)
  {
    return false;
  }

  const SnapTraj3D opt_traj = optimizer->getSnapTrajectory();
  const double total_T = opt_traj.getTotalDuration();
  const Eigen::Vector3d final_pos = opt_traj.evaluate(total_T, 0);
  const Eigen::Vector3d final_vel = opt_traj.evaluate(total_T, 1);
  const Eigen::Vector3d final_acc = opt_traj.evaluate(total_T, 2);
  const Eigen::Vector3d predicted_plate_at_touch =
      decoded.plate_position_ref +
      decoded.plate_velocity * (total_T - decoded.reference_time);
  ROS_INFO("[CompiledPerching] solved T=%.2f final_pos=[%.2f %.2f %.2f] final_vel=[%.2f %.2f %.2f] final_acc=[%.2f %.2f %.2f] predicted_plate=[%.2f %.2f %.2f] ref_t=%.2f",
           total_T,
           final_pos.x(),
           final_pos.y(),
           final_pos.z(),
           final_vel.x(),
           final_vel.y(),
           final_vel.z(),
           final_acc.x(),
           final_acc.y(),
           final_acc.z(),
           predicted_plate_at_touch.x(),
           predicted_plate_at_touch.y(),
           predicted_plate_at_touch.z(),
           decoded.reference_time);

  const double yaw0 = [&]() -> double
  {
    const auto &prev_local = planner_manager_->traj_.local_traj;
    if (prev_local.has_yaw_ref)
    {
      const double t_local_now =
          std::max(0.0, ros::Time::now().toSec() - prev_local.start_time);
      return prev_local.sampleYaw(t_local_now);
    }
    if (problem.start_boundary.velocity.head<2>().norm() > 1.0e-3)
    {
      return std::atan2(problem.start_boundary.velocity.y(),
                       problem.start_boundary.velocity.x());
    }
    const Eigen::Vector3d rel = decoded.plate_position_ref - problem.start_boundary.position;
    if (rel.head<2>().norm() > 1.0e-3)
    {
      return std::atan2(rel.y(), rel.x());
    }
    return 0.0;
  }();

  planner_manager_->setLocalTrajFromOpt(opt_traj, true);
  double yaw_cost = 0.0;
  if (optimizer->optimizePerchingYawProjectionTrajectory(
          opt_traj,
          perching_semantics,
          yaw0,
          yaw_cost))
  {
    YawTraj1D yaw_traj;
    if (optimizer->getPerchingYawTrajectory(yaw_traj))
    {
      std::vector<double> yaw_time;
      std::vector<double> yaw_ref;
      sampleYawTrajectoryForReference(yaw_traj, 0.05, yaw_time, yaw_ref);
      planner_manager_->traj_.setLocalYawTraj(yaw_traj);
      planner_manager_->traj_.setLocalYawRef(yaw_time, yaw_ref);
      ROS_INFO("[CompiledPerching] yaw_projection_ref samples=%zu cost=%.3f",
               yaw_ref.size(),
               yaw_cost);
    }
  }
  else
  {
    ROS_WARN("[CompiledPerching] yaw projection optimization failed; local yaw_ref is not updated.");
  }
  planner_manager_->clearActiveTrackingArtifacts();
  cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
  if (visualization)
  {
    visualization->displayOptimalList(cstr_pts, 2);
  }
  solution.has_snap_trajectory = true;
  solution.snap_trajectory = opt_traj;
  solution.has_yaw_ref = planner_manager_->traj_.local_traj.has_yaw_ref;
  solution.yaw_time = planner_manager_->traj_.local_traj.yaw_time;
  solution.yaw_ref = planner_manager_->traj_.local_traj.yaw_ref;
  return true;
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

  auto *optimizer = planner_manager_->getOptimizer();
  const auto visualization = planner_manager_->getVisualization();

  if (problem.representation != core::RepresentationKind::MINCO ||
      !problem.start_boundary.valid ||
      !problem.terminal_boundary.valid)
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking problem has invalid MINCO boundaries";
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
    return false;
  }

  const core::TrackingSemanticArtifact *tracking_semantics =
      (problem.task_semantics.task_type == core::TaskType::TRACKING &&
       problem.task_semantics.tracking.consistent())
          ? &problem.task_semantics.tracking
          : (problem.references.has_tracking_semantic_artifact
                 ? &problem.references.tracking_semantic_artifact
                 : nullptr);
  cost_functional::TrackingSemanticGuide tracking_semantic_guide;
  const cost_functional::TrackingSemanticGuide *tracking_semantic_guide_ptr =
      (tracking_semantics != nullptr && tracking_semantics->buildSemanticGuide(tracking_semantic_guide))
          ? &tracking_semantic_guide
          : nullptr;

  const char *active_mode = activeSpaceModelString(problem.active_space_model);
  ROS_INFO("[CompiledTracking] active_mode=%s target_ref=%zu view_ref=%zu compiler_hint_guide=%zu semantic_guide=%s backend_semantic_guide=%s viewpoint_hint=%s visibility_hint=%s seed_kind=%s",
           active_mode,
           tracking_ref.t_ref.size(),
           tracking_ref.t_view_ref.size(),
           problem.references.guide_path.size(),
           (tracking_semantics != nullptr && tracking_semantics->hasSemanticGuidePath()) ? "yes" : "no",
           tracking_semantic_guide_ptr != nullptr ? "yes" : "no",
           (tracking_semantics != nullptr && tracking_semantics->hasViewpointHints()) ? "yes" : "no",
           (tracking_semantics != nullptr && tracking_semantics->hasVisibilityHints()) ? "yes" : "no",
           seedKindString(problem.seed.kind));

  state_to_state_initializer_.setResources(planner_manager_->makeStateToStateInitResources());
  solver::StateToStateInitializationResult init_result;
  if (!state_to_state_initializer_.initialize(problem, init_result))
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking initialization failed: " + init_result.failure_reason;
    return false;
  }
  const frontend::InitArtifact &init_artifact = init_result.init_artifact;
  populateInitArtifacts(init_artifact, solution);
  ego_planner::optimization::BackendPluginInput plugin_input =
      makeBackendPluginInput(problem, init_artifact, &tracking_ref, tracking_semantic_guide_ptr);
  const frontend::InitArtifact &solver_input = *plugin_input.transit_init;
  solution.active_space_model = init_result.selected_mode;
  if (tracking_semantics != nullptr)
  {
    solution.has_tracking_semantic_artifact = tracking_semantics->consistent();
    solution.tracking_semantic_artifact = *tracking_semantics;
  }

  // Tracking keeps its task-specific reference and tracking optimizer, but its
  // plain/ESDF/corridor initialization artifacts now come from the same
  // state-to-state frontend path used by transit and perching.
  ROS_INFO("[CompiledTrackingInit] active_mode=%s init_source=%s anchor_terminal=%s objective(distance=%s view=%s visibility=%s yaw=%s) final_guide_pts=%zu final_corridor_polys=%zu final_init_pieces=%ld",
           active_mode,
           solver_input.source.c_str(),
           (tracking_semantics != nullptr && tracking_semantics->anchor_terminal_state.valid) ? "yes" : "no",
           (tracking_semantics != nullptr && tracking_semantics->objective_metadata.enable_distance) ? "on" : "off",
           (tracking_semantics != nullptr && tracking_semantics->objective_metadata.enable_view) ? "on" : "off",
           (tracking_semantics != nullptr && tracking_semantics->objective_metadata.enable_visibility) ? "on" : "off",
           (tracking_semantics != nullptr && tracking_semantics->objective_metadata.enable_yaw) ? "on" : "off",
           solver_input.guide_path.size(),
           solver_input.corridor_hpolys.size(),
           static_cast<long>(solver_input.durations.size()));

  if (!solver_input.hasValidTiming())
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking initialization returned invalid durations";
    return false;
  }
  if (!solver_input.hasValidPieceLayout())
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking initialization returned invalid inner-point layout";
    return false;
  }
  if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR &&
      (!solver_input.hasValidCorridorAllocation()))
  {
    solution.success = false;
    solution.used_legacy_adapter = false;
    solution.message = "compiled tracking corridor initialization returned invalid corridor allocation";
    return false;
  }

  optimizer->setIfTouchGoal(plugin_input.task_semantics != nullptr
                                ? plugin_input.task_semantics->transit.touch_goal
                                : false);

  if (visualization)
  {
    const std::vector<Eigen::Vector3d> &display_path =
        solver_input.dense_path.empty() ? solver_input.guide_path : solver_input.dense_path;
    if (!display_path.empty())
    {
      visualization->displayGlobalPathList(display_path, 0.08, 3);
    }
    if (!solver_input.guide_path.empty())
    {
      visualization->displayFrontendList(solver_input.guide_path, 0.10, 3);
    }
    if (!solver_input.corridor_hpolys.empty())
    {
      std::vector<Eigen::Vector3d> tri, edges;
      buildCorridorVisualization(solver_input.corridor_hpolys, tri, edges);
      visualization->displayCorridor(tri, edges, 3);
    }
    const Eigen::MatrixXd init_display =
        solver_input.init_traj.getInitConstraintPoints(optimizer->get_cps_num_prePiece_());
    std::vector<Eigen::Vector3d> init_pts;
    init_pts.reserve(static_cast<std::size_t>(std::max<Eigen::Index>(0, init_display.cols())));
    for (int i = 0; i < init_display.cols(); ++i)
    {
      init_pts.push_back(init_display.col(i));
    }
    if (!init_pts.empty())
    {
      visualization->displayInitPathList(init_pts, 0.16, 3);
    }
  }

  bool ok = false;
  double final_cost = 0.0;
  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    if (solver_input.corridor_hpolys.empty())
    {
      solution.success = false;
      solution.used_legacy_adapter = false;
      solution.message = "compiled tracking corridor mode missing corridor geometry";
      return false;
    }
    if (tracking_semantic_guide_ptr != nullptr &&
        !tracking_semantic_guide_ptr->visible_regions.empty())
    {
      ok = optimizer->optimizeTrackingTrajectoryWithVisibleRegions(
          solver_input.head_state,
          solver_input.tail_state,
          solver_input.inner_points,
          solver_input.durations,
          solver_input.corridor_hpolys,
          &solver_input.corridor_piece_idx,
          tracking_ref,
          *tracking_semantic_guide_ptr,
          final_cost);
    }
    else
    {
      ok = optimizer->optimizeTrackingTrajectory(
          solver_input.head_state,
          solver_input.tail_state,
          solver_input.inner_points,
          solver_input.durations,
          solver_input.corridor_hpolys,
          &solver_input.corridor_piece_idx,
          tracking_ref,
          final_cost);
    }
    break;
  case core::ActiveSpaceModel::ESDF:
    ok = optimizer->optimizeTrackingTrajectoryWithDistanceField(
        solver_input.head_state,
        solver_input.tail_state,
        solver_input.inner_points,
        solver_input.durations,
        tracking_ref,
        plugin_input.tracking_semantic_guide,
        final_cost);
    break;
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  case core::ActiveSpaceModel::PLAIN:
  default:
    ok = optimizer->optimizeTrackingTrajectory(
        solver_input.head_state,
        solver_input.tail_state,
        solver_input.inner_points,
        solver_input.durations,
        tracking_ref,
        plugin_input.tracking_semantic_guide,
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

  const MINCOTraj3D opt_traj = optimizer->getTrajectory();
  planner_manager_->setLocalTrajFromOpt(opt_traj, false);
  if (tracking_semantic_guide_ptr != nullptr)
  {
    const spatial_map::PolyhedraH active_tracking_corridor =
        (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
            ? solver_input.corridor_hpolys
            : spatial_map::PolyhedraH{};
    planner_manager_->setActiveTrackingArtifacts(*tracking_semantic_guide_ptr,
                                                 active_tracking_corridor);
    solution.has_tracking_semantic_guide = true;
    solution.tracking_semantic_guide = *tracking_semantic_guide_ptr;
  }
  else
  {
    planner_manager_->clearActiveTrackingArtifacts();
  }

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

  if (visualization)
  {
    const Eigen::MatrixXd opt_display = sampleTrajectoryForDisplay(opt_traj, 0.02);
    visualization->displayOptimalList(opt_display, 3);
    // if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
    // {
    //   planner_manager_->visualization_->displayCorridor();
    // }
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
  return solveTrackingCompiledProblem(problem, solution);
}

bool PlannerEngine::solvePerchingLegacy(const core::PlanningProblem &problem,
                                        core::PlanningSolution &solution)
{
  return solvePerchingCompiledProblem(problem, solution);
}

} // namespace ego_planner::engine
