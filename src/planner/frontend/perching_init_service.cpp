#include <frontend/perching_init_service.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include <ros/ros.h>

namespace ego_planner::frontend
{

namespace
{

constexpr double kDefaultPreContactDistance = 0.4;
constexpr double kGravity = 9.81;
constexpr double kDefaultApproachVelocityAlpha = 0.35;

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

const char *activeSpaceModelString(const core::ActiveSpaceModel mode)
{
  switch (mode)
  {
  case core::ActiveSpaceModel::ESDF:
    return "ESDF";
  case core::ActiveSpaceModel::CORRIDOR:
    return "CORRIDOR";
  case core::ActiveSpaceModel::VISIBLE_REGION:
    return "VISIBLE_REGION";
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
    return "TERMINAL_MANIFOLD";
  case core::ActiveSpaceModel::PLAIN:
  default:
    return "PLAIN";
  }
}

const core::PhaseProblemSpec *findPhase0ApproachGoal(const core::PlanningProblem &problem)
{
  if (problem.phase_specs.empty())
  {
    return nullptr;
  }

  const core::PhaseProblemSpec &phase0 = problem.phase_specs.front();
  if (!phase0.terminal_is_set || !phase0.goal_state.valid)
  {
    return nullptr;
  }

  return &phase0;
}

const core::PhaseProblemSpec *findPhase1ContactManifold(const core::PlanningProblem &problem)
{
  if (problem.phase_specs.size() > 1)
  {
    const core::PhaseProblemSpec &phase1 = problem.phase_specs[1];
    if (phase1.terminal_is_manifold && phase1.manifold_params.size() > 0)
    {
      return &phase1;
    }
  }

  for (const auto &phase : problem.phase_specs)
  {
    if (phase.terminal_is_manifold && phase.manifold_params.size() > 0)
    {
      return &phase;
    }
  }

  return nullptr;
}

bool isFiniteStateDefinition(const core::StateDefinition &state)
{
  return state.valid &&
         state.position.allFinite() &&
         state.velocity.allFinite() &&
         state.acceleration.allFinite();
}

Eigen::Vector3d normalizedOrFallback(const Eigen::Vector3d &vec,
                                     const Eigen::Vector3d &fallback)
{
  if (!vec.allFinite() || vec.norm() < 1.0e-6)
  {
    return fallback;
  }
  return vec.normalized();
}

Eigen::Vector3d softenedApproachAnchorVelocity(const Eigen::Vector3d &plate_velocity,
                                               const double v_plus,
                                               const Eigen::Vector3d &surface_normal,
                                               const double alpha)
{
  return plate_velocity - alpha * v_plus * surface_normal;
}

bool keepGuidePrefixTowardAnchor(const std::vector<Eigen::Vector3d> &input_path,
                                 const Eigen::Vector3d &anchor_position,
                                 std::vector<Eigen::Vector3d> &output_path)
{
  output_path.clear();
  if (input_path.empty() || !anchor_position.allFinite())
  {
    return false;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::size_t best_idx = 0;
  for (std::size_t i = 0; i < input_path.size(); ++i)
  {
    if (!input_path[i].allFinite())
    {
      continue;
    }
    const double dist = (input_path[i] - anchor_position).norm();
    if (dist < best_dist)
    {
      best_dist = dist;
      best_idx = i;
    }
  }

  if (!std::isfinite(best_dist))
  {
    return false;
  }

  output_path.reserve(best_idx + 1);
  for (std::size_t i = 0; i <= best_idx; ++i)
  {
    if (input_path[i].allFinite())
    {
      output_path.push_back(input_path[i]);
    }
  }

  if (output_path.empty())
  {
    return false;
  }

  if ((output_path.back() - anchor_position).norm() > 1.0e-3)
  {
    output_path.push_back(anchor_position);
  }
  else
  {
    output_path.back() = anchor_position;
  }
  return true;
}

std::vector<Eigen::Vector3d> filterFiniteAnchors(const std::vector<Eigen::Vector3d> &anchors)
{
  std::vector<Eigen::Vector3d> filtered;
  filtered.reserve(anchors.size());
  for (const auto &pt : anchors)
  {
    if (pt.allFinite())
    {
      filtered.push_back(pt);
    }
  }
  return filtered;
}

void orthonormalizeContactFrame(PerchingDecodedContactSemantics &semantics)
{
  semantics.surface_z = normalizedOrFallback(semantics.surface_z, Eigen::Vector3d::UnitZ());
  semantics.surface_x = normalizedOrFallback(semantics.surface_x, Eigen::Vector3d::UnitX());
  semantics.surface_y = semantics.surface_z.cross(semantics.surface_x);
  if (!semantics.surface_y.allFinite() || semantics.surface_y.norm() < 1.0e-6)
  {
    semantics.surface_y = Eigen::Vector3d::UnitY();
  }
  semantics.surface_y.normalize();
  semantics.surface_x = semantics.surface_y.cross(semantics.surface_z);
  if (!semantics.surface_x.allFinite() || semantics.surface_x.norm() < 1.0e-6)
  {
    semantics.surface_x = Eigen::Vector3d::UnitX();
  }
  semantics.surface_x.normalize();
}

core::PlanningProblem makeApproachProblem(const core::PlanningProblem &problem,
                                          const PerchingPreContactAnchorState &anchor_state,
                                          PerchingHandoffDebugSummary *handoff_debug)
{
  core::PlanningProblem approach_problem = problem;
  approach_problem.problem_name = problem.problem_name + "_perching_approach";
  // Perching should not inherit the rolling tracking local/global trajectory
  // as a warm-start. Fast-Perching replans the perching segment from the
  // current relative state instead of extending the previous tracking path.
  approach_problem.task.flag_poly_init = true;
  approach_problem.task.flag_random_poly_traj = false;
  approach_problem.task_definition.runtime_policy.flag_poly_init = true;
  approach_problem.task_definition.runtime_policy.flag_random_poly_traj = false;
  approach_problem.terminal_boundary.valid = anchor_state.valid;
  approach_problem.terminal_boundary.position = anchor_state.position;
  approach_problem.terminal_boundary.velocity = anchor_state.velocity;
  approach_problem.terminal_boundary.acceleration = anchor_state.acceleration;

  const std::size_t guide_pts_before = approach_problem.references.guide_path.size();
  const std::size_t guide_times_before = approach_problem.references.guide_times.size();
  const std::size_t feasible_before = approach_problem.feasible_sets.size();
  const bool seed_valid_before = approach_problem.seed.valid;
  const std::size_t seed_anchor_before = approach_problem.seed.anchor_points.size();
  bool guide_path_trimmed_to_anchor = false;
  bool seed_trimmed_to_anchor = false;

  if (!approach_problem.references.guide_path.empty())
  {
    std::vector<Eigen::Vector3d> filtered_guide_path;
    std::vector<double> filtered_guide_times;
    filtered_guide_path.reserve(approach_problem.references.guide_path.size());
    const bool have_paired_times =
        approach_problem.references.guide_times.size() == approach_problem.references.guide_path.size();
    if (have_paired_times)
    {
      filtered_guide_times.reserve(approach_problem.references.guide_times.size());
    }

    for (std::size_t i = 0; i < approach_problem.references.guide_path.size(); ++i)
    {
      const Eigen::Vector3d &pt = approach_problem.references.guide_path[i];
      if (!pt.allFinite())
      {
        continue;
      }
      filtered_guide_path.push_back(pt);
      if (have_paired_times)
      {
        filtered_guide_times.push_back(approach_problem.references.guide_times[i]);
      }
    }

    approach_problem.references.guide_path.swap(filtered_guide_path);
    if (have_paired_times)
    {
      bool monotonic_times = true;
      for (std::size_t i = 0; i < filtered_guide_times.size(); ++i)
      {
        if (!std::isfinite(filtered_guide_times[i]) ||
            (i > 0 && filtered_guide_times[i] + 1.0e-6 < filtered_guide_times[i - 1]))
        {
          monotonic_times = false;
          break;
        }
      }
      if (monotonic_times)
      {
        approach_problem.references.guide_times.swap(filtered_guide_times);
      }
      else
      {
        approach_problem.references.guide_times.clear();
      }
    }
    else
    {
      approach_problem.references.guide_times.clear();
    }
  }
  else if (!approach_problem.references.guide_times.empty())
  {
    approach_problem.references.guide_times.clear();
  }

  if (anchor_state.valid && !approach_problem.references.guide_path.empty())
  {
    std::vector<Eigen::Vector3d> clipped_guide_path;
    if (keepGuidePrefixTowardAnchor(approach_problem.references.guide_path,
                                    anchor_state.position,
                                    clipped_guide_path))
    {
      guide_path_trimmed_to_anchor =
          clipped_guide_path.size() < approach_problem.references.guide_path.size();
      approach_problem.references.guide_path.swap(clipped_guide_path);
      if (!approach_problem.references.guide_times.empty())
      {
        const std::size_t keep_count =
            std::min(approach_problem.references.guide_times.size(),
                     approach_problem.references.guide_path.size());
        std::vector<double> clipped_guide_times(
            approach_problem.references.guide_times.begin(),
            approach_problem.references.guide_times.begin() + keep_count);
        approach_problem.references.guide_times.swap(clipped_guide_times);
      }
    }
  }

  if (anchor_state.valid && approach_problem.seed.valid &&
      !approach_problem.seed.anchor_points.empty())
  {
    std::vector<Eigen::Vector3d> clipped_seed_points;
    if (keepGuidePrefixTowardAnchor(approach_problem.seed.anchor_points,
                                    anchor_state.position,
                                    clipped_seed_points))
    {
      seed_trimmed_to_anchor =
          clipped_seed_points.size() < approach_problem.seed.anchor_points.size();
      approach_problem.seed.anchor_points.swap(clipped_seed_points);
      approach_problem.variable_layout.piece_num =
          std::max(0, static_cast<int>(approach_problem.seed.anchor_points.size()) - 1);
      approach_problem.variable_layout.inner_point_num =
          std::max(0, static_cast<int>(approach_problem.seed.anchor_points.size()) - 2);
    }
  }

  const bool guide_path_preserved =
      guide_pts_before == 0 || !approach_problem.references.guide_path.empty();
  const bool feasible_sets_preserved =
      feasible_before == 0 || !approach_problem.feasible_sets.empty();
  const bool seed_hint_preserved =
      !seed_valid_before || approach_problem.seed.valid;
  const bool guide_times_preserved =
      guide_times_before == 0 || !approach_problem.references.guide_times.empty();

  if (handoff_debug != nullptr)
  {
    handoff_debug->guide_path_preserved = guide_path_preserved;
    handoff_debug->guide_times_preserved = guide_times_preserved;
    handoff_debug->feasible_sets_preserved = feasible_sets_preserved;
    handoff_debug->seed_hint_preserved = seed_hint_preserved;
    handoff_debug->guide_path_trimmed_to_anchor =
        (approach_problem.references.guide_path.size() < guide_pts_before) ||
        guide_path_trimmed_to_anchor;
    handoff_debug->seed_trimmed_to_anchor = seed_trimmed_to_anchor;
    handoff_debug->guide_points_before = guide_pts_before;
    handoff_debug->guide_points_after = approach_problem.references.guide_path.size();
    handoff_debug->guide_times_before = guide_times_before;
    handoff_debug->guide_times_after = approach_problem.references.guide_times.size();
    handoff_debug->feasible_sets_before = feasible_before;
    handoff_debug->feasible_sets_after = approach_problem.feasible_sets.size();
    handoff_debug->seed_valid_before = seed_valid_before;
    handoff_debug->seed_valid_after = approach_problem.seed.valid;
    handoff_debug->seed_anchor_points_before = seed_anchor_before;
    handoff_debug->seed_anchor_points_after = approach_problem.seed.anchor_points.size();
  }

  ROS_INFO("[PerchingInit] guide_path_preserved=%s guide_path_trimmed=%s guide_pts=%zu->%zu guide_times_preserved=%s guide_times=%zu->%zu",
           guide_path_preserved ? "yes" : "no",
           (approach_problem.references.guide_path.size() < guide_pts_before ||
            guide_path_trimmed_to_anchor) ? "yes" : "no",
           guide_pts_before,
           approach_problem.references.guide_path.size(),
           guide_times_preserved ? "yes" : "no",
           guide_times_before,
           approach_problem.references.guide_times.size());
  ROS_INFO("[PerchingInit] feasible_sets_preserved=%s feasible_sets=%zu->%zu",
           feasible_sets_preserved ? "yes" : "no",
           feasible_before,
           approach_problem.feasible_sets.size());
  ROS_INFO("[PerchingInit] seed_hint_preserved=%s seed_valid=%s->%s",
           seed_hint_preserved ? "yes" : "no",
           seed_valid_before ? "yes" : "no",
           approach_problem.seed.valid ? "yes" : "no");
  ROS_INFO("[PerchingInit] approach init warm_start_mode=fresh_poly_init seed_anchor_pts=%zu->%zu seed_trimmed_to_anchor=%s",
           seed_anchor_before,
           approach_problem.seed.anchor_points.size(),
           seed_trimmed_to_anchor ? "yes" : "no");

  return approach_problem;
}

} // namespace

bool PerchingInitService::decodeContactSemantics(const core::PlanningProblem &problem,
                                                 PerchingDecodedContactSemantics &semantics,
                                                 std::string *reason,
                                                 std::string *source) const
{
  semantics = PerchingDecodedContactSemantics{};
  if (source != nullptr)
  {
    *source = "unknown";
  }

  if (!problem.task_semantics.perching.valid)
  {
    if (reason != nullptr)
    {
      *reason = "missing valid perching task semantics";
    }
    return false;
  }

  const core::PhaseProblemSpec *phase1_manifold = findPhase1ContactManifold(problem);
  const Eigen::VectorXd *params_ptr =
      phase1_manifold != nullptr
          ? &phase1_manifold->manifold_params
          : &problem.task_semantics.perching.terminal_manifold_params;
  const Eigen::VectorXd &params = *params_ptr;
  if (source != nullptr)
  {
    *source = phase1_manifold != nullptr ? "phase_ir_phase1" : "task_semantics";
  }
  if (params.size() >= 29)
  {
    semantics.plate_position_ref = params.segment<3>(6);
    semantics.plate_velocity = params.segment<3>(9);
    semantics.surface_x = params.segment<3>(12);
    semantics.surface_y = params.segment<3>(15);
    semantics.surface_z = params.segment<3>(18);
    semantics.robot_l = params(21);
    semantics.v_plus = params(22);
    semantics.thrust_nominal = params(23);
    semantics.thrust_range = params(24);
    semantics.nu_seed.x() = params(25);
    semantics.nu_seed.y() = params(26);
    semantics.tau_f_seed = params(27);
    semantics.use_dynamics_terminal_accel = params(28) > 0.5;
    semantics.reference_time = params.size() >= 30 ? std::max(0.0, params(29)) : 0.0;
  }
  else if (params.size() >= 11)
  {
    semantics.surface_z = params.segment<3>(6);
    semantics.robot_l = params(9);
    semantics.v_plus = params(10);
    semantics.plate_position_ref =
        problem.task_semantics.perching.contact_state.position - semantics.robot_l * semantics.surface_z;
    semantics.plate_velocity =
        problem.task_semantics.perching.contact_state.velocity + semantics.v_plus * semantics.surface_z;
    semantics.reference_time = 0.0;
  }
  else
  {
    if (reason != nullptr)
    {
      *reason = "terminal manifold parameters are incomplete";
    }
    return false;
  }

  orthonormalizeContactFrame(semantics);
  semantics.robot_l = std::max(0.0, semantics.robot_l);
  semantics.v_plus = std::max(0.0, semantics.v_plus);
  semantics.thrust_nominal = std::max(0.0, semantics.thrust_nominal);
  semantics.thrust_range = std::max(0.0, semantics.thrust_range);
  const double compiled_approach_distance =
      std::max(0.0, problem.task_semantics.perching.approach_distance);
  semantics.pre_contact_distance =
      compiled_approach_distance > 1.0e-6
          ? compiled_approach_distance
          : std::max(kDefaultPreContactDistance, semantics.robot_l + 0.2);
  semantics.valid = semantics.plate_position_ref.allFinite() &&
                    semantics.plate_velocity.allFinite() &&
                    semantics.surface_x.allFinite() &&
                    semantics.surface_y.allFinite() &&
                    semantics.surface_z.allFinite();
  if (!semantics.valid && reason != nullptr)
  {
    *reason = "decoded perching contact semantics are not finite";
  }
  return semantics.valid;
}

bool PerchingInitService::initialize(const TransitInitRuntimeConfig &config,
                                     const core::PlanningProblem &problem,
                                     PerchingInitArtifact &artifact) const
{
  artifact = PerchingInitArtifact{};
  artifact.selected_mode = problem.active_space_model;

  std::string decode_reason;
  std::string manifold_source;
  if (!decodeContactSemantics(problem,
                              artifact.decoded_contact_semantics,
                              &decode_reason,
                              &manifold_source))
  {
    artifact.message = "failed to decode perching semantics: " + decode_reason;
    return false;
  }
  artifact.final_manifold_source = manifold_source;

  PerchingPredictedContactState &predicted = artifact.predicted_contact_state;
  const PerchingDecodedContactSemantics &semantics = artifact.decoded_contact_semantics;
  // plate_position_ref is already the plate state sampled at reference_time.
  predicted.prediction_time = std::max(0.0, semantics.reference_time);
  predicted.plate_position = semantics.plate_position_ref;
  predicted.contact_position =
      predicted.plate_position + semantics.robot_l * semantics.surface_z;
  predicted.contact_velocity =
      semantics.plate_velocity - semantics.v_plus * semantics.surface_z;
  if (semantics.use_dynamics_terminal_accel)
  {
    predicted.contact_acceleration =
        semantics.thrust_nominal * semantics.surface_z +
        Eigen::Vector3d(0.0, 0.0, -kGravity);
  }
  else
  {
    predicted.contact_acceleration = Eigen::Vector3d::Zero();
  }
  predicted.valid = predicted.plate_position.allFinite() &&
                    predicted.contact_position.allFinite() &&
                    predicted.contact_velocity.allFinite() &&
                    predicted.contact_acceleration.allFinite();
  if (!predicted.valid)
  {
    artifact.message = "predicted contact state is not finite";
    return false;
  }

  PerchingPreContactAnchorState &anchor = artifact.pre_contact_anchor_state;
  const core::PerchingSemanticArtifact &compiled_semantics = problem.task_semantics.perching;
  const core::PhaseProblemSpec *phase0_anchor = findPhase0ApproachGoal(problem);
  const core::StateDefinition &semantic_anchor =
      compiled_semantics.approach_anchor_state;
  std::string anchor_source = "decoded_contact_fallback";
  Eigen::Vector3d previous_anchor_velocity = predicted.contact_velocity;
  Eigen::Vector3d previous_anchor_acceleration = predicted.contact_acceleration;
  const Eigen::Vector3d softened_anchor_velocity =
      softenedApproachAnchorVelocity(semantics.plate_velocity,
                                     semantics.v_plus,
                                     semantics.surface_z,
                                     kDefaultApproachVelocityAlpha);
  if (phase0_anchor != nullptr &&
      phase0_anchor->goal_state.position.allFinite() &&
      phase0_anchor->goal_state.velocity.allFinite() &&
      phase0_anchor->goal_state.acceleration.allFinite())
  {
    anchor.position = phase0_anchor->goal_state.position;
    previous_anchor_velocity = phase0_anchor->goal_state.velocity;
    previous_anchor_acceleration = phase0_anchor->goal_state.acceleration;
    anchor.pre_contact_distance =
        compiled_semantics.approach_distance > 1.0e-6
            ? compiled_semantics.approach_distance
            : std::max(0.0,
                       (predicted.contact_position - anchor.position)
                           .dot(semantics.surface_z));
    anchor_source = "phase_ir_phase0";
  }
  else if (isFiniteStateDefinition(semantic_anchor))
  {
    anchor.position = semantic_anchor.position;
    previous_anchor_velocity = semantic_anchor.velocity;
    previous_anchor_acceleration = semantic_anchor.acceleration;
    anchor.pre_contact_distance =
        compiled_semantics.approach_distance > 1.0e-6
            ? compiled_semantics.approach_distance
            : std::max(0.0,
                       (predicted.contact_position - anchor.position)
                           .dot(semantics.surface_z));
    anchor_source = "task_semantics";
  }
  else
  {
    anchor.pre_contact_distance = semantics.pre_contact_distance;
    anchor.position = predicted.contact_position - anchor.pre_contact_distance * semantics.surface_z;
    ROS_WARN("[PerchingInit] approach anchor source=decoded_contact_fallback reason=invalid_phase0_and_task_semantics_anchor");
  }
  anchor.velocity = softened_anchor_velocity;
  anchor.acceleration = Eigen::Vector3d::Zero();
  anchor.valid = anchor.position.allFinite() &&
                 anchor.velocity.allFinite() &&
                 anchor.acceleration.allFinite();
  if (!anchor.valid)
  {
    artifact.message = "pre-contact anchor state is not finite";
    return false;
  }
  artifact.approach_anchor_source = anchor_source;
  ROS_INFO("[PerchingInit] phase-aware source summary: phase0_goal=%s final_manifold=%s chosen_anchor=%s",
           phase0_anchor != nullptr ? "phase_ir_phase0" : "unavailable",
           artifact.final_manifold_source.c_str(),
           artifact.approach_anchor_source.c_str());
  ROS_INFO("[PerchingInit] soften approach anchor source=%s alpha=%.2f prev_anchor_vel=[%.2f %.2f %.2f] new_anchor_vel=[%.2f %.2f %.2f] prev_anchor_acc=[%.2f %.2f %.2f] new_anchor_acc=[0.00 0.00 0.00]",
           anchor_source.c_str(),
           kDefaultApproachVelocityAlpha,
           previous_anchor_velocity.x(),
           previous_anchor_velocity.y(),
           previous_anchor_velocity.z(),
           anchor.velocity.x(),
           anchor.velocity.y(),
           anchor.velocity.z(),
           previous_anchor_acceleration.x(),
           previous_anchor_acceleration.y(),
           previous_anchor_acceleration.z());
  ROS_INFO("[PerchingInit] approach anchor source=%s pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] distance=%.2f",
           anchor_source.c_str(),
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

  const core::PlanningProblem approach_problem =
      makeApproachProblem(problem, anchor, &artifact.handoff_debug);
  TransitInitResult transit_result;
  transit_result.selected_mode = problem.active_space_model;
  TransitInitService transit_service;

  const auto tryHandoffAnchorInit =
      [&](const std::vector<Eigen::Vector3d> &raw_anchors,
          const char *source_label) -> bool
  {
    if (config.plan_params == nullptr)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> anchors = filterFiniteAnchors(raw_anchors);
    if (anchors.size() < 2)
    {
      return false;
    }

    InitArtifact handoff_init;
    if (!transit_service.buildFromAnchors(problem.start_boundary.position,
                                          problem.start_boundary.velocity,
                                          problem.start_boundary.acceleration,
                                          anchor.position,
                                          anchor.velocity,
                                          anchors,
                                          config.plan_params->polyTraj_piece_length,
                                          config.plan_params->max_vel_,
                                          handoff_init,
                                          source_label))
    {
      return false;
    }

    transit_result.selected_mode = problem.active_space_model;
    transit_result.success = handoff_init.valid;
    transit_result.init_artifact = handoff_init;
    return handoff_init.valid;
  };

  bool init_ok = false;
  const bool committed_dynamic_stage =
      artifact.decoded_contact_semantics.use_dynamics_terminal_accel &&
      !problem.task_definition.runtime_policy.enable_successor_planning;
  if (problem.active_space_model == core::ActiveSpaceModel::PLAIN ||
      problem.active_space_model == core::ActiveSpaceModel::VISIBLE_REGION ||
      problem.active_space_model == core::ActiveSpaceModel::TERMINAL_MANIFOLD)
  {
    if (committed_dynamic_stage)
    {
      ROS_INFO("[PerchingInit] skip handoff anchors for committed perching init; fallback to perching-specific local initializer.");
    }
    else if (approach_problem.seed.valid &&
             tryHandoffAnchorInit(approach_problem.seed.anchor_points, "handoff_seed"))
    {
      ROS_INFO("[PerchingInit] consumed handoff seed anchors for approach init: pts=%zu",
               approach_problem.seed.anchor_points.size());
      init_ok = true;
    }
    else if (!approach_problem.references.guide_path.empty() &&
             tryHandoffAnchorInit(approach_problem.references.guide_path, "handoff_guide"))
    {
      ROS_INFO("[PerchingInit] consumed handoff guide anchors for approach init: pts=%zu",
               approach_problem.references.guide_path.size());
      init_ok = true;
    }
  }

  if (!init_ok) switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    init_ok = transit_service.initializeCorridor(config, approach_problem, transit_result);
    break;
  case core::ActiveSpaceModel::ESDF:
    init_ok = transit_service.initializeEsdf(config, approach_problem, transit_result);
    break;
  case core::ActiveSpaceModel::PLAIN:
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  default:
    init_ok = transit_service.initializePlain(config, approach_problem, transit_result);
    break;
  }

  artifact.selected_mode = transit_result.selected_mode;
  artifact.transit_init = transit_result.init_artifact;
  if (!artifact.transit_init.source.empty())
  {
    artifact.transit_init.source = "perching_" + artifact.transit_init.source;
  }
  artifact.valid = init_ok && artifact.transit_init.valid;
  artifact.message = init_ok ? "perching init ready via " + artifact.transit_init.source
                             : transit_result.failure_reason;
  if (perchingOnlyDebugEnabled())
  {
    ROS_INFO("[PerchingOnlyDebug][Init] selected_mode=%s init_valid=%s init_source=%s phase0_source=%s phase1_source=%s guide_preserved=%s feasible_preserved=%s seed_preserved=%s guide_pts=%zu->%zu seed_pts=%zu->%zu corridor_polys=%zu",
             activeSpaceModelString(artifact.selected_mode),
             artifact.valid ? "yes" : "no",
             artifact.transit_init.source.c_str(),
             artifact.approach_anchor_source.c_str(),
             artifact.final_manifold_source.c_str(),
             artifact.handoff_debug.guide_path_preserved ? "yes" : "no",
             artifact.handoff_debug.feasible_sets_preserved ? "yes" : "no",
             artifact.handoff_debug.seed_hint_preserved ? "yes" : "no",
             artifact.handoff_debug.guide_points_before,
             artifact.handoff_debug.guide_points_after,
             artifact.handoff_debug.seed_anchor_points_before,
             artifact.handoff_debug.seed_anchor_points_after,
             artifact.transit_init.corridor_hpolys.size());
  }
  return artifact.valid;
}

} // namespace ego_planner::frontend
