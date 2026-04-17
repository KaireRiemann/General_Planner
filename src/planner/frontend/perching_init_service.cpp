#include <frontend/perching_init_service.hpp>

#include <algorithm>

namespace ego_planner::frontend
{

namespace
{

constexpr double kDefaultPreContactDistance = 0.4;
constexpr double kGravity = 9.81;

Eigen::Vector3d normalizedOrFallback(const Eigen::Vector3d &vec,
                                     const Eigen::Vector3d &fallback)
{
  if (!vec.allFinite() || vec.norm() < 1.0e-6)
  {
    return fallback;
  }
  return vec.normalized();
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
                                          const PerchingPreContactAnchorState &anchor_state)
{
  core::PlanningProblem approach_problem = problem;
  approach_problem.problem_name = problem.problem_name + "_perching_approach";
  approach_problem.terminal_boundary.valid = anchor_state.valid;
  approach_problem.terminal_boundary.position = anchor_state.position;
  approach_problem.terminal_boundary.velocity = anchor_state.velocity;
  approach_problem.terminal_boundary.acceleration = anchor_state.acceleration;

  // The contact manifold remains perching-owned semantics. Approach transit
  // init must not treat the real contact point or any contact-derived corridor
  // hint as a generic free-space target.
  approach_problem.references.guide_path.clear();
  approach_problem.references.guide_times.clear();
  approach_problem.feasible_sets.clear();
  approach_problem.seed = core::SeedSpec{};

  return approach_problem;
}

} // namespace

bool PerchingInitService::decodeContactSemantics(const core::PlanningProblem &problem,
                                                 PerchingDecodedContactSemantics &semantics,
                                                 std::string *reason) const
{
  semantics = PerchingDecodedContactSemantics{};

  if (!problem.task_semantics.perching.valid)
  {
    if (reason != nullptr)
    {
      *reason = "missing valid perching task semantics";
    }
    return false;
  }

  const Eigen::VectorXd &params = problem.task_semantics.perching.terminal_manifold_params;
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
  if (!decodeContactSemantics(problem, artifact.decoded_contact_semantics, &decode_reason))
  {
    artifact.message = "failed to decode perching semantics: " + decode_reason;
    return false;
  }

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
  if (compiled_semantics.approach_anchor_state.valid)
  {
    anchor.position = compiled_semantics.approach_anchor_state.position;
    anchor.velocity = compiled_semantics.approach_anchor_state.velocity;
    anchor.acceleration = compiled_semantics.approach_anchor_state.acceleration;
    anchor.pre_contact_distance =
        compiled_semantics.approach_distance > 1.0e-6
            ? compiled_semantics.approach_distance
            : std::max(0.0,
                       (predicted.contact_position - anchor.position)
                           .dot(semantics.surface_z));
  }
  else
  {
    anchor.pre_contact_distance = semantics.pre_contact_distance;
    anchor.position = predicted.contact_position - anchor.pre_contact_distance * semantics.surface_z;
    anchor.velocity = predicted.contact_velocity;
    anchor.acceleration = predicted.contact_acceleration;
  }
  anchor.valid = anchor.position.allFinite() &&
                 anchor.velocity.allFinite() &&
                 anchor.acceleration.allFinite();
  if (!anchor.valid)
  {
    artifact.message = "pre-contact anchor state is not finite";
    return false;
  }

  const core::PlanningProblem approach_problem = makeApproachProblem(problem, anchor);
  TransitInitResult transit_result;
  transit_result.selected_mode = problem.active_space_model;
  TransitInitService transit_service;

  bool init_ok = false;
  switch (problem.active_space_model)
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
  return artifact.valid;
}

} // namespace ego_planner::frontend
