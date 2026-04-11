#include <compiler/reference_builder.hpp>

namespace ego_planner::compiler
{

bool ReferenceBuilder::build(const core::PlanningContext &context,
                             const core::TaskDefinition &task_definition,
                             core::PlanningProblem &problem) const
{
  (void)context;
  problem.representation = core::RepresentationKind::MINCO;
  problem.references = core::CompiledReferenceSpec{};

  problem.start_boundary.valid = task_definition.start_state.valid;
  problem.start_boundary.position = task_definition.start_state.position;
  problem.start_boundary.velocity = task_definition.start_state.velocity;
  problem.start_boundary.acceleration = task_definition.start_state.acceleration;

  if (task_definition.type == core::TaskType::TRACKING &&
      task_definition.tracking_semantics.consistent())
  {
    problem.references.has_tracking_semantic_artifact = true;
    problem.references.tracking_semantic_artifact = task_definition.tracking_semantics;
    problem.terminal_boundary.valid = task_definition.tracking_semantics.anchor_terminal_state.valid;
    problem.terminal_boundary.position = task_definition.tracking_semantics.anchor_terminal_state.position;
    problem.terminal_boundary.velocity = task_definition.tracking_semantics.anchor_terminal_state.velocity;
    problem.terminal_boundary.acceleration = task_definition.tracking_semantics.anchor_terminal_state.acceleration;
    if (problem.references.guide_path.empty())
    {
      problem.references.guide_path = task_definition.tracking_semantics.semantic_guide_path;
      problem.references.guide_times = task_definition.tracking_semantics.semantic_guide_times;
    }
  }
  else
  {
    problem.terminal_boundary.valid = task_definition.goal.state.valid;
    problem.terminal_boundary.position = task_definition.goal.state.position;
    problem.terminal_boundary.velocity = task_definition.goal.state.velocity;
    problem.terminal_boundary.acceleration = task_definition.goal.state.acceleration;
  }

  for (const auto &reference : task_definition.references)
  {
    if (!reference.active || !reference.valid())
    {
      continue;
    }

    if ((reference.semantic == core::ReferenceSemanticType::GUIDE_PATH ||
         reference.semantic == core::ReferenceSemanticType::WAYPOINT_SEQUENCE) &&
        problem.references.guide_path.empty())
    {
      problem.references.guide_path = reference.points;
      problem.references.guide_times = reference.times;
    }
    else if (reference.semantic == core::ReferenceSemanticType::TRACKING_TRAJECTORY &&
             reference.tracking_reference.valid())
    {
      problem.references.has_tracking_reference = true;
      problem.references.tracking_reference = reference.tracking_reference;
      problem.references.t_ref = reference.tracking_reference.t_ref;
      problem.references.p_ref = reference.tracking_reference.p_ref;
      problem.references.v_ref = reference.tracking_reference.v_ref;

      if (problem.references.guide_path.empty() && reference.tracking_reference.viewValid())
      {
        problem.references.guide_path = reference.tracking_reference.p_view_ref;
        problem.references.guide_times = reference.tracking_reference.t_view_ref;
      }

      Eigen::Vector3d p_term = Eigen::Vector3d::Zero();
      Eigen::Vector3d v_term = Eigen::Vector3d::Zero();
      if (cost_functional::sampleTrackingTerminalReference(reference.tracking_reference, p_term, v_term))
      {
        problem.terminal_boundary.valid = true;
        problem.terminal_boundary.position = p_term;
        problem.terminal_boundary.velocity = v_term;
      }
    }
  }

  if (problem.references.has_tracking_semantic_artifact)
  {
    const auto &tracking_semantics = problem.references.tracking_semantic_artifact;
    if (tracking_semantics.hasSemanticGuidePath())
    {
      problem.references.guide_path = tracking_semantics.semantic_guide_path;
      problem.references.guide_times = tracking_semantics.semantic_guide_times;
    }
    if (tracking_semantics.anchor_terminal_state.valid)
    {
      problem.terminal_boundary.valid = true;
      problem.terminal_boundary.position = tracking_semantics.anchor_terminal_state.position;
      problem.terminal_boundary.velocity = tracking_semantics.anchor_terminal_state.velocity;
      problem.terminal_boundary.acceleration = tracking_semantics.anchor_terminal_state.acceleration;
    }
  }

  problem.phase_specs.clear();
  if (task_definition.phases.empty())
  {
    core::PhaseProblemSpec phase_problem;
    phase_problem.name = task_definition.task_name.empty() ? "phase" : task_definition.task_name;
    phase_problem.terminal_is_set = task_definition.goal.isTerminalSet();
    phase_problem.terminal_is_manifold = task_definition.goal.isTerminalManifold();
    problem.phase_specs.push_back(phase_problem);
    return true;
  }

  problem.phase_specs.reserve(task_definition.phases.size());
  for (const auto &phase : task_definition.phases)
  {
    core::PhaseProblemSpec phase_problem;
    phase_problem.name = phase.name;
    phase_problem.terminal_is_set = phase.goal.isTerminalSet();
    phase_problem.terminal_is_manifold = phase.goal.isTerminalManifold();
    phase_problem.objective_mask = phase.objective_mask;
    phase_problem.constraint_mask = phase.constraint_mask;
    problem.phase_specs.push_back(phase_problem);
  }
  return true;
}

} // namespace ego_planner::compiler
