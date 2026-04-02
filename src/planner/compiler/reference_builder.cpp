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

  problem.terminal_boundary.valid = task_definition.goal.state.valid;
  problem.terminal_boundary.position = task_definition.goal.state.position;
  problem.terminal_boundary.velocity = task_definition.goal.state.velocity;
  problem.terminal_boundary.acceleration = task_definition.goal.state.acceleration;

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
      problem.references.t_ref = reference.tracking_reference.t_ref;
      problem.references.p_ref = reference.tracking_reference.p_ref;
      problem.references.v_ref = reference.tracking_reference.v_ref;
      if (!problem.terminal_boundary.valid && !reference.tracking_reference.p_ref.empty())
      {
        problem.terminal_boundary.valid = true;
        problem.terminal_boundary.position = reference.tracking_reference.p_ref.back();
        if (!reference.tracking_reference.v_ref.empty())
        {
          problem.terminal_boundary.velocity = reference.tracking_reference.v_ref.back();
        }
      }
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
