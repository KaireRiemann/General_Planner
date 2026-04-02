#include <compiler/reference_builder.hpp>

namespace ego_planner::compiler
{

bool ReferenceBuilder::build(const core::PlanningContext &context,
                             const core::TaskSpec &task,
                             core::PlanningProblem &problem) const
{
  (void)context;
  problem.task = task;
  problem.representation = core::RepresentationKind::MINCO;

  problem.start_boundary.valid = true;
  problem.start_boundary.position = task.start_pt;
  problem.start_boundary.velocity = task.start_vel;
  problem.start_boundary.acceleration = task.start_acc;

  problem.terminal_boundary.valid = true;
  problem.terminal_boundary.position = task.goal_pt;
  problem.terminal_boundary.velocity = task.goal_vel;
  problem.terminal_boundary.acceleration = Eigen::Vector3d::Zero();

  if (task.type == core::TaskType::TRACKING && task.tracking_reference.valid())
  {
    problem.references.t_ref = task.tracking_reference.t_ref;
    problem.references.p_ref = task.tracking_reference.p_ref;
    problem.references.v_ref = task.tracking_reference.v_ref;
  }

  problem.phase_specs.clear();
  for (const auto &phase : task.phases)
  {
    core::PhaseProblemSpec phase_problem;
    phase_problem.name = phase.name;
    phase_problem.terminal_is_set =
        phase.terminal_type == core::TerminalSpecType::TERMINAL_SET;
    phase_problem.terminal_is_manifold =
        phase.terminal_type == core::TerminalSpecType::TERMINAL_MANIFOLD;
    phase_problem.objective_mask = phase.active_objective_mask;
    phase_problem.constraint_mask = phase.active_constraint_mask;
    problem.phase_specs.push_back(phase_problem);
  }
  return true;
}

} // namespace ego_planner::compiler

