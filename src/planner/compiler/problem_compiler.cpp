#include <compiler/problem_compiler.hpp>

#include <optimization/constraint_registry.hpp>
#include <optimization/objective_registry.hpp>

namespace ego_planner::compiler
{

bool ProblemCompiler::compile(const core::PlanningContext &context,
                              const core::TaskSpec &task,
                              core::PlanningProblem &problem) const
{
  problem = core::PlanningProblem{};
  problem.problem_name = task.task_name;
  problem.context = context;
  problem.task = task;

  if (!reference_builder_.build(context, task, problem))
  {
    return false;
  }
  if (!feasible_set_builder_.build(context, task, problem))
  {
    return false;
  }
  if (!seed_builder_.build(context, task, problem))
  {
    return false;
  }

  // Default objective/constraint masks are generic and task-agnostic at solver layer.
  problem.objective_mask =
      optimization::OBJ_SMOOTHNESS |
      optimization::OBJ_FEASIBILITY |
      optimization::OBJ_OBSTACLE;
  problem.constraint_mask =
      optimization::CON_DYNAMICS |
      optimization::CON_COLLISION;

  if (adapter_ != nullptr)
  {
    problem.solve_callback =
        [adapter = adapter_](const core::PlanningProblem &p, core::PlanningSolution &s) -> bool
    {
      return adapter->solveCompatibility(p, s);
    };
  }

  problem.valid = static_cast<bool>(problem.solve_callback);
  return problem.valid;
}

} // namespace ego_planner::compiler

