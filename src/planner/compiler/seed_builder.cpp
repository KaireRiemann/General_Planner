#include <compiler/seed_builder.hpp>

namespace ego_planner::compiler
{

bool SeedBuilder::build(const core::PlanningContext &context,
                        const core::TaskSpec &task,
                        core::PlanningProblem &problem) const
{
  (void)task;
  core::WarmStartCache warm_start;
  if (warm_start_service_.fetch(context, warm_start))
  {
    problem.context.warm_start = warm_start;
  }
  return true;
}

} // namespace ego_planner::compiler

