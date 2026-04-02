#include <compiler/reference_builder.hpp>

namespace ego_planner::compiler
{

bool ReferenceBuilder::build(const core::PlanningContext &context,
                             const core::TaskSpec &task,
                             core::PlanningProblem &problem) const
{
  (void)context;
  problem.task = task;
  return true;
}

} // namespace ego_planner::compiler

