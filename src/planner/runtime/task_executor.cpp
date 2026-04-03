#include <runtime/task_executor.hpp>

#include <engine/planner_engine.hpp>

namespace ego_planner::runtime
{

bool TaskExecutor::execute(const core::PlanningContext &context,
                           const core::TaskDefinition &task_definition,
                           core::PlanningSolution &solution)
{
  if (planner_engine_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner engine";
    return false;
  }
  return planner_engine_->solveTask(context, task_definition, solution);
}

} // namespace ego_planner::runtime
