#include <runtime/task_executor.hpp>

#include <plan_manage/planner_manager.h>

namespace ego_planner::runtime
{

bool TaskExecutor::execute(const core::PlanningContext &context,
                           const core::TaskSpec &task,
                           core::PlanningSolution &solution)
{
  if (planner_manager_ == nullptr)
  {
    solution.success = false;
    solution.message = "null planner manager";
    return false;
  }
  return planner_manager_->solveTask(context, task, solution);
}

} // namespace ego_planner::runtime

