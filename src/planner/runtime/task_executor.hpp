#ifndef PLANNER_RUNTIME_TASK_EXECUTOR_HPP_
#define PLANNER_RUNTIME_TASK_EXECUTOR_HPP_

#include <core/planning_context.hpp>
#include <core/planning_solution.hpp>
#include <core/task_spec.hpp>

namespace ego_planner
{
class EGOPlannerManager;
}

namespace ego_planner::runtime
{

class TaskExecutor
{
public:
  explicit TaskExecutor(EGOPlannerManager *planner_manager)
      : planner_manager_(planner_manager) {}

  bool execute(const core::PlanningContext &context,
               const core::TaskSpec &task,
               core::PlanningSolution &solution);

private:
  EGOPlannerManager *planner_manager_{nullptr};
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_TASK_EXECUTOR_HPP_

