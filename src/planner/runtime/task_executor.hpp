#ifndef PLANNER_RUNTIME_TASK_EXECUTOR_HPP_
#define PLANNER_RUNTIME_TASK_EXECUTOR_HPP_

#include <core/planning_context.hpp>
#include <core/planning_solution.hpp>
#include <core/task_definition.hpp>

namespace ego_planner
{
namespace engine
{
class PlannerEngine;
}
}

namespace ego_planner::runtime
{

class TaskExecutor
{
public:
  // Runtime owns execution timing and delegates task solving to PlannerEngine.
  explicit TaskExecutor(engine::PlannerEngine *planner_engine)
      : planner_engine_(planner_engine) {}

  bool execute(const core::PlanningContext &context,
               const core::TaskDefinition &task_definition,
               core::PlanningSolution &solution);

private:
  engine::PlannerEngine *planner_engine_{nullptr};
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_TASK_EXECUTOR_HPP_
