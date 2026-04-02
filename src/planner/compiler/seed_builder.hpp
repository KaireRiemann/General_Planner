#ifndef PLANNER_COMPILER_SEED_BUILDER_HPP_
#define PLANNER_COMPILER_SEED_BUILDER_HPP_

#include <core/planning_problem.hpp>
#include <frontend/warm_start_service.hpp>

namespace ego_planner::compiler
{

class SeedBuilder
{
public:
  bool build(const core::PlanningContext &context,
             const core::TaskSpec &task,
             core::PlanningProblem &problem) const;

private:
  frontend::WarmStartService warm_start_service_;
};

} // namespace ego_planner::compiler

#endif // PLANNER_COMPILER_SEED_BUILDER_HPP_

