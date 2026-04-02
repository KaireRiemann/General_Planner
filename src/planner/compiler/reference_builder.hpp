#ifndef PLANNER_COMPILER_REFERENCE_BUILDER_HPP_
#define PLANNER_COMPILER_REFERENCE_BUILDER_HPP_

#include <core/planning_problem.hpp>

namespace ego_planner::compiler
{

class ReferenceBuilder
{
public:
  bool build(const core::PlanningContext &context,
             const core::TaskDefinition &task_definition,
             core::PlanningProblem &problem) const;
};

} // namespace ego_planner::compiler

#endif // PLANNER_COMPILER_REFERENCE_BUILDER_HPP_
