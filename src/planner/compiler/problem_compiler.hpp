#ifndef PLANNER_COMPILER_PROBLEM_COMPILER_HPP_
#define PLANNER_COMPILER_PROBLEM_COMPILER_HPP_

#include <compiler/feasible_set_builder.hpp>
#include <compiler/reference_builder.hpp>
#include <compiler/seed_builder.hpp>
#include <optimization/problem_adapter.hpp>
#include <core/task_definition.hpp>

namespace ego_planner::compiler
{

class ProblemCompiler
{
public:
  // ProblemCompiler is the semantic-to-optimization bridge:
  // it lowers TaskDefinition into a solver-facing PlanningProblem IR and may attach optional hints,
  // but task-stable state-to-state initialization remains downstream in StateToStateInitializer.
  void setProblemAdapter(optimization::ProblemAdapter *adapter)
  {
    adapter_ = adapter;
  }

  bool compile(const core::PlanningContext &context,
               const core::TaskDefinition &task_definition,
               core::PlanningProblem &problem) const;

  bool compile(const core::PlanningContext &context,
               const core::TaskSpec &task,
               core::PlanningProblem &problem) const;

private:
  optimization::ProblemAdapter *adapter_{nullptr};
  ReferenceBuilder reference_builder_;
  FeasibleSetBuilder feasible_set_builder_;
  SeedBuilder seed_builder_;
};

} // namespace ego_planner::compiler

#endif // PLANNER_COMPILER_PROBLEM_COMPILER_HPP_
