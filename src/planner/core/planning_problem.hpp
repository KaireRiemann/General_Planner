#ifndef PLANNER_CORE_PLANNING_PROBLEM_HPP_
#define PLANNER_CORE_PLANNING_PROBLEM_HPP_

#include <functional>
#include <string>
#include <vector>

#include <core/feasible_set_spec.hpp>
#include <core/planning_context.hpp>
#include <core/task_spec.hpp>

namespace ego_planner::core
{

struct PlanningSolution;
struct PlanningProblem;

using SolveCallback = std::function<bool(const PlanningProblem &, PlanningSolution &)>;

struct PlanningProblem
{
  std::string problem_name{"planning_problem"};
  bool valid{false};

  PlanningContext context;
  TaskSpec task;

  std::vector<FeasibleSetSpec> feasible_sets;
  uint32_t objective_mask{0U};
  uint32_t constraint_mask{0U};

  SolveCallback solve_callback;
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PLANNING_PROBLEM_HPP_

