#ifndef PLANNER_OPTIMIZATION_PROBLEM_ADAPTER_HPP_
#define PLANNER_OPTIMIZATION_PROBLEM_ADAPTER_HPP_

namespace ego_planner::core
{
struct PlanningProblem;
struct PlanningSolution;
} // namespace ego_planner::core

namespace ego_planner::optimization
{

class ProblemAdapter
{
public:
  virtual ~ProblemAdapter() = default;
  virtual bool solveCompatibility(const core::PlanningProblem &problem,
                                  core::PlanningSolution &solution) = 0;
};

} // namespace ego_planner::optimization

#endif // PLANNER_OPTIMIZATION_PROBLEM_ADAPTER_HPP_

