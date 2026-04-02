#ifndef PLANNER_OPTIMIZATION_BACKEND_SOLVER_HPP_
#define PLANNER_OPTIMIZATION_BACKEND_SOLVER_HPP_

#include <core/planning_problem.hpp>
#include <core/planning_solution.hpp>

namespace ego_planner::optimization
{

class BackendSolver
{
public:
  virtual ~BackendSolver() = default;
  virtual bool solve(const core::PlanningProblem &problem,
                     core::PlanningSolution &solution) = 0;
};

class CompatibilityBackendSolver final : public BackendSolver
{
public:
  bool solve(const core::PlanningProblem &problem,
             core::PlanningSolution &solution) override
  {
    if (!problem.valid || !problem.solve_callback)
    {
      solution.success = false;
      solution.message = "invalid planning problem or missing callback";
      return false;
    }
    return problem.solve_callback(problem, solution);
  }
};

} // namespace ego_planner::optimization

#endif // PLANNER_OPTIMIZATION_BACKEND_SOLVER_HPP_

