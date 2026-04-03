#ifndef PLANNER_ENGINE_PLANNER_ENGINE_HPP_
#define PLANNER_ENGINE_PLANNER_ENGINE_HPP_

#include <memory>

#include <compiler/problem_compiler.hpp>
#include <optimization/backend_solver.hpp>
#include <optimization/problem_adapter.hpp>
#include <solver/state_to_state_initializer.hpp>

namespace ego_planner
{
class EGOPlannerManager;
}

namespace ego_planner::engine
{

// This system is organized as:
// TaskDefinition + PlanningContext -> ProblemCompiler -> PlanningProblem -> BackendSolver -> PlanningSolution.
// Task-specific semantics belong in TaskSpec/ProblemCompiler; planner_manager only hosts shared resources.
class PlannerEngine : public optimization::ProblemAdapter
{
public:
  explicit PlannerEngine(EGOPlannerManager *planner_manager);

  bool solveTask(const core::PlanningContext &context,
                 const core::TaskDefinition &task_definition,
                 core::PlanningSolution &solution);

  bool solveTask(const core::PlanningContext &context,
                 const core::TaskSpec &task,
                 core::PlanningSolution &solution);

  bool solveProblem(const core::PlanningProblem &problem,
                    core::PlanningSolution &solution);

  bool solveCompatibility(const core::PlanningProblem &problem,
                          core::PlanningSolution &solution) override;
  bool solveStateToStateCompiled(const core::PlanningProblem &problem,
                                 core::PlanningSolution &solution) override;
  bool solveTrackingLegacy(const core::PlanningProblem &problem,
                           core::PlanningSolution &solution) override;
  bool solvePerchingLegacy(const core::PlanningProblem &problem,
                           core::PlanningSolution &solution) override;

private:
  bool solveStateToStateLegacy(const core::TaskSpec &task,
                               core::PlanningSolution &solution);
  bool solveTrackingLegacyTask(const core::TaskSpec &task,
                               core::PlanningSolution &solution);
  bool solvePerchingLegacyTask(const core::TaskSpec &task,
                               core::PlanningSolution &solution);
  bool solveStateToStateCompiledProblem(const core::PlanningProblem &problem,
                                        core::PlanningSolution &solution);
  solver::StateToStateInitResources makeStateToStateInitResources() const;

private:
  EGOPlannerManager *planner_manager_{nullptr};
  compiler::ProblemCompiler problem_compiler_;
  std::unique_ptr<optimization::BackendSolver> backend_solver_;
  solver::StateToStateInitializer state_to_state_initializer_;
};

} // namespace ego_planner::engine

#endif // PLANNER_ENGINE_PLANNER_ENGINE_HPP_
