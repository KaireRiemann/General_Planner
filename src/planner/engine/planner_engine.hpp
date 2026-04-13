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

// PlannerEngine is the unified task-level solve facade:
// TaskDefinition + PlanningContext -> ProblemCompiler -> PlanningProblem -> solveProblem -> PlanningSolution.
// ProblemCompiler owns semantics + optional hints, StateToStateInitializer owns final state-to-state
// construction, and planner_manager only hosts shared resources/modules.
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
  bool solveTrackingCompiled(const core::PlanningProblem &problem,
                             core::PlanningSolution &solution) override;
  bool solveTrackingLegacy(const core::PlanningProblem &problem,
                           core::PlanningSolution &solution) override;
  bool solvePerchingLegacy(const core::PlanningProblem &problem,
                           core::PlanningSolution &solution) override;

private:
  bool solvePerchingLegacyTask(const core::TaskSpec &task,
                               core::PlanningSolution &solution);
  bool solveStateToStateCompiledProblem(const core::PlanningProblem &problem,
                                        core::PlanningSolution &solution);
  bool solveTrackingCompiledProblem(const core::PlanningProblem &problem,
                                    core::PlanningSolution &solution);
  bool solvePerchingCompiledProblem(const core::PlanningProblem &problem,
                                    core::PlanningSolution &solution);

private:
  EGOPlannerManager *planner_manager_{nullptr};
  compiler::ProblemCompiler problem_compiler_;
  std::unique_ptr<optimization::BackendSolver> backend_solver_;
  solver::StateToStateInitializer state_to_state_initializer_;
};

} // namespace ego_planner::engine

#endif // PLANNER_ENGINE_PLANNER_ENGINE_HPP_
