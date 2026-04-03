#ifndef PLANNER_COMPILER_FEASIBLE_SET_BUILDER_HPP_
#define PLANNER_COMPILER_FEASIBLE_SET_BUILDER_HPP_

#include <core/planning_problem.hpp>
#include <frontend/corridor_service.hpp>
#include <frontend/guide_path_service.hpp>
#include <frontend/visible_region_service.hpp>

namespace ego_planner::compiler
{

class FeasibleSetBuilder
{
public:
  bool build(const core::PlanningContext &context,
             const core::TaskDefinition &task_definition,
             core::PlanningProblem &problem) const;

private:
  bool ensureTransitGuidePath(const core::PlanningContext &context,
                              const core::TaskDefinition &task_definition,
                              core::PlanningProblem &problem) const;
  bool buildPlainFeasibleSets(const core::PlanningContext &context,
                              const core::TaskDefinition &task_definition,
                              core::PlanningProblem &problem) const;
  bool buildEsdfFeasibleSets(const core::PlanningContext &context,
                             const core::TaskDefinition &task_definition,
                             core::PlanningProblem &problem) const;
  bool buildCorridorFeasibleSets(const core::PlanningContext &context,
                                 const core::TaskDefinition &task_definition,
                                 core::PlanningProblem &problem) const;
  bool buildTransitFeasibleSets(const core::PlanningContext &context,
                                const core::TaskDefinition &task_definition,
                                core::PlanningProblem &problem) const;
  bool buildTrackingFeasibleSets(const core::PlanningContext &context,
                                 const core::TaskDefinition &task_definition,
                                 core::PlanningProblem &problem) const;
  bool buildPerchingFeasibleSets(const core::PlanningContext &context,
                                 const core::TaskDefinition &task_definition,
                                 core::PlanningProblem &problem) const;

  frontend::GuidePathService guide_path_service_;
  frontend::CorridorService corridor_service_;
  frontend::VisibleRegionService visible_region_service_;
};

} // namespace ego_planner::compiler

#endif // PLANNER_COMPILER_FEASIBLE_SET_BUILDER_HPP_
