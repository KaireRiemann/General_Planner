#ifndef PLANNER_OPTIMIZATION_BACKEND_PLUGIN_INPUT_HPP_
#define PLANNER_OPTIMIZATION_BACKEND_PLUGIN_INPUT_HPP_

#include <core/planning_problem.hpp>
#include <frontend/init_artifact.hpp>

#include <CostFunctionalManager/TrackingSemanticGuide.hpp>

namespace ego_planner::optimization
{

// BackendPluginInput is the explicit backend-facing boundary:
// task semantics + shared transit init + backend-specific references.
// Backends should consume this bundle instead of reconstructing task/init state.
struct BackendPluginInput
{
  const core::PlanningProblem *problem{nullptr};
  const core::TaskSemanticArtifact *task_semantics{nullptr};
  const frontend::InitArtifact *transit_init{nullptr};
  const cost_functional::TrackingReference *tracking_reference{nullptr};
  const cost_functional::TrackingSemanticGuide *tracking_semantic_guide{nullptr};

  bool hasTransitInit() const
  {
    return transit_init != nullptr && transit_init->valid;
  }

  bool hasTrackingReference() const
  {
    return tracking_reference != nullptr && tracking_reference->valid();
  }

  bool hasTrackingSemanticGuide() const
  {
    return tracking_semantic_guide != nullptr && tracking_semantic_guide->consistent();
  }
};

} // namespace ego_planner::optimization

#endif // PLANNER_OPTIMIZATION_BACKEND_PLUGIN_INPUT_HPP_
