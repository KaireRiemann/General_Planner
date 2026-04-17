#ifndef PLANNER_CORE_TASK_SEMANTIC_ARTIFACT_HPP_
#define PLANNER_CORE_TASK_SEMANTIC_ARTIFACT_HPP_

#include <core/goal_definition.hpp>
#include <core/task_spec.hpp>
#include <core/tracking_semantic_artifact.hpp>

namespace ego_planner::core
{

// TaskSemanticArtifact is the explicit semantic boundary passed downstream from
// TaskDefinition. It contains task meaning only, without any transit
// initialization artifacts or optimizer-owned solve state.
struct TransitSemanticArtifact
{
  bool valid{false};
  StateDefinition start_state;
  StateDefinition terminal_state;
  bool touch_goal{false};
};

struct PerchingSemanticArtifact
{
  bool valid{false};
  StateDefinition contact_state;
  StateDefinition approach_anchor_state;
  double approach_distance{0.0};
  Eigen::VectorXd terminal_manifold_params;
  bool touch_goal{true};
};

struct TaskSemanticArtifact
{
  TaskType task_type{TaskType::UNKNOWN};
  TransitSemanticArtifact transit;
  TrackingSemanticArtifact tracking;
  PerchingSemanticArtifact perching;

  bool valid() const
  {
    if (!transit.valid)
    {
      return false;
    }

    switch (task_type)
    {
    case TaskType::TRACKING:
      return tracking.consistent();
    case TaskType::PERCHING:
      return perching.valid;
    case TaskType::STATE_TO_STATE:
      return true;
    case TaskType::UNKNOWN:
    default:
      return false;
    }
  }
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_TASK_SEMANTIC_ARTIFACT_HPP_
