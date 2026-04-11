#ifndef PLANNER_CORE_TRACKING_SEMANTIC_ARTIFACT_HPP_
#define PLANNER_CORE_TRACKING_SEMANTIC_ARTIFACT_HPP_

#include <core/goal_definition.hpp>

#include <CostFunctionalManager/TrackingSemanticGuide.hpp>
#include <CostFunctionalManager/TrackingTypes.hpp>

#include <string>
#include <vector>

namespace ego_planner::core
{

struct TrackingObjectiveMetadata
{
  bool enable_distance{true};
  bool enable_view{true};
  bool enable_visibility{true};
  bool enable_yaw{true};
  bool use_view_terminal{false};
};

struct TrackingSemanticArtifact
{
  bool valid{false};
  StateDefinition anchor_terminal_state;
  std::vector<Eigen::Vector3d> semantic_guide_path;
  std::vector<double> semantic_guide_times;
  std::vector<Eigen::Vector3d> viewpoint_series;
  std::vector<double> viewpoint_times;
  std::vector<Eigen::Vector3d> viewpoint_velocities;
  std::vector<cost_functional::VisibleFanRegion> visible_regions;
  TrackingObjectiveMetadata objective_metadata;

  bool hasSemanticGuidePath() const
  {
    return semantic_guide_path.size() >= 2 &&
           semantic_guide_times.size() == semantic_guide_path.size();
  }

  bool hasViewpointHints() const
  {
    return viewpoint_series.size() >= 2 &&
           viewpoint_times.size() == viewpoint_series.size() &&
           (viewpoint_velocities.empty() || viewpoint_velocities.size() == viewpoint_series.size());
  }

  bool hasVisibilityHints() const
  {
    return !visible_regions.empty();
  }

  bool consistent() const
  {
    if (!valid || !anchor_terminal_state.valid || !hasSemanticGuidePath())
    {
      return false;
    }

    if (!viewpoint_series.empty() &&
        (viewpoint_series.size() < 2 ||
         viewpoint_series.size() != viewpoint_times.size() ||
         (!viewpoint_velocities.empty() && viewpoint_velocities.size() != viewpoint_series.size())))
    {
      return false;
    }

    const std::size_t semantic_sample_count =
        viewpoint_series.empty() ? semantic_guide_path.size() : viewpoint_series.size();
    if (!visible_regions.empty() && visible_regions.size() != semantic_sample_count)
    {
      return false;
    }

    return true;
  }

  bool buildSemanticGuide(cost_functional::TrackingSemanticGuide &guide) const
  {
    guide.clear();
    if (!consistent())
    {
      return false;
    }

    if (hasViewpointHints())
    {
      guide.viewpoint_series = viewpoint_series;
      guide.viewpoint_times = viewpoint_times;
    }
    else
    {
      guide.viewpoint_series = semantic_guide_path;
      guide.viewpoint_times = semantic_guide_times;
    }

    guide.visible_regions = visible_regions;
    guide.corridor_seed_path = semantic_guide_path;
    guide.corridor_seed_times = semantic_guide_times;
    guide.valid = true;
    if (!guide.consistent())
    {
      guide.clear();
      return false;
    }
    return true;
  }

  static TrackingSemanticArtifact fromTrackingReference(const cost_functional::TrackingReference &reference)
  {
    TrackingSemanticArtifact artifact;
    if (!reference.valid())
    {
      return artifact;
    }

    cost_functional::TrackingReference normalized = reference;
    std::string normalize_reason;
    if (!cost_functional::normalizeTrackingReference(reference, normalized, &normalize_reason))
    {
      (void)normalize_reason;
      normalized = reference;
    }

    Eigen::Vector3d p_term = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_term = Eigen::Vector3d::Zero();
    bool has_terminal = cost_functional::sampleTrackingTerminalReference(normalized, p_term, v_term);
    if (!has_terminal)
    {
      p_term = normalized.p_ref.back();
      v_term = normalized.v_ref.empty() ? Eigen::Vector3d::Zero() : normalized.v_ref.back();
      has_terminal = true;
    }

    artifact.anchor_terminal_state.valid = has_terminal;
    artifact.anchor_terminal_state.position = p_term;
    artifact.anchor_terminal_state.velocity = v_term;
    artifact.anchor_terminal_state.acceleration = Eigen::Vector3d::Zero();

    if (normalized.viewValid())
    {
      artifact.semantic_guide_path = normalized.p_view_ref;
      artifact.semantic_guide_times = normalized.t_view_ref;
      artifact.viewpoint_series = normalized.p_view_ref;
      artifact.viewpoint_times = normalized.t_view_ref;
      artifact.viewpoint_velocities = normalized.v_view_ref;
    }
    else
    {
      artifact.semantic_guide_path = normalized.p_ref;
      artifact.semantic_guide_times = normalized.t_ref;
    }

    artifact.objective_metadata.enable_distance = true;
    artifact.objective_metadata.enable_view = normalized.viewValid();
    artifact.objective_metadata.enable_visibility = normalized.viewValid();
    artifact.objective_metadata.enable_yaw = true;
    artifact.objective_metadata.use_view_terminal = normalized.use_view_terminal;

    artifact.valid = artifact.anchor_terminal_state.valid && artifact.hasSemanticGuidePath();
    return artifact;
  }
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_TRACKING_SEMANTIC_ARTIFACT_HPP_
