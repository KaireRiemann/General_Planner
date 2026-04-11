#include <tasks/tracking_task.hpp>
#include <tasks/state_to_state_task.hpp>

namespace ego_planner::tasks
{

core::TaskDefinition TrackingTask::buildDefinition(const cost_functional::TrackingReference &reference,
                                                   const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   bool flag_poly_init,
                                                   bool flag_random_poly_traj,
                                                   bool force_plain,
                                                   bool prefer_corridor,
                                                   bool prefer_esdf)
{
  const core::TrackingSemanticArtifact tracking_semantics =
      core::TrackingSemanticArtifact::fromTrackingReference(reference);

  const Eigen::Vector3d terminal_pos =
      tracking_semantics.anchor_terminal_state.valid
          ? tracking_semantics.anchor_terminal_state.position
          : (reference.valid() ? reference.p_ref.back() : Eigen::Vector3d::Zero());
  const Eigen::Vector3d terminal_vel =
      tracking_semantics.anchor_terminal_state.valid
          ? tracking_semantics.anchor_terminal_state.velocity
          : ((!reference.v_ref.empty()) ? reference.v_ref.back() : Eigen::Vector3d::Zero());

  // Tracking is a transit-style task with tracking-specific semantics layered
  // on top. Shared transit template owns space-model/runtime defaults, while
  // tracking reference processing and backend objectives remain task-specific.
  core::TaskDefinition task = StateToStateTask::buildDefinition(start_pt,
                                                                start_vel,
                                                                start_acc,
                                                                terminal_pos,
                                                                terminal_vel,
                                                                false,
                                                                flag_poly_init,
                                                                flag_random_poly_traj,
                                                                force_plain,
                                                                prefer_corridor,
                                                                prefer_esdf);
  task.type = core::TaskType::TRACKING;
  task.task_name = "tracking";
  task.tracking_semantics = tracking_semantics;

  task.runtime_policy.touch_goal = false;
  task.runtime_policy.preserve_legacy_compatibility = false;

  core::ReferenceDefinition tracking_reference;
  tracking_reference.semantic = core::ReferenceSemanticType::TRACKING_TRAJECTORY;
  tracking_reference.name = "tracking_reference";
  tracking_reference.active = true;
  tracking_reference.tracking_reference = reference;
  tracking_reference.points = reference.p_ref;
  tracking_reference.times = reference.t_ref;
  tracking_reference.velocities = reference.v_ref;
  task.references.push_back(tracking_reference);

  core::ReferenceDefinition guide_reference;
  guide_reference.semantic = core::ReferenceSemanticType::GUIDE_PATH;
  guide_reference.name = "tracking_semantic_guide";
  guide_reference.active = true;
  if (tracking_semantics.hasViewpointHints())
  {
    guide_reference.points = tracking_semantics.viewpoint_series;
    guide_reference.times = tracking_semantics.viewpoint_times;
    guide_reference.velocities = tracking_semantics.viewpoint_velocities;
  }
  else
  {
    guide_reference.points = tracking_semantics.semantic_guide_path;
    guide_reference.times = tracking_semantics.semantic_guide_times;
    guide_reference.velocities = reference.v_ref;
  }
  if (guide_reference.valid())
  {
    task.references.push_back(guide_reference);
  }

  task.goal.semantic = core::GoalSemanticType::FIXED_STATE;
  task.goal.state.valid = tracking_semantics.anchor_terminal_state.valid;
  task.goal.state.position = terminal_pos;
  task.goal.state.velocity = terminal_vel;
  task.goal.touch_goal = false;

  task.phases.clear();
  core::PhaseDefinition phase;
  phase.name = "tracking_follow";
  phase.goal.semantic = core::GoalSemanticType::FIXED_STATE;
  if (tracking_semantics.anchor_terminal_state.valid)
  {
    phase.goal.state.valid = true;
    phase.goal.state.position = terminal_pos;
    phase.goal.state.velocity = terminal_vel;
  }
  phase.active_reference_indices.push_back(0U);
  if (task.references.size() > 1)
  {
    phase.active_reference_indices.push_back(1U);
  }
  task.phases.push_back(phase);
  return task;
}

core::TaskSpec TrackingTask::build(const cost_functional::TrackingReference &reference,
                                   const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   bool flag_poly_init,
                                   bool flag_random_poly_traj,
                                   bool force_plain,
                                   bool prefer_corridor,
                                   bool prefer_esdf)
{
  return buildDefinition(reference,
                         start_pt,
                         start_vel,
                         start_acc,
                         flag_poly_init,
                         flag_random_poly_traj,
                         force_plain,
                         prefer_corridor,
                         prefer_esdf)
      .toTaskSpec();
}

} // namespace ego_planner::tasks
