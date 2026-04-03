#include <tasks/tracking_task.hpp>

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
  core::TaskDefinition task;
  task.type = core::TaskType::TRACKING;
  task.task_name = "tracking";
  task.start_state.valid = true;
  task.start_state.position = start_pt;
  task.start_state.velocity = start_vel;
  task.start_state.acceleration = start_acc;

  task.runtime_policy.flag_poly_init = flag_poly_init;
  task.runtime_policy.flag_random_poly_traj = flag_random_poly_traj;
  task.runtime_policy.touch_goal = false;
  task.runtime_policy.preserve_legacy_compatibility = false;

  task.space_model_policy.force_plain = force_plain;
  if (force_plain)
  {
    task.space_model_policy.preferred = core::SpaceModelPreference::PLAIN;
  }
  else if (prefer_corridor)
  {
    task.space_model_policy.preferred = core::SpaceModelPreference::CORRIDOR;
  }
  else if (prefer_esdf)
  {
    task.space_model_policy.preferred = core::SpaceModelPreference::ESDF;
  }
  else
  {
    task.space_model_policy.preferred = core::SpaceModelPreference::AUTO;
  }

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
  guide_reference.name = "tracking_view_reference";
  guide_reference.active = true;
  if (reference.viewValid())
  {
    guide_reference.points = reference.p_view_ref;
    guide_reference.times = reference.t_view_ref;
    guide_reference.velocities = reference.v_view_ref;
  }
  else
  {
    guide_reference.points = reference.p_ref;
    guide_reference.times = reference.t_ref;
    guide_reference.velocities = reference.v_ref;
  }
  if (guide_reference.valid())
  {
    task.references.push_back(guide_reference);
  }

  Eigen::Vector3d terminal_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d terminal_vel = Eigen::Vector3d::Zero();
  bool has_terminal = cost_functional::sampleTrackingTerminalReference(reference, terminal_pos, terminal_vel);
  if (!has_terminal && reference.valid())
  {
    terminal_pos = reference.p_ref.back();
    terminal_vel = reference.v_ref.empty() ? Eigen::Vector3d::Zero() : reference.v_ref.back();
    has_terminal = true;
  }

  task.goal.semantic = core::GoalSemanticType::FIXED_STATE;
  task.goal.state.valid = has_terminal;
  task.goal.state.position = terminal_pos;
  task.goal.state.velocity = terminal_vel;
  task.goal.touch_goal = false;

  core::PhaseDefinition phase;
  phase.name = "tracking_follow";
  phase.goal.semantic = core::GoalSemanticType::FIXED_STATE;
  if (has_terminal)
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
