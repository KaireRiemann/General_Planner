#include <tasks/tracking_task.hpp>

namespace ego_planner::tasks
{

core::TaskDefinition TrackingTask::buildDefinition(const cost_functional::TrackingReference &reference,
                                                   const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   bool flag_poly_init,
                                                   bool flag_random_poly_traj,
                                                   bool force_plain)
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

  task.space_model_policy.force_plain = force_plain;
  task.space_model_policy.preferred =
      force_plain ? core::SpaceModelPreference::PLAIN
                  : core::SpaceModelPreference::VISIBLE_REGION;

  core::ReferenceDefinition tracking_reference;
  tracking_reference.semantic = core::ReferenceSemanticType::TRACKING_TRAJECTORY;
  tracking_reference.name = "tracking_reference";
  tracking_reference.active = true;
  tracking_reference.tracking_reference = reference;
  tracking_reference.points = reference.p_ref;
  tracking_reference.times = reference.t_ref;
  tracking_reference.velocities = reference.v_ref;
  task.references.push_back(tracking_reference);

  core::PhaseDefinition phase;
  phase.name = "tracking_follow";
  phase.goal.semantic = core::GoalSemanticType::TERMINAL_SET;
  if (reference.valid())
  {
    phase.goal.state.valid = true;
    phase.goal.state.position = reference.p_ref.back();
    phase.goal.state.velocity =
        reference.v_ref.empty() ? Eigen::Vector3d::Zero() : reference.v_ref.back();
  }
  phase.active_reference_indices.push_back(0U);
  task.phases.push_back(phase);
  task.goal = phase.goal;
  return task;
}

core::TaskSpec TrackingTask::build(const cost_functional::TrackingReference &reference,
                                   const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   bool flag_poly_init,
                                   bool flag_random_poly_traj,
                                   bool force_plain)
{
  return buildDefinition(reference,
                         start_pt,
                         start_vel,
                         start_acc,
                         flag_poly_init,
                         flag_random_poly_traj,
                         force_plain)
      .toTaskSpec();
}

} // namespace ego_planner::tasks
