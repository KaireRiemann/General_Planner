#include <tasks/tracking_task.hpp>

namespace ego_planner::tasks
{

core::TaskSpec TrackingTask::build(const cost_functional::TrackingReference &reference,
                                   const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   bool flag_poly_init,
                                   bool flag_random_poly_traj,
                                   bool force_plain)
{
  core::TaskSpec task;
  task.type = core::TaskType::TRACKING;
  task.task_name = "tracking";
  task.start_pt = start_pt;
  task.start_vel = start_vel;
  task.start_acc = start_acc;
  task.flag_poly_init = flag_poly_init;
  task.flag_random_poly_traj = flag_random_poly_traj;
  task.force_plain = force_plain;
  task.touch_goal = false;
  task.tracking_reference = reference;

  core::PhaseSpec phase;
  phase.name = "tracking_follow";
  phase.terminal_type = core::TerminalSpecType::TERMINAL_SET;
  if (reference.valid())
  {
    phase.terminal_position = reference.p_ref.back();
    phase.terminal_velocity = reference.v_ref.empty() ? Eigen::Vector3d::Zero() : reference.v_ref.back();
    task.goal_pt = reference.p_ref.back();
    task.goal_vel = phase.terminal_velocity;
  }
  task.phases.push_back(phase);
  return task;
}

} // namespace ego_planner::tasks
