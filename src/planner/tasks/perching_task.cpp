#include <tasks/perching_task.hpp>

namespace ego_planner::tasks
{

core::TaskSpec PerchingTask::build(const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &contact_pt,
                                   const Eigen::Vector3d &contact_vel,
                                   bool force_plain)
{
  core::TaskSpec task;
  task.type = core::TaskType::PERCHING;
  task.task_name = "perching";
  task.start_pt = start_pt;
  task.start_vel = start_vel;
  task.start_acc = start_acc;
  task.goal_pt = contact_pt;
  task.goal_vel = contact_vel;
  task.force_plain = force_plain;
  task.touch_goal = true;

  // Multi-phase placeholder: approach + contact.
  core::PhaseSpec approach;
  approach.name = "approach";
  approach.terminal_type = core::TerminalSpecType::TERMINAL_SET;
  approach.terminal_position = contact_pt;
  approach.terminal_velocity = contact_vel;
  task.phases.push_back(approach);

  core::PhaseSpec contact;
  contact.name = "contact";
  contact.terminal_type = core::TerminalSpecType::TERMINAL_MANIFOLD;
  contact.terminal_position = contact_pt;
  contact.terminal_velocity = contact_vel;
  task.phases.push_back(contact);

  return task;
}

} // namespace ego_planner::tasks

