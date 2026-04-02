#include <tasks/state_to_state_task.hpp>

namespace ego_planner::tasks
{

core::TaskSpec StateToStateTask::build(const Eigen::Vector3d &start_pt,
                                       const Eigen::Vector3d &start_vel,
                                       const Eigen::Vector3d &start_acc,
                                       const Eigen::Vector3d &goal_pt,
                                       const Eigen::Vector3d &goal_vel,
                                       bool touch_goal,
                                       bool flag_poly_init,
                                       bool flag_random_poly_traj,
                                       bool force_plain,
                                       bool prefer_corridor,
                                       bool prefer_esdf)
{
  core::TaskSpec task;
  task.type = core::TaskType::STATE_TO_STATE;
  task.task_name = "state_to_state";
  task.start_pt = start_pt;
  task.start_vel = start_vel;
  task.start_acc = start_acc;
  task.goal_pt = goal_pt;
  task.goal_vel = goal_vel;
  task.touch_goal = touch_goal;
  task.flag_poly_init = flag_poly_init;
  task.flag_random_poly_traj = flag_random_poly_traj;
  task.force_plain = force_plain;
  task.prefer_corridor = prefer_corridor;
  task.prefer_esdf = prefer_esdf;

  core::PhaseSpec phase;
  phase.name = "cruise";
  phase.terminal_type = core::TerminalSpecType::FIXED_STATE;
  phase.terminal_position = goal_pt;
  phase.terminal_velocity = goal_vel;
  task.phases.push_back(phase);
  return task;
}

} // namespace ego_planner::tasks
