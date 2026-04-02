#include <tasks/state_to_state_task.hpp>

namespace ego_planner::tasks
{

core::TaskDefinition StateToStateTask::buildDefinition(const Eigen::Vector3d &start_pt,
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
  core::TaskDefinition task;
  task.type = core::TaskType::STATE_TO_STATE;
  task.task_name = "state_to_state";
  task.start_state.valid = true;
  task.start_state.position = start_pt;
  task.start_state.velocity = start_vel;
  task.start_state.acceleration = start_acc;

  task.goal.semantic = core::GoalSemanticType::FIXED_STATE;
  task.goal.state.valid = true;
  task.goal.state.position = goal_pt;
  task.goal.state.velocity = goal_vel;
  task.goal.touch_goal = touch_goal;

  task.runtime_policy.touch_goal = touch_goal;
  task.runtime_policy.flag_poly_init = flag_poly_init;
  task.runtime_policy.flag_random_poly_traj = flag_random_poly_traj;

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

  core::PhaseDefinition phase;
  phase.name = "cruise";
  phase.goal = task.goal;
  task.phases.push_back(phase);
  return task;
}

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
  return buildDefinition(start_pt,
                         start_vel,
                         start_acc,
                         goal_pt,
                         goal_vel,
                         touch_goal,
                         flag_poly_init,
                         flag_random_poly_traj,
                         force_plain,
                         prefer_corridor,
                         prefer_esdf)
      .toTaskSpec();
}

} // namespace ego_planner::tasks
