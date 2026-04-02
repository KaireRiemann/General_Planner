#include <tasks/perching_task.hpp>

namespace ego_planner::tasks
{

core::TaskDefinition PerchingTask::buildDefinition(const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   const Eigen::Vector3d &contact_pt,
                                                   const Eigen::Vector3d &contact_vel,
                                                   bool force_plain)
{
  core::TaskDefinition task;
  task.type = core::TaskType::PERCHING;
  task.task_name = "perching";
  task.start_state.valid = true;
  task.start_state.position = start_pt;
  task.start_state.velocity = start_vel;
  task.start_state.acceleration = start_acc;

  task.goal.semantic = core::GoalSemanticType::TERMINAL_MANIFOLD;
  task.goal.state.valid = true;
  task.goal.state.position = contact_pt;
  task.goal.state.velocity = contact_vel;
  task.goal.touch_goal = true;

  task.runtime_policy.touch_goal = true;
  task.space_model_policy.force_plain = force_plain;
  task.space_model_policy.preferred =
      force_plain ? core::SpaceModelPreference::PLAIN
                  : core::SpaceModelPreference::TERMINAL_MANIFOLD;

  core::PhaseDefinition approach;
  approach.name = "approach";
  approach.goal.semantic = core::GoalSemanticType::TERMINAL_SET;
  approach.goal.state.valid = true;
  approach.goal.state.position = contact_pt;
  approach.goal.state.velocity = contact_vel;
  task.phases.push_back(approach);

  core::PhaseDefinition contact;
  contact.name = "contact";
  contact.goal = task.goal;
  task.phases.push_back(contact);

  return task;
}

core::TaskSpec PerchingTask::build(const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &contact_pt,
                                   const Eigen::Vector3d &contact_vel,
                                   bool force_plain)
{
  return buildDefinition(start_pt,
                         start_vel,
                         start_acc,
                         contact_pt,
                         contact_vel,
                         force_plain)
      .toTaskSpec();
}

} // namespace ego_planner::tasks
