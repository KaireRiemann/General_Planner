#include <tasks/perching_task.hpp>

namespace ego_planner::tasks
{

core::TaskDefinition PerchingTask::buildDefinition(const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   const Eigen::Vector3d &contact_pt,
                                                   const Eigen::Vector3d &contact_vel,
                                                   const Eigen::Vector3d &contact_acc,
                                                   const Eigen::Vector3d &landing_normal,
                                                   const double robot_l,
                                                   const double v_plus,
                                                   const bool force_plain,
                                                   const bool prefer_corridor,
                                                   const bool prefer_esdf)
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
  task.goal.state.acceleration = contact_acc;
  task.goal.touch_goal = true;
  task.goal.manifold_params.resize(11);
  task.goal.manifold_params.segment<3>(0) = contact_pt;
  task.goal.manifold_params.segment<3>(3) = contact_vel;
  task.goal.manifold_params.segment<3>(6) =
      landing_normal.norm() > 1.0e-6 ? landing_normal.normalized()
                                      : Eigen::Vector3d::UnitZ();
  task.goal.manifold_params(9) = robot_l;
  task.goal.manifold_params(10) = v_plus;

  task.runtime_policy.touch_goal = true;
  task.runtime_policy.enable_keep_current = false;
  task.runtime_policy.enable_successor_planning = false;
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

  core::PhaseDefinition approach;
  approach.name = "approach";
  approach.goal.semantic = core::GoalSemanticType::TERMINAL_SET;
  approach.goal.state.valid = true;
  approach.goal.state.position = contact_pt;
  approach.goal.state.velocity = contact_vel;
  approach.goal.state.acceleration = contact_acc;
  task.phases.push_back(approach);

  core::PhaseDefinition contact;
  contact.name = "contact";
  contact.goal = task.goal;
  task.phases.push_back(contact);

  return task;
}

core::TaskDefinition PerchingTask::buildDefinition(const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   const Eigen::Vector3d &contact_pt,
                                                   const Eigen::Vector3d &contact_vel,
                                                   const bool force_plain)
{
  return buildDefinition(start_pt,
                         start_vel,
                         start_acc,
                         contact_pt,
                         contact_vel,
                         Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::UnitZ(),
                         0.0,
                         0.0,
                         force_plain,
                         false,
                         false);
}

core::TaskSpec PerchingTask::build(const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &contact_pt,
                                   const Eigen::Vector3d &contact_vel,
                                   const Eigen::Vector3d &contact_acc,
                                   const Eigen::Vector3d &landing_normal,
                                   const double robot_l,
                                   const double v_plus,
                                   const bool force_plain,
                                   const bool prefer_corridor,
                                   const bool prefer_esdf)
{
  return buildDefinition(start_pt,
                         start_vel,
                         start_acc,
                         contact_pt,
                         contact_vel,
                         contact_acc,
                         landing_normal,
                         robot_l,
                         v_plus,
                         force_plain,
                         prefer_corridor,
                         prefer_esdf)
      .toTaskSpec();
}

core::TaskSpec PerchingTask::build(const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &contact_pt,
                                   const Eigen::Vector3d &contact_vel,
                                   const bool force_plain)
{
  return build(start_pt,
               start_vel,
               start_acc,
               contact_pt,
               contact_vel,
               Eigen::Vector3d::Zero(),
               Eigen::Vector3d::UnitZ(),
               0.0,
               0.0,
               force_plain,
               false,
               false);
}

} // namespace ego_planner::tasks
