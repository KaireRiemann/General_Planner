#include <tasks/task_factory.hpp>

namespace ego_planner::tasks
{

core::TaskDefinition TaskFactory::makeStateToStateDefinition(const Eigen::Vector3d &start_pt,
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
  return StateToStateTask::buildDefinition(start_pt,
                                           start_vel,
                                           start_acc,
                                           goal_pt,
                                           goal_vel,
                                           touch_goal,
                                           flag_poly_init,
                                           flag_random_poly_traj,
                                           force_plain,
                                           prefer_corridor,
                                           prefer_esdf);
}

core::TaskSpec TaskFactory::makeStateToStateTask(const Eigen::Vector3d &start_pt,
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
  return makeStateToStateDefinition(start_pt,
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

core::TaskDefinition TaskFactory::makeTrackingDefinition(const cost_functional::TrackingReference &reference,
                                                         const Eigen::Vector3d &start_pt,
                                                         const Eigen::Vector3d &start_vel,
                                                         const Eigen::Vector3d &start_acc,
                                                         bool flag_poly_init,
                                                         bool flag_random_poly_traj,
                                                         bool force_plain)
{
  return TrackingTask::buildDefinition(reference,
                                       start_pt,
                                       start_vel,
                                       start_acc,
                                       flag_poly_init,
                                       flag_random_poly_traj,
                                       force_plain);
}

core::TaskSpec TaskFactory::makeTrackingTask(const cost_functional::TrackingReference &reference,
                                             const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel,
                                             const Eigen::Vector3d &start_acc,
                                             bool flag_poly_init,
                                             bool flag_random_poly_traj,
                                             bool force_plain)
{
  return makeTrackingDefinition(reference,
                                start_pt,
                                start_vel,
                                start_acc,
                                flag_poly_init,
                                flag_random_poly_traj,
                                force_plain)
      .toTaskSpec();
}

core::TaskDefinition TaskFactory::makePerchingDefinition(const Eigen::Vector3d &start_pt,
                                                         const Eigen::Vector3d &start_vel,
                                                         const Eigen::Vector3d &start_acc,
                                                         const Eigen::Vector3d &contact_pt,
                                                         const Eigen::Vector3d &contact_vel,
                                                         bool force_plain)
{
  return PerchingTask::buildDefinition(start_pt,
                                       start_vel,
                                       start_acc,
                                       contact_pt,
                                       contact_vel,
                                       force_plain);
}

core::TaskSpec TaskFactory::makePerchingTask(const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel,
                                             const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &contact_pt,
                                             const Eigen::Vector3d &contact_vel,
                                             bool force_plain)
{
  return makePerchingDefinition(start_pt,
                                start_vel,
                                start_acc,
                                contact_pt,
                                contact_vel,
                                force_plain)
      .toTaskSpec();
}

} // namespace ego_planner::tasks
