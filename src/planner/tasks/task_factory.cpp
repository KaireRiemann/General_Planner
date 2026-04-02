#include <tasks/task_factory.hpp>

namespace ego_planner::tasks
{

core::TaskSpec TaskFactory::makeStateToStateTask(const Eigen::Vector3d &start_pt,
                                                 const Eigen::Vector3d &start_vel,
                                                 const Eigen::Vector3d &start_acc,
                                                 const Eigen::Vector3d &goal_pt,
                                                 const Eigen::Vector3d &goal_vel,
                                                 bool touch_goal,
                                                 bool flag_poly_init,
                                                 bool flag_random_poly_traj,
                                                 bool force_plain)
{
  return StateToStateTask::build(start_pt,
                                 start_vel,
                                 start_acc,
                                 goal_pt,
                                 goal_vel,
                                 touch_goal,
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
  return TrackingTask::build(reference,
                             start_pt,
                             start_vel,
                             start_acc,
                             flag_poly_init,
                             flag_random_poly_traj,
                             force_plain);
}

core::TaskSpec TaskFactory::makePerchingTask(const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel,
                                             const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &contact_pt,
                                             const Eigen::Vector3d &contact_vel,
                                             bool force_plain)
{
  return PerchingTask::build(start_pt,
                             start_vel,
                             start_acc,
                             contact_pt,
                             contact_vel,
                             force_plain);
}

} // namespace ego_planner::tasks

