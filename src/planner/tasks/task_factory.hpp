#ifndef PLANNER_TASKS_TASK_FACTORY_HPP_
#define PLANNER_TASKS_TASK_FACTORY_HPP_

#include <core/task_definition.hpp>
#include <core/task_spec.hpp>
#include <tasks/perching_task.hpp>
#include <tasks/state_to_state_task.hpp>
#include <tasks/tracking_task.hpp>

namespace ego_planner::tasks
{

class TaskFactory
{
public:
  static core::TaskDefinition makeStateToStateDefinition(const Eigen::Vector3d &start_pt,
                                                         const Eigen::Vector3d &start_vel,
                                                         const Eigen::Vector3d &start_acc,
                                                         const Eigen::Vector3d &goal_pt,
                                                         const Eigen::Vector3d &goal_vel,
                                                         bool touch_goal,
                                                         bool flag_poly_init,
                                                         bool flag_random_poly_traj,
                                                         bool force_plain,
                                                         bool prefer_corridor,
                                                         bool prefer_esdf);

  static core::TaskSpec makeStateToStateTask(const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel,
                                             const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &goal_pt,
                                             const Eigen::Vector3d &goal_vel,
                                             bool touch_goal,
                                             bool flag_poly_init,
                                             bool flag_random_poly_traj,
                                             bool force_plain,
                                             bool prefer_corridor,
                                             bool prefer_esdf);

  static core::TaskDefinition makeTrackingDefinition(const cost_functional::TrackingReference &reference,
                                                     const Eigen::Vector3d &start_pt,
                                                     const Eigen::Vector3d &start_vel,
                                                     const Eigen::Vector3d &start_acc,
                                                     bool flag_poly_init,
                                                     bool flag_random_poly_traj,
                                                     bool force_plain,
                                                     bool prefer_corridor,
                                                     bool prefer_esdf);

  static core::TaskSpec makeTrackingTask(const cost_functional::TrackingReference &reference,
                                         const Eigen::Vector3d &start_pt,
                                         const Eigen::Vector3d &start_vel,
                                         const Eigen::Vector3d &start_acc,
                                         bool flag_poly_init,
                                         bool flag_random_poly_traj,
                                         bool force_plain,
                                         bool prefer_corridor,
                                         bool prefer_esdf);

  static core::TaskDefinition makePerchingDefinition(const Eigen::Vector3d &start_pt,
                                                     const Eigen::Vector3d &start_vel,
                                                     const Eigen::Vector3d &start_acc,
                                                     const Eigen::Vector3d &contact_pt,
                                                     const Eigen::Vector3d &contact_vel,
                                                     const Eigen::Vector3d &contact_acc,
                                                     const Eigen::Vector3d &plate_position,
                                                     const Eigen::Vector3d &plate_velocity,
                                                     const Eigen::Vector3d &landing_tangent_x,
                                                     const Eigen::Vector3d &landing_tangent_y,
                                                     const Eigen::Vector3d &landing_normal,
                                                     double robot_l,
                                                     double v_plus,
                                                     double terminal_thrust_nominal,
                                                     double terminal_thrust_range,
                                                     bool use_dynamics_terminal_accel,
                                                     bool force_plain,
                                                     bool prefer_corridor,
                                                     bool prefer_esdf);

  static core::TaskDefinition makePerchingDefinition(const Eigen::Vector3d &start_pt,
                                                     const Eigen::Vector3d &start_vel,
                                                     const Eigen::Vector3d &start_acc,
                                                     const Eigen::Vector3d &contact_pt,
                                                     const Eigen::Vector3d &contact_vel,
                                                     bool force_plain);

  static core::TaskSpec makePerchingTask(const Eigen::Vector3d &start_pt,
                                         const Eigen::Vector3d &start_vel,
                                         const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &contact_pt,
                                         const Eigen::Vector3d &contact_vel,
                                         const Eigen::Vector3d &contact_acc,
                                         const Eigen::Vector3d &plate_position,
                                         const Eigen::Vector3d &plate_velocity,
                                         const Eigen::Vector3d &landing_tangent_x,
                                         const Eigen::Vector3d &landing_tangent_y,
                                         const Eigen::Vector3d &landing_normal,
                                         double robot_l,
                                         double v_plus,
                                         double terminal_thrust_nominal,
                                         double terminal_thrust_range,
                                         bool use_dynamics_terminal_accel,
                                         bool force_plain,
                                         bool prefer_corridor,
                                         bool prefer_esdf);

  static core::TaskSpec makePerchingTask(const Eigen::Vector3d &start_pt,
                                         const Eigen::Vector3d &start_vel,
                                         const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &contact_pt,
                                         const Eigen::Vector3d &contact_vel,
                                         bool force_plain);
};

} // namespace ego_planner::tasks

#endif // PLANNER_TASKS_TASK_FACTORY_HPP_
