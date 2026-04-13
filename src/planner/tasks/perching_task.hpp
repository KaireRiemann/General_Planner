#ifndef PLANNER_TASKS_PERCHING_TASK_HPP_
#define PLANNER_TASKS_PERCHING_TASK_HPP_

#include <core/task_definition.hpp>

namespace ego_planner::tasks
{

class PerchingTask
{
public:
  static core::TaskDefinition buildDefinition(const Eigen::Vector3d &start_pt,
                                              const Eigen::Vector3d &start_vel,
                                              const Eigen::Vector3d &start_acc,
                                              const Eigen::Vector3d &contact_pt,
                                              const Eigen::Vector3d &contact_vel,
                                              const Eigen::Vector3d &contact_acc,
                                              const Eigen::Vector3d &plate_position_now,
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

  static core::TaskDefinition buildDefinition(const Eigen::Vector3d &start_pt,
                                              const Eigen::Vector3d &start_vel,
                                              const Eigen::Vector3d &start_acc,
                                              const Eigen::Vector3d &contact_pt,
                                              const Eigen::Vector3d &contact_vel,
                                              bool force_plain);

  static core::TaskSpec build(const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &start_vel,
                              const Eigen::Vector3d &start_acc,
                              const Eigen::Vector3d &contact_pt,
                              const Eigen::Vector3d &contact_vel,
                              const Eigen::Vector3d &contact_acc,
                              const Eigen::Vector3d &plate_position_now,
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

  static core::TaskSpec build(const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &start_vel,
                              const Eigen::Vector3d &start_acc,
                              const Eigen::Vector3d &contact_pt,
                              const Eigen::Vector3d &contact_vel,
                              bool force_plain);
};

} // namespace ego_planner::tasks

#endif // PLANNER_TASKS_PERCHING_TASK_HPP_
