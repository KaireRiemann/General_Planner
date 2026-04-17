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
                                                         bool force_plain,
                                                         bool prefer_corridor,
                                                         bool prefer_esdf)
{
  return TrackingTask::buildDefinition(reference,
                                       start_pt,
                                       start_vel,
                                       start_acc,
                                       flag_poly_init,
                                       flag_random_poly_traj,
                                       force_plain,
                                       prefer_corridor,
                                       prefer_esdf);
}

core::TaskSpec TaskFactory::makeTrackingTask(const cost_functional::TrackingReference &reference,
                                             const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel,
                                             const Eigen::Vector3d &start_acc,
                                             bool flag_poly_init,
                                             bool flag_random_poly_traj,
                                             bool force_plain,
                                             bool prefer_corridor,
                                             bool prefer_esdf)
{
  return makeTrackingDefinition(reference,
                                start_pt,
                                start_vel,
                                start_acc,
                                flag_poly_init,
                                flag_random_poly_traj,
                                force_plain,
                                prefer_corridor,
                                prefer_esdf)
      .toTaskSpec();
}

core::TaskDefinition TaskFactory::makePerchingDefinition(const Eigen::Vector3d &start_pt,
                                                         const Eigen::Vector3d &start_vel,
                                                         const Eigen::Vector3d &start_acc,
                                                         const Eigen::Vector3d &approach_anchor_pt,
                                                         const Eigen::Vector3d &approach_anchor_vel,
                                                         const Eigen::Vector3d &approach_anchor_acc,
                                                         const Eigen::Vector3d &contact_pt,
                                                         const Eigen::Vector3d &contact_vel,
                                                         const Eigen::Vector3d &contact_acc,
                                                         const Eigen::Vector3d &plate_position_ref,
                                                         const Eigen::Vector3d &plate_velocity,
                                                         const double prediction_time,
                                                         const Eigen::Vector3d &landing_tangent_x,
                                                         const Eigen::Vector3d &landing_tangent_y,
                                                         const Eigen::Vector3d &landing_normal,
                                                         const Eigen::Vector2d &tangential_velocity_seed,
                                                         const double thrust_phase_seed,
                                                         const double robot_l,
                                                         const double v_plus,
                                                         const double terminal_thrust_nominal,
                                                         const double terminal_thrust_range,
                                                         const bool use_dynamics_terminal_accel,
                                                         const bool force_plain,
                                                         const bool prefer_corridor,
                                                         const bool prefer_esdf)
{
  return PerchingTask::buildDefinition(start_pt,
                                       start_vel,
                                       start_acc,
                                       approach_anchor_pt,
                                       approach_anchor_vel,
                                       approach_anchor_acc,
                                       contact_pt,
                                       contact_vel,
                                       contact_acc,
                                       plate_position_ref,
                                       plate_velocity,
                                       prediction_time,
                                       landing_tangent_x,
                                       landing_tangent_y,
                                       landing_normal,
                                       tangential_velocity_seed,
                                       thrust_phase_seed,
                                       robot_l,
                                       v_plus,
                                       terminal_thrust_nominal,
                                       terminal_thrust_range,
                                       use_dynamics_terminal_accel,
                                       force_plain,
                                       prefer_corridor,
                                       prefer_esdf);
}

core::TaskDefinition TaskFactory::makePerchingDefinition(const Eigen::Vector3d &start_pt,
                                                         const Eigen::Vector3d &start_vel,
                                                         const Eigen::Vector3d &start_acc,
                                                         const Eigen::Vector3d &contact_pt,
                                                         const Eigen::Vector3d &contact_vel,
                                                         const bool force_plain)
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
                                             const Eigen::Vector3d &approach_anchor_pt,
                                             const Eigen::Vector3d &approach_anchor_vel,
                                             const Eigen::Vector3d &approach_anchor_acc,
                                             const Eigen::Vector3d &contact_pt,
                                             const Eigen::Vector3d &contact_vel,
                                             const Eigen::Vector3d &contact_acc,
                                             const Eigen::Vector3d &plate_position_ref,
                                             const Eigen::Vector3d &plate_velocity,
                                             const double prediction_time,
                                             const Eigen::Vector3d &landing_tangent_x,
                                             const Eigen::Vector3d &landing_tangent_y,
                                             const Eigen::Vector3d &landing_normal,
                                             const Eigen::Vector2d &tangential_velocity_seed,
                                             const double thrust_phase_seed,
                                             const double robot_l,
                                             const double v_plus,
                                             const double terminal_thrust_nominal,
                                             const double terminal_thrust_range,
                                             const bool use_dynamics_terminal_accel,
                                             const bool force_plain,
                                             const bool prefer_corridor,
                                             const bool prefer_esdf)
{
  return makePerchingDefinition(start_pt,
                                start_vel,
                                start_acc,
                                approach_anchor_pt,
                                approach_anchor_vel,
                                approach_anchor_acc,
                                contact_pt,
                                contact_vel,
                                contact_acc,
                                plate_position_ref,
                                plate_velocity,
                                prediction_time,
                                landing_tangent_x,
                                landing_tangent_y,
                                landing_normal,
                                tangential_velocity_seed,
                                thrust_phase_seed,
                                robot_l,
                                v_plus,
                                terminal_thrust_nominal,
                                terminal_thrust_range,
                                use_dynamics_terminal_accel,
                                force_plain,
                                prefer_corridor,
                                prefer_esdf)
      .toTaskSpec();
}

core::TaskSpec TaskFactory::makePerchingTask(const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel,
                                             const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &contact_pt,
                                             const Eigen::Vector3d &contact_vel,
                                             const bool force_plain)
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
