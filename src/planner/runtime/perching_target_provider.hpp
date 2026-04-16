#ifndef PLANNER_RUNTIME_PERCHING_TARGET_PROVIDER_HPP_
#define PLANNER_RUNTIME_PERCHING_TARGET_PROVIDER_HPP_

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

namespace ego_planner::runtime
{

struct PerchingTerminalState
{
  bool valid{false};
  double prediction_time{0.0};
  Eigen::Vector3d plate_position_now{Eigen::Vector3d::Zero()};
  // Plate state sampled at prediction_time in the future. This is the primary
  // perching reference state used by the task/manifold layer.
  Eigen::Vector3d plate_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond landing_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d landing_tangent_x{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d landing_tangent_y{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d landing_normal{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d terminal_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terminal_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terminal_acceleration{Eigen::Vector3d::Zero()};
  double terminal_thrust_nominal{9.81};
  double terminal_thrust_range{0.0};
  bool use_dynamics_terminal_accel{false};
  Eigen::Vector2d tangential_velocity_seed{Eigen::Vector2d::Zero()};
  double thrust_phase_seed{0.0};
};

// Converts a moving landing plate odometry into perching terminal semantics.
// This is a runtime semantic adapter only: it predicts the landing/contact
// state consumed by TaskFactory/PerchingTask, while shared transit frontend
// initialization still owns guide/corridor/seed generation.
class PerchingTargetProvider
{
public:
  void configure(double robot_l,
                 double v_plus,
                 double min_prediction_time,
                 double max_prediction_time,
                 double terminal_thrust,
                 double terminal_thrust_range,
                 bool use_dynamics_terminal_accel,
                 const Eigen::Quaterniond &fallback_landing_orientation,
                 bool override_target_orientation);

  void updateFromOdometry(const nav_msgs::Odometry &odom);

  bool hasTarget() const
  {
    return has_target_;
  }

  bool buildTerminalState(const Eigen::Vector3d &ego_position,
                          double max_velocity,
                          PerchingTerminalState &terminal) const;

  bool buildTerminalStateAtPrediction(double prediction_time,
                                      PerchingTerminalState &terminal) const;

  static Eigen::Quaterniond quaternionFromAxisAngle(const Eigen::Vector3d &axis,
                                                    double theta);

private:
  double robot_l_{0.02};
  double v_plus_{0.3};
  double min_prediction_time_{1.0};
  double max_prediction_time_{5.0};
  double terminal_thrust_{9.81};
  double terminal_thrust_range_{0.0};
  bool use_dynamics_terminal_accel_{false};
  bool override_target_orientation_{false};
  bool has_target_{false};

  Eigen::Vector3d plate_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond plate_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond fallback_landing_orientation_{Eigen::Quaterniond::Identity()};
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_PERCHING_TARGET_PROVIDER_HPP_
