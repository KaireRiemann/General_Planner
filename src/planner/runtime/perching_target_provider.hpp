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
  Eigen::Vector3d plate_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond landing_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d landing_normal{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d terminal_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terminal_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terminal_acceleration{Eigen::Vector3d::Zero()};
};

// Converts a moving landing plate odometry into the fixed terminal boundary
// used by Perching V1. Position/velocity mirror Fast-Perching's terminal-state
// idea. Terminal acceleration is optional because the current generic backend
// is still a kinematic MINCO boundary solver rather than Fast-Perching's full
// terminal thrust/time/tangent-velocity optimizer.
class PerchingTargetProvider
{
public:
  void configure(double robot_l,
                 double v_plus,
                 double min_prediction_time,
                 double max_prediction_time,
                 double terminal_thrust,
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

  static Eigen::Quaterniond quaternionFromAxisAngle(const Eigen::Vector3d &axis,
                                                    double theta);

private:
  double robot_l_{0.02};
  double v_plus_{0.3};
  double min_prediction_time_{1.0};
  double max_prediction_time_{5.0};
  double terminal_thrust_{9.81};
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
