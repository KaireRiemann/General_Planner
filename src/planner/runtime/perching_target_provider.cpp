#include <runtime/perching_target_provider.hpp>

#include <algorithm>
#include <cmath>

namespace ego_planner::runtime
{

namespace
{

Eigen::Quaterniond normalizedOrIdentity(Eigen::Quaterniond q)
{
  if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
      !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
      q.norm() < 1.0e-6)
  {
    return Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

} // namespace

void PerchingTargetProvider::configure(const double robot_l,
                                       const double v_plus,
                                       const double min_prediction_time,
                                       const double max_prediction_time,
                                       const double terminal_thrust,
                                       const bool use_dynamics_terminal_accel,
                                       const Eigen::Quaterniond &fallback_landing_orientation,
                                       const bool override_target_orientation)
{
  robot_l_ = std::max(0.0, robot_l);
  v_plus_ = std::max(0.0, v_plus);
  min_prediction_time_ = std::max(0.05, min_prediction_time);
  max_prediction_time_ = std::max(min_prediction_time_, max_prediction_time);
  terminal_thrust_ = std::max(0.0, terminal_thrust);
  use_dynamics_terminal_accel_ = use_dynamics_terminal_accel;
  fallback_landing_orientation_ = normalizedOrIdentity(fallback_landing_orientation);
  override_target_orientation_ = override_target_orientation;
}

void PerchingTargetProvider::updateFromOdometry(const nav_msgs::Odometry &odom)
{
  plate_position_.x() = odom.pose.pose.position.x;
  plate_position_.y() = odom.pose.pose.position.y;
  plate_position_.z() = odom.pose.pose.position.z;
  plate_velocity_.x() = odom.twist.twist.linear.x;
  plate_velocity_.y() = odom.twist.twist.linear.y;
  plate_velocity_.z() = odom.twist.twist.linear.z;

  plate_orientation_ = normalizedOrIdentity(
      Eigen::Quaterniond(odom.pose.pose.orientation.w,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z));
  has_target_ = true;
}

bool PerchingTargetProvider::buildTerminalState(const Eigen::Vector3d &ego_position,
                                                const double max_velocity,
                                                PerchingTerminalState &terminal) const
{
  terminal = PerchingTerminalState{};
  if (!has_target_)
  {
    return false;
  }

  const Eigen::Quaterniond landing_q =
      override_target_orientation_ ? fallback_landing_orientation_ : plate_orientation_;
  Eigen::Vector3d normal = landing_q.toRotationMatrix().col(2);
  if (!normal.allFinite() || normal.norm() < 1.0e-6)
  {
    normal = Eigen::Vector3d::UnitZ();
  }
  normal.normalize();

  const double nominal_speed = std::max(0.2, 0.75 * max_velocity);
  const double raw_time = (plate_position_ - ego_position).norm() / nominal_speed;
  const double prediction_time =
      std::min(max_prediction_time_, std::max(min_prediction_time_, raw_time));

  terminal.valid = true;
  terminal.prediction_time = prediction_time;
  terminal.plate_position = plate_position_ + plate_velocity_ * prediction_time;
  terminal.plate_velocity = plate_velocity_;
  terminal.landing_orientation = landing_q;
  terminal.landing_normal = normal;
  terminal.terminal_position = terminal.plate_position + normal * robot_l_;
  terminal.terminal_velocity = plate_velocity_ - normal * v_plus_;
  terminal.terminal_acceleration = Eigen::Vector3d::Zero();
  if (use_dynamics_terminal_accel_)
  {
    terminal.terminal_acceleration =
        terminal_thrust_ * normal + Eigen::Vector3d(0.0, 0.0, -9.81);
  }
  return terminal.terminal_position.allFinite() &&
         terminal.terminal_velocity.allFinite() &&
         terminal.terminal_acceleration.allFinite();
}

Eigen::Quaterniond PerchingTargetProvider::quaternionFromAxisAngle(const Eigen::Vector3d &axis,
                                                                   const double theta)
{
  Eigen::Vector3d safe_axis = axis;
  if (!safe_axis.allFinite() || safe_axis.norm() < 1.0e-6)
  {
    safe_axis = Eigen::Vector3d::UnitZ();
  }
  safe_axis.normalize();
  return Eigen::Quaterniond(Eigen::AngleAxisd(theta, safe_axis));
}

} // namespace ego_planner::runtime
