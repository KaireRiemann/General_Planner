#include <runtime/perching_target_provider.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include <ros/ros.h>

namespace ego_planner::runtime
{

namespace
{

bool perchingOnlyDebugEnabled()
{
  if (const char *env = std::getenv("PERCHING_ONLY_DEBUG"))
  {
    return env[0] != '\0' && env[0] != '0';
  }
  bool enabled = false;
  ros::param::get("/debug/perching_only", enabled);
  return enabled;
}

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
                                       const double approach_velocity_alpha,
                                       const double min_prediction_time,
                                       const double max_prediction_time,
                                       const double terminal_thrust,
                                       const double terminal_thrust_range,
                                       const bool use_dynamics_terminal_accel,
                                       const Eigen::Quaterniond &fallback_landing_orientation,
                                       const bool override_target_orientation)
{
  robot_l_ = std::max(0.0, robot_l);
  v_plus_ = std::max(0.0, v_plus);
  approach_velocity_alpha_ = std::max(0.0, std::min(1.0, approach_velocity_alpha));
  min_prediction_time_ = std::max(0.05, min_prediction_time);
  max_prediction_time_ = std::max(min_prediction_time_, max_prediction_time);
  terminal_thrust_ = std::max(0.0, terminal_thrust);
  terminal_thrust_range_ = std::max(0.0, terminal_thrust_range);
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
  if (!has_target_)
  {
    return false;
  }

  const double nominal_speed = std::max(0.2, 0.75 * std::max(0.2, max_velocity));
  const double raw_time = (plate_position_ - ego_position).norm() / nominal_speed;
  const double prediction_time =
      std::min(max_prediction_time_, std::max(min_prediction_time_, raw_time));

  return buildTerminalStateAtPrediction(prediction_time, terminal);
}

bool PerchingTargetProvider::buildTerminalStateAtPrediction(const double prediction_time,
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
  Eigen::Vector3d tangent_x = landing_q.toRotationMatrix().col(0);
  Eigen::Vector3d tangent_y = landing_q.toRotationMatrix().col(1);
  if (!tangent_x.allFinite() || tangent_x.norm() < 1.0e-6)
  {
    tangent_x = Eigen::Vector3d::UnitX();
  }
  tangent_x.normalize();
  tangent_y = normal.cross(tangent_x);
  if (!tangent_y.allFinite() || tangent_y.norm() < 1.0e-6)
  {
    tangent_y = Eigen::Vector3d::UnitY();
  }
  tangent_y.normalize();
  tangent_x = tangent_y.cross(normal);
  tangent_x.normalize();

  const double clamped_prediction_time = std::max(0.0, prediction_time);
  const Eigen::Vector3d requested_preview_plate_position =
      plate_position_ + plate_velocity_ * clamped_prediction_time;
  const double approach_distance = std::max(0.4, robot_l_ + 0.2);
  terminal.valid = true;
  terminal.prediction_time = 0.0;
  terminal.approach_distance = approach_distance;
  terminal.plate_position_now = plate_position_;
  terminal.plate_position = plate_position_;
  terminal.plate_velocity = plate_velocity_;
  terminal.landing_orientation = landing_q;
  terminal.landing_tangent_x = tangent_x;
  terminal.landing_tangent_y = tangent_y;
  terminal.landing_normal = normal;
  terminal.terminal_position = terminal.plate_position + normal * robot_l_;
  terminal.terminal_velocity = plate_velocity_ - normal * v_plus_;
  terminal.terminal_acceleration = Eigen::Vector3d::Zero();
  terminal.terminal_thrust_nominal = terminal_thrust_;
  terminal.terminal_thrust_range = terminal_thrust_range_;
  terminal.use_dynamics_terminal_accel = use_dynamics_terminal_accel_;
  terminal.tangential_velocity_seed =
      has_terminal_warm_start_hint_ ? tangential_velocity_seed_hint_ : Eigen::Vector2d::Zero();
  terminal.thrust_phase_seed =
      has_terminal_warm_start_hint_ ? thrust_phase_seed_hint_ : 0.0;
  if (use_dynamics_terminal_accel_)
  {
    terminal.terminal_acceleration =
        terminal_thrust_ * normal + Eigen::Vector3d(0.0, 0.0, -9.81);
  }
  terminal.approach_anchor_position =
      terminal.terminal_position - terminal.approach_distance * normal;
  const Eigen::Vector3d previous_anchor_velocity = terminal.terminal_velocity;
  const Eigen::Vector3d softened_anchor_velocity =
      plate_velocity_ - approach_velocity_alpha_ * v_plus_ * normal;
  terminal.approach_anchor_velocity = softened_anchor_velocity;
  terminal.approach_anchor_acceleration = Eigen::Vector3d::Zero();
  ROS_INFO_THROTTLE(0.8,
                    "[PerchingTarget] current-source perching reference requested_pred_t=%.2f preview_plate=[%.2f %.2f %.2f] ref_t=%.2f ref_plate=[%.2f %.2f %.2f] alpha=%.2f prev_anchor_vel=[%.2f %.2f %.2f] new_anchor_vel=[%.2f %.2f %.2f]",
                    clamped_prediction_time,
                    requested_preview_plate_position.x(),
                    requested_preview_plate_position.y(),
                    requested_preview_plate_position.z(),
                    terminal.prediction_time,
                    terminal.plate_position.x(),
                    terminal.plate_position.y(),
                    terminal.plate_position.z(),
                    approach_velocity_alpha_,
                    previous_anchor_velocity.x(),
                    previous_anchor_velocity.y(),
                    previous_anchor_velocity.z(),
                    softened_anchor_velocity.x(),
                    softened_anchor_velocity.y(),
                    softened_anchor_velocity.z());
  if (perchingOnlyDebugEnabled())
  {
    ROS_INFO_THROTTLE(0.5,
                      "[PerchingOnlyDebug][Provider] valid=%s terminal_pos=[%.2f %.2f %.2f] terminal_vel=[%.2f %.2f %.2f] anchor_pos=[%.2f %.2f %.2f] anchor_vel=[%.2f %.2f %.2f] active_source=current_plate_state warm_hint=%s",
                      terminal.valid ? "yes" : "no",
                      terminal.terminal_position.x(),
                      terminal.terminal_position.y(),
                      terminal.terminal_position.z(),
                      terminal.terminal_velocity.x(),
                      terminal.terminal_velocity.y(),
                      terminal.terminal_velocity.z(),
                      terminal.approach_anchor_position.x(),
                      terminal.approach_anchor_position.y(),
                      terminal.approach_anchor_position.z(),
                      terminal.approach_anchor_velocity.x(),
                      terminal.approach_anchor_velocity.y(),
                      terminal.approach_anchor_velocity.z(),
                      has_terminal_warm_start_hint_ ? "yes" : "no");
  }
  return terminal.terminal_position.allFinite() &&
         terminal.approach_anchor_position.allFinite() &&
         terminal.approach_anchor_velocity.allFinite() &&
         terminal.approach_anchor_acceleration.allFinite() &&
         terminal.terminal_velocity.allFinite() &&
         terminal.terminal_acceleration.allFinite();
}

void PerchingTargetProvider::setTerminalWarmStartHint(
    const Eigen::Vector2d &tangential_velocity_seed,
    const double thrust_phase_seed)
{
  if (!tangential_velocity_seed.allFinite() || !std::isfinite(thrust_phase_seed))
  {
    clearTerminalWarmStartHint();
    return;
  }

  tangential_velocity_seed_hint_ = tangential_velocity_seed;
  thrust_phase_seed_hint_ = thrust_phase_seed;
  has_terminal_warm_start_hint_ = true;
}

void PerchingTargetProvider::clearTerminalWarmStartHint()
{
  tangential_velocity_seed_hint_.setZero();
  thrust_phase_seed_hint_ = 0.0;
  has_terminal_warm_start_hint_ = false;
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
