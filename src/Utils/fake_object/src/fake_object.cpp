#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace
{

Eigen::Quaterniond yawToQuaternion(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_object");
  ros::NodeHandle nh("~");

  std::string frame_id = "world";
  std::string motion_mode = "circle";
  double publish_rate = 30.0;
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 1.0;
  double radius = 2.5;
  double angular_speed = 0.35;
  double line_length = 6.0;
  double line_speed = 0.6;
  double line_heading_deg = 0.0;
  double line_start_x = 0.0;
  double line_start_y = 0.0;
  double line_start_z = 1.0;
  double line_end_x = 0.0;
  double line_end_y = 0.0;
  double line_end_z = 1.0;
  double figure8_radius = 2.0;
  double noise_std = 0.0;
  bool yaw_align_velocity = true;

  nh.param("frame_id", frame_id, frame_id);
  nh.param("motion_mode", motion_mode, motion_mode);
  nh.param("publish_rate", publish_rate, publish_rate);
  nh.param("center_x", center_x, center_x);
  nh.param("center_y", center_y, center_y);
  nh.param("center_z", center_z, center_z);
  nh.param("radius", radius, radius);
  nh.param("angular_speed", angular_speed, angular_speed);
  nh.param("line_length", line_length, line_length);
  nh.param("line_speed", line_speed, line_speed);
  nh.param("line_heading_deg", line_heading_deg, line_heading_deg);
  nh.param("line_start_x", line_start_x, line_start_x);
  nh.param("line_start_y", line_start_y, line_start_y);
  nh.param("line_start_z", line_start_z, line_start_z);
  nh.param("line_end_x", line_end_x, line_end_x);
  nh.param("line_end_y", line_end_y, line_end_y);
  nh.param("line_end_z", line_end_z, line_end_z);
  nh.param("figure8_radius", figure8_radius, figure8_radius);
  nh.param("noise_std", noise_std, noise_std);
  nh.param("yaw_align_velocity", yaw_align_velocity, yaw_align_velocity);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/object_odom", 10);
  ros::Rate rate(std::max(1.0, publish_rate));

  const ros::Time t0 = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    const double t = std::max(0.0, (now - t0).toSec());

    Eigen::Vector3d pos(center_x, center_y, center_z);
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();

    if (motion_mode == "line")
    {
      const Eigen::Vector3d start_pt(line_start_x, line_start_y, line_start_z);
      const Eigen::Vector3d end_pt(line_end_x, line_end_y, line_end_z);
      const Eigen::Vector3d seg = end_pt - start_pt;
      const double seg_len = seg.norm();

      if (seg_len > 1.0e-3)
      {
        const Eigen::Vector3d dir = seg / seg_len;
        const double phase = std::fmod(std::max(0.0, line_speed) * t, 2.0 * seg_len);
        const bool forward = phase <= seg_len;
        const double dist = forward ? phase : (2.0 * seg_len - phase);
        pos = start_pt + dir * dist;
        vel = dir * (forward ? std::max(0.0, line_speed) : -std::max(0.0, line_speed));
      }
      else
      {
        const double omega = std::max(0.05, line_speed / std::max(0.2, line_length * 0.5));
        const double heading_rad = line_heading_deg * M_PI / 180.0;
        const Eigen::Vector2d dir(std::cos(heading_rad), std::sin(heading_rad));
        const double s = 0.5 * line_length * std::sin(omega * t);
        const double ds = 0.5 * line_length * omega * std::cos(omega * t);
        pos.x() = center_x + dir.x() * s;
        pos.y() = center_y + dir.y() * s;
        vel.x() = dir.x() * ds;
        vel.y() = dir.y() * ds;
      }
    }
    else if (motion_mode == "figure8")
    {
      const double omega = std::max(0.05, angular_speed);
      pos.x() = center_x + figure8_radius * std::sin(omega * t);
      pos.y() = center_y + 0.5 * figure8_radius * std::sin(2.0 * omega * t);
      vel.x() = figure8_radius * omega * std::cos(omega * t);
      vel.y() = figure8_radius * omega * std::cos(2.0 * omega * t);
    }
    else
    {
      const double omega = std::max(0.05, angular_speed);
      pos.x() = center_x + radius * std::cos(omega * t);
      pos.y() = center_y + radius * std::sin(omega * t);
      vel.x() = -radius * omega * std::sin(omega * t);
      vel.y() = radius * omega * std::cos(omega * t);
    }

    if (noise_std > 1.0e-6)
    {
      pos.x() += noise_std * std::sin(0.37 * t);
      pos.y() += noise_std * std::cos(0.53 * t);
      pos.z() += 0.5 * noise_std * std::sin(0.71 * t);
    }

    double yaw = 0.0;
    if (yaw_align_velocity && vel.head<2>().norm() > 1.0e-3)
    {
      yaw = std::atan2(vel.y(), vel.x());
    }
    const Eigen::Quaterniond q = yawToQuaternion(yaw);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = "object";
    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = pos.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();

    odom_pub.publish(odom);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
