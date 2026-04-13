#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace
{

Eigen::Quaterniond quaternionFromAxisAngle(Eigen::Vector3d axis, const double theta)
{
  if (!axis.allFinite() || axis.norm() < 1.0e-6)
  {
    axis = Eigen::Vector3d::UnitY();
  }
  axis.normalize();
  return Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
}

Eigen::Quaterniond quaternionFromRpy(const double roll, const double pitch, const double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

Eigen::Quaterniond normalized(Eigen::Quaterniond q)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perching_target_sim");
  ros::NodeHandle nh("~");

  std::string frame_id = "world";
  std::string motion_mode = "line";
  std::string orientation_mode = "rpy";
  std::string odom_topic = "/perching/target_odom";
  std::string trigger_topic = "/land_triger";
  std::string reset_topic = "/perching/reset";
  std::string trigger_mode = "once";
  double publish_rate = 50.0;
  double px = 0.5;
  double py = 0.0;
  double pz = 2.0;
  double vx = 1.0;
  double vy = 0.0;
  double vz = 0.0;
  double axis_x = 0.0;
  double axis_y = 1.0;
  double axis_z = 0.0;
  double theta = -1.5708;
  double roll = 0.0;
  double pitch = -1.5708;
  double yaw = 0.0;
  bool attitude_sine_enable = false;
  double attitude_sine_amplitude = 0.0;
  double attitude_sine_rate = 0.5;
  double line_start_x = -2.0;
  double line_start_y = 0.0;
  double line_start_z = 2.0;
  double line_end_x = 4.0;
  double line_end_y = 0.0;
  double line_end_z = 2.0;
  double line_speed = 0.9;
  bool publish_trigger = true;
  double trigger_delay = 1.0;
  double trigger_period = 8.0;

  nh.param("frame_id", frame_id, frame_id);
  nh.param("motion_mode", motion_mode, motion_mode);
  nh.param("orientation_mode", orientation_mode, orientation_mode);
  nh.param("odom_topic", odom_topic, odom_topic);
  nh.param("trigger_topic", trigger_topic, trigger_topic);
  nh.param("reset_topic", reset_topic, reset_topic);
  nh.param("trigger_mode", trigger_mode, trigger_mode);
  nh.param("publish_rate", publish_rate, publish_rate);
  nh.param("perching_px", px, px);
  nh.param("perching_py", py, py);
  nh.param("perching_pz", pz, pz);
  nh.param("perching_vx", vx, vx);
  nh.param("perching_vy", vy, vy);
  nh.param("perching_vz", vz, vz);
  nh.param("perching_axis_x", axis_x, axis_x);
  nh.param("perching_axis_y", axis_y, axis_y);
  nh.param("perching_axis_z", axis_z, axis_z);
  nh.param("perching_theta", theta, theta);
  nh.param("plate_roll", roll, roll);
  nh.param("plate_pitch", pitch, pitch);
  nh.param("plate_yaw", yaw, yaw);
  nh.param("attitude_sine_enable", attitude_sine_enable, attitude_sine_enable);
  nh.param("attitude_sine_amplitude", attitude_sine_amplitude, attitude_sine_amplitude);
  nh.param("attitude_sine_rate", attitude_sine_rate, attitude_sine_rate);
  nh.param("line_start_x", line_start_x, line_start_x);
  nh.param("line_start_y", line_start_y, line_start_y);
  nh.param("line_start_z", line_start_z, line_start_z);
  nh.param("line_end_x", line_end_x, line_end_x);
  nh.param("line_end_y", line_end_y, line_end_y);
  nh.param("line_end_z", line_end_z, line_end_z);
  nh.param("line_speed", line_speed, line_speed);
  nh.param("publish_trigger", publish_trigger, publish_trigger);
  nh.param("trigger_delay", trigger_delay, trigger_delay);
  nh.param("trigger_period", trigger_period, trigger_period);

  Eigen::Vector3d attitude_axis(axis_x, axis_y, axis_z);
  if (!attitude_axis.allFinite() || attitude_axis.norm() < 1.0e-6)
  {
    attitude_axis = Eigen::Vector3d::UnitY();
  }
  attitude_axis.normalize();

  ros::Time round_start_time = ros::Time::now();
  bool trigger_sent = false;
  int round_id = 0;

  auto reset_round = [&round_start_time, &trigger_sent, &round_id]()
  {
    round_start_time = ros::Time::now();
    trigger_sent = false;
    round_id++;
    ROS_INFO("Perching target simulator reset round %d.", round_id);
  };

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
  ros::Publisher trigger_pub = nh.advertise<std_msgs::Empty>(trigger_topic, 1, true);
  ros::Subscriber reset_sub = nh.subscribe<std_msgs::Empty>(
      reset_topic,
      1,
      [&reset_round](const std_msgs::EmptyConstPtr &)
      {
        reset_round();
      });
  ros::Rate rate(std::max(1.0, publish_rate));

  ROS_INFO("Perching target simulator: odom=%s trigger=%s reset=%s lock=%s mode=%s orientation=%s trigger_mode=%s",
           odom_topic.c_str(),
           trigger_topic.c_str(),
           reset_topic.c_str(),
           motion_mode.c_str(),
           orientation_mode.c_str(),
           trigger_mode.c_str());

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    double t = std::max(0.0, (now - round_start_time).toSec());

    if (trigger_mode == "periodic" &&
        trigger_period > trigger_delay + 0.1 &&
        t >= trigger_period)
    {
      reset_round();
      t = 0.0;
    }

    Eigen::Vector3d pos(px, py, pz);
    Eigen::Vector3d vel(vx, vy, vz);

    if (motion_mode == "line")
    {
      const Eigen::Vector3d start_pt(line_start_x, line_start_y, line_start_z);
      const Eigen::Vector3d end_pt(line_end_x, line_end_y, line_end_z);
      const Eigen::Vector3d seg = end_pt - start_pt;
      const double seg_len = seg.norm();
      if (seg_len > 1.0e-6)
      {
        const Eigen::Vector3d dir = seg / seg_len;
        const double speed = std::max(0.0, line_speed);
        const double phase = std::fmod(speed * t, 2.0 * seg_len);
        const bool forward = phase <= seg_len;
        const double dist = forward ? phase : (2.0 * seg_len - phase);
        pos = start_pt + dir * dist;
        vel = dir * (forward ? speed : -speed);
      }
      else
      {
        pos = start_pt;
        vel.setZero();
      }
    }
    else if (motion_mode == "static")
    {
      vel.setZero();
    }
    else
    {
      pos += vel * t;
    }

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    const double dynamic_theta =
        theta + (attitude_sine_enable
                     ? attitude_sine_amplitude * std::sin(attitude_sine_rate * t)
                     : 0.0);
    if (orientation_mode == "axis_angle")
    {
      q = quaternionFromAxisAngle(attitude_axis, dynamic_theta);
    }
    else
    {
      q = quaternionFromRpy(roll,
                            pitch + (attitude_sine_enable
                                         ? attitude_sine_amplitude * std::sin(attitude_sine_rate * t)
                                         : 0.0),
                            yaw);
    }
    q = normalized(q);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = "perching_plate";
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

    if (publish_trigger &&
        trigger_mode != "manual" &&
        !trigger_sent &&
        t >= trigger_delay)
    {
      trigger_pub.publish(std_msgs::Empty{});
      trigger_sent = true;
      ROS_INFO("Perching target simulator published trigger on %s for round %d.",
               trigger_topic.c_str(),
               round_id);
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
