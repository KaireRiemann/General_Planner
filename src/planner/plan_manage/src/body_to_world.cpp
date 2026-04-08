#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <string>
#include <cmath>
#include <limits>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace ego_planner
{

Eigen::Matrix3d rpyToRotation(const double roll, const double pitch, const double yaw)
{
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

class BodyToWorldNode
{
public:
  explicit BodyToWorldNode(ros::NodeHandle &nh)
      : nh_(nh)
  {
    nh_.param<std::string>("odom_topic", odom_topic_, "/iris_0/mavros/vision_odom/odom");
    nh_.param<std::string>("cloud_in_topic", cloud_in_topic_, "/iris_0/velodyne_points");
    nh_.param<std::string>("cloud_out_topic", cloud_out_topic_, "/iris_0/points_world");
    nh_.param<std::string>("world_frame", world_frame_, "world");
    nh_.param("filter_near_enable", filter_near_enable_, true);
    nh_.param("filter_near_radius", filter_near_radius_, 0.45);
    nh_.param("filter_ground_enable", filter_ground_enable_, true);
    nh_.param("filter_ground_z", filter_ground_z_, 0.05);
    nh_.param("source_in_body_frame", source_in_body_frame_, true);
    nh_.param("source_offset_x", source_offset_(0), 0.0);
    nh_.param("source_offset_y", source_offset_(1), 0.0);
    nh_.param("source_offset_z", source_offset_(2), 0.0);

    double source_roll = 0.0;
    double source_pitch = 0.0;
    double source_yaw = 0.0;
    nh_.param("source_roll", source_roll, 0.0);
    nh_.param("source_pitch", source_pitch, 0.0);
    nh_.param("source_yaw", source_yaw, 0.0);
    source_rot_ = rpyToRotation(source_roll, source_pitch, source_yaw);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &BodyToWorldNode::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_in_topic_, 10, &BodyToWorldNode::cloudCallback, this);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_out_topic_, 5);

    filter_near_radius_ = std::max(0.0, filter_near_radius_);

    ROS_INFO("body_to_world ready. odom=%s cloud_in=%s cloud_out=%s source_in_body=%s near_filter=%s(%.2fm) ground_filter=%s(z<=%.2f)",
             odom_topic_.c_str(),
             cloud_in_topic_.c_str(),
             cloud_out_topic_.c_str(),
             source_in_body_frame_ ? "true" : "false",
             filter_near_enable_ ? "on" : "off",
             filter_near_radius_,
             filter_ground_enable_ ? "on" : "off",
             filter_ground_z_);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!has_odom_)
      {
        ROS_WARN_THROTTLE(1.0, "body_to_world waiting odom: %s", odom_topic_.c_str());
        return;
      }
      odom = latest_odom_;
    }

    const Eigen::Quaterniond q_wb(odom.pose.pose.orientation.w,
                                  odom.pose.pose.orientation.x,
                                  odom.pose.pose.orientation.y,
                                  odom.pose.pose.orientation.z);
    if (!q_wb.norm())
    {
      ROS_WARN_THROTTLE(1.0, "body_to_world invalid odom quaternion.");
      return;
    }
    const Eigen::Matrix3d rot_wb = q_wb.normalized().toRotationMatrix();
    const Eigen::Vector3d pos_wb(odom.pose.pose.position.x,
                                 odom.pose.pose.position.y,
                                 odom.pose.pose.position.z);

    sensor_msgs::PointCloud2 out = *msg;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = world_frame_;

    sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    const double near_radius_sq = filter_near_radius_ * filter_near_radius_;

    for (; in_x != in_x.end();
         ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z)
    {
      const float sx = *in_x;
      const float sy = *in_y;
      const float sz = *in_z;
      if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz))
      {
        *out_x = sx;
        *out_y = sy;
        *out_z = sz;
        continue;
      }

      Eigen::Vector3d p_source(sx, sy, sz);
      Eigen::Vector3d p_body = p_source;
      if (!source_in_body_frame_)
      {
        p_body = source_rot_ * p_source + source_offset_;
      }
      const Eigen::Vector3d p_world = rot_wb * p_body + pos_wb;

      const bool reject_near =
          filter_near_enable_ && p_body.squaredNorm() <= near_radius_sq;
      const bool reject_ground =
          filter_ground_enable_ && p_world.z() <= filter_ground_z_;

      if (reject_near || reject_ground)
      {
        *out_x = nan_f;
        *out_y = nan_f;
        *out_z = nan_f;
      }
      else
      {
        *out_x = static_cast<float>(p_world.x());
        *out_y = static_cast<float>(p_world.y());
        *out_z = static_cast<float>(p_world.z());
      }
    }

    cloud_pub_.publish(out);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  std::mutex mtx_;
  nav_msgs::Odometry latest_odom_;
  bool has_odom_{false};

  std::string odom_topic_;
  std::string cloud_in_topic_;
  std::string cloud_out_topic_;
  std::string world_frame_;

  bool source_in_body_frame_{true};
  bool filter_near_enable_{true};
  bool filter_ground_enable_{true};
  double filter_near_radius_{0.45};
  double filter_ground_z_{0.05};
  Eigen::Vector3d source_offset_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d source_rot_{Eigen::Matrix3d::Identity()};
};

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "body_to_world");
  ros::NodeHandle nh("~");
  ego_planner::BodyToWorldNode node(nh);
  ros::spin();
  return 0;
}
