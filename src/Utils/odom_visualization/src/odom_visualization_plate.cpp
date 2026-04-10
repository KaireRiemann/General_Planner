#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>

#include <cmath>

double plate_height = 1.2;
double plate_width = 1.6;
double plate_thickness = 0.035;
double color_r = 0.1;
double color_g = 0.45;
double color_b = 1.0;
double color_a = 0.65;
double normal_length = 0.6;

ros::Publisher plate_pub;
ros::Publisher normal_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!msg)
  {
    return;
  }

  visualization_msgs::Marker plate;
  plate.header = msg->header;
  if (plate.header.frame_id.empty())
  {
    plate.header.frame_id = "world";
  }
  plate.ns = "perching_plate";
  plate.id = 0;
  plate.type = visualization_msgs::Marker::CUBE;
  plate.action = visualization_msgs::Marker::ADD;
  plate.pose = msg->pose.pose;
  plate.scale.x = plate_height;
  plate.scale.y = plate_width;
  plate.scale.z = plate_thickness;
  plate.color.r = color_r;
  plate.color.g = color_g;
  plate.color.b = color_b;
  plate.color.a = color_a;
  plate.lifetime = ros::Duration(0.25);
  plate_pub.publish(plate);

  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                       msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1.0e-6)
  {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  const Eigen::Vector3d p(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
  const Eigen::Vector3d n = q.toRotationMatrix().col(2).normalized();

  visualization_msgs::Marker normal;
  normal.header = plate.header;
  normal.ns = "perching_plate_normal";
  normal.id = 1;
  normal.type = visualization_msgs::Marker::ARROW;
  normal.action = visualization_msgs::Marker::ADD;
  normal.points.resize(2);
  normal.points[0].x = p.x();
  normal.points[0].y = p.y();
  normal.points[0].z = p.z();
  normal.points[1].x = p.x() + normal_length * n.x();
  normal.points[1].y = p.y() + normal_length * n.y();
  normal.points[1].z = p.z() + normal_length * n.z();
  normal.scale.x = 0.035;
  normal.scale.y = 0.09;
  normal.scale.z = 0.09;
  normal.color.r = 1.0;
  normal.color.g = 0.85;
  normal.color.b = 0.05;
  normal.color.a = 0.9;
  normal.lifetime = ros::Duration(0.25);
  normal_pub.publish(normal);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_visualization_plate");
  ros::NodeHandle nh("~");

  nh.param("height", plate_height, plate_height);
  nh.param("width", plate_width, plate_width);
  nh.param("thickness", plate_thickness, plate_thickness);
  nh.param("color/r", color_r, color_r);
  nh.param("color/g", color_g, color_g);
  nh.param("color/b", color_b, color_b);
  nh.param("color/a", color_a, color_a);
  nh.param("normal_length", normal_length, normal_length);

  ros::Subscriber odom_sub = nh.subscribe("odom", 20, odomCallback);
  plate_pub = nh.advertise<visualization_msgs::Marker>("polygon", 20, true);
  normal_pub = nh.advertise<visualization_msgs::Marker>("normal", 20, true);

  ros::spin();
  return 0;
}
