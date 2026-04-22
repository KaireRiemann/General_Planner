#include <algorithm>
#include <array>
#include <string>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/GoalSet.h>
#include <ros/ros.h>

class BenchmarkNode
{
public:
  explicit BenchmarkNode(ros::NodeHandle &nh) : nh_(nh), pnh_("~")
  {
    pnh_.param("map_size_x", map_size_x_, 20.0);
    pnh_.param("map_size_y", map_size_y_, 20.0);
    pnh_.param("map_size_z", map_size_z_, 5.0);
    pnh_.param("odom_topic", odom_topic_, std::string("/uav/odom"));
    pnh_.param("goal_topic", goal_topic_, std::string("/benchmark/goal"));
    pnh_.param("publish_rate", publish_rate_, 1.0);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &BenchmarkNode::odomCallback, this);
    goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>(goal_topic_, 1, true);

    const double rate_hz = std::max(0.2, publish_rate_);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz), &BenchmarkNode::publishGoalTimer, this);

    ROS_INFO("benchmark_node ready: map_size=[%.2f, %.2f, %.2f], odom_topic=%s, goal_topic=%s",
             map_size_x_, map_size_y_, map_size_z_, odom_topic_.c_str(), goal_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    current_position_[0] = msg->pose.pose.position.x;
    current_position_[1] = msg->pose.pose.position.y;
    current_position_[2] = msg->pose.pose.position.z;
    have_odom_ = true;
  }

  std::array<double, 3> pickGoal() const
  {
    const double half_x = 0.5 * map_size_x_;
    const double half_y = 0.5 * map_size_y_;
    const double half_z = 0.5 * map_size_z_;

    const double margin_x = std::min(2.0, std::max(0.5, 0.15 * map_size_x_));
    const double margin_y = std::min(2.0, std::max(0.5, 0.15 * map_size_y_));
    const double margin_z = std::min(1.0, std::max(0.3, 0.10 * map_size_z_));

    const double goal_x = std::max(0.0, half_x - margin_x);
    const double goal_y = std::max(0.0, half_y - margin_y);
    const double goal_z = std::max(1.0, half_z - margin_z);

    const std::array<std::array<double, 3>, 4> candidates = {{
        {{-goal_x, -goal_y, goal_z}},
        {{-goal_x, goal_y, goal_z}},
        {{goal_x, -goal_y, goal_z}},
        {{goal_x, goal_y, goal_z}},
    }};

    std::array<double, 3> best_goal = candidates.front();
    double best_dist_sq = -1.0;
    for (const auto &candidate : candidates)
    {
      const double dx = candidate[0] - current_position_[0];
      const double dy = candidate[1] - current_position_[1];
      const double dz = candidate[2] - current_position_[2];
      const double dist_sq = dx * dx + dy * dy + dz * dz;
      if (dist_sq > best_dist_sq)
      {
        best_dist_sq = dist_sq;
        best_goal = candidate;
      }
    }

    return best_goal;
  }

  void publishGoalTimer(const ros::TimerEvent &)
  {
    if (!have_odom_)
    {
      return;
    }

    const std::array<double, 3> goal = pickGoal();

    quadrotor_msgs::GoalSet goal_msg;
    goal_msg.drone_id = 0;
    goal_msg.goal[0] = static_cast<float>(goal[0]);
    goal_msg.goal[1] = static_cast<float>(goal[1]);
    goal_msg.goal[2] = static_cast<float>(goal[2]);

    goal_pub_.publish(goal_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher goal_pub_;
  ros::Timer publish_timer_;

  double map_size_x_{20.0};
  double map_size_y_{20.0};
  double map_size_z_{5.0};
  double publish_rate_{1.0};
  std::string odom_topic_;
  std::string goal_topic_;
  std::array<double, 3> current_position_{{0.0, 0.0, 0.0}};
  bool have_odom_{false};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark_node");
  ros::NodeHandle nh;

  BenchmarkNode node(nh);
  ros::spin();

  return 0;
}