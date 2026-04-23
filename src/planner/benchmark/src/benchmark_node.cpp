#include <algorithm>
#include <array>
#include <cmath>
#include <random>
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
    pnh_.param("max_speed", max_speed_, 1.0);
    pnh_.param("max_cnt", max_cnt_, 10);

    odom_sub_ = nh_.subscribe(odom_topic_, 1, &BenchmarkNode::odomCallback, this);
    goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>(goal_topic_, 1, true);
    max_speed_ = std::max(0.01, max_speed_);

    ros::Duration(1.0).sleep(); // wait for publishers and subscribers to be ready
    ROS_INFO("benchmark_node ready: map_size=[%.2f, %.2f, %.2f], max_speed=%.2f, odom_topic=%s, goal_topic=%s",
             map_size_x_, map_size_y_, map_size_z_, max_speed_, odom_topic_.c_str(), goal_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    current_position_[0] = msg->pose.pose.position.x;
    current_position_[1] = msg->pose.pose.position.y;
    current_position_[2] = msg->pose.pose.position.z;
    if (have_odom_)
    {
      const double dx = current_position_[0] - last_position_[0];
      const double dy = current_position_[1] - last_position_[1];
      const double dz = current_position_[2] - last_position_[2];
      path_length_ += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    last_position_ = current_position_;
    have_odom_ = true;

    if (!have_published_)
    {
      publishGoal();
      have_published_ = true;
    }

    if (!have_arrived_)
    {
      const double dx = current_goal_[0] - current_position_[0];
      const double dy = current_goal_[1] - current_position_[1];
      const double dz = current_goal_[2] - current_position_[2];
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      double arrival_time = (ros::Time::now() - start_time_).toSec();
      if (dist < 1)
      {
        have_arrived_ = true;
        ROS_INFO("arrived at goal: [%.2f, %.2f, %.2f], dist=%.2f m, path_length=%.2f m, time=%.2f s",
                 current_goal_[0], current_goal_[1], current_goal_[2], goal_length_, path_length_, arrival_time);
        if (cur_cnt_ < max_cnt_)
        {
          ros::Duration(0.5).sleep(); // add some delay
          publishGoal();
        }
        else
        {
          ROS_INFO("benchmark finished: total_cnt=%d", cur_cnt_);
          ros::shutdown();
        }
      }
      else if (arrival_time > min_arrival_time_)
      {
        ROS_WARN("goal timeout: [%.2f, %.2f, %.2f], dist=%.2f m, path_length=%.2f m, time=%.2f s",
                 current_goal_[0], current_goal_[1], current_goal_[2], goal_length_, path_length_, arrival_time);
      }
    }
  }

  std::array<double, 3> pickGoal() const
  {
    const double half_x = 0.5 * map_size_x_;
    const double half_y = 0.5 * map_size_y_;

    std::uniform_real_distribution<double> dist_x(-half_x, half_x);
    std::uniform_real_distribution<double> dist_y(-half_y, half_y);
    std::uniform_real_distribution<double> dist_z(0.5, std::max(2.0, map_size_z_));

    return {{dist_x(rng_), dist_y(rng_), dist_z(rng_)}};
  }

  void publishGoal(void)
  {
    if (!have_odom_ || !have_arrived_)
    {
      return;
    }
    current_goal_ = pickGoal();
    const double dx = current_goal_[0] - current_position_[0];
    const double dy = current_goal_[1] - current_position_[1];
    const double dz = current_goal_[2] - current_position_[2];
    goal_length_ = std::sqrt(dx * dx + dy * dy + dz * dz);
    min_arrival_time_ = goal_length_ / max_speed_ * 2.0; // add some margin
    
    quadrotor_msgs::GoalSet goal_msg;
    goal_msg.drone_id = 0;
    goal_msg.goal[0] = static_cast<float>(current_goal_[0]);
    goal_msg.goal[1] = static_cast<float>(current_goal_[1]);
    goal_msg.goal[2] = static_cast<float>(current_goal_[2]);
    
    goal_pub_.publish(goal_msg);
    cur_cnt_ += 1;
    path_length_ = 0.0;
    have_arrived_ = false;
    start_time_ = ros::Time::now();
    ROS_INFO("publish goal: [%.2f, %.2f, %.2f], dist=%.2f m, t_min=%.2f s, cur_cnt: %d, (v_max=%.2f m/s)",
             current_goal_[0], current_goal_[1], current_goal_[2], goal_length_, min_arrival_time_, cur_cnt_, max_speed_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher goal_pub_;
  ros::Timer publish_timer_;

  double map_size_x_{20.0};
  double map_size_y_{20.0};
  double map_size_z_{5.0};
  double max_speed_{1.0};
  double min_arrival_time_{0.0};
  double path_length_{0.0};
  double goal_length_{0.0};
  std::array<double, 3> last_position_{{0.0, 0.0, 0.0}};
  int max_cnt_{10};
  int cur_cnt_{0};
  ros::Time start_time_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::array<double, 3> current_position_{{0.0, 0.0, 0.0}};
  std::array<double, 3> current_goal_{{0.0, 0.0, 0.0}};
  mutable std::mt19937 rng_{std::random_device{}()};
  bool have_odom_{false};
  bool have_arrived_{true};
  bool have_published_{false};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark_node");
  ros::NodeHandle nh;

  BenchmarkNode node(nh);
  ros::spin();

  return 0;
}