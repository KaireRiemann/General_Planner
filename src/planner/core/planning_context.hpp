#ifndef PLANNER_CORE_PLANNING_CONTEXT_HPP_
#define PLANNER_CORE_PLANNING_CONTEXT_HPP_

#include <Eigen/Core>

#include <vector>

namespace ego_planner::core
{

struct WarmStartCache
{
  bool valid{false};
  std::vector<Eigen::Vector3d> anchor_points;
  Eigen::VectorXd durations;
  double stamp{0.0};
};

struct PlanningContext
{
  int drone_id{-1};
  double now{0.0};
  bool map_ready{false};
  bool use_corridor{false};
  bool use_esdf{false};

  Eigen::Vector3d odom_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d global_goal{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_target{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_target_vel{Eigen::Vector3d::Zero()};

  WarmStartCache warm_start;
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PLANNING_CONTEXT_HPP_

