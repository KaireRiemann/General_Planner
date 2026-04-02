#ifndef PLANNER_CORE_PLANNING_CONTEXT_HPP_
#define PLANNER_CORE_PLANNING_CONTEXT_HPP_

#include <Eigen/Core>

#include <vector>
#include <memory>

#include <plan_env/grid_map.h>
#include <path_searching/jps_a_star.hpp>

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
  bool allow_warm_start{true};

  GridMap::Ptr grid_map;
  JPSAStar *jps_astar{nullptr};

  Eigen::Vector3d odom_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d global_goal{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_target{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_target_vel{Eigen::Vector3d::Zero()};

  double max_vel{1.0};
  double max_acc{1.0};
  double poly_piece_length{0.6};
  double guide_min_clearance{0.2};
  double sfc_progress{0.75};
  double sfc_range{0.8};
  double sfc_corridor_margin{0.05};

  WarmStartCache warm_start;
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PLANNING_CONTEXT_HPP_
