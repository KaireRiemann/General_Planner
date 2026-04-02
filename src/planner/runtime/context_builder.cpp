#include <runtime/context_builder.hpp>

#include <ros/ros.h>
#include <algorithm>

namespace ego_planner::runtime
{

core::PlanningContext ContextBuilder::build(int drone_id,
                                            bool map_ready,
                                            bool use_corridor,
                                            bool use_esdf,
                                            const GridMap::Ptr &grid_map,
                                            JPSAStar *jps_astar,
                                            const Eigen::Vector3d &odom_pos,
                                            const Eigen::Vector3d &odom_vel,
                                            const Eigen::Vector3d &odom_acc,
                                            const Eigen::Vector3d &global_goal,
                                            const Eigen::Vector3d &local_target,
                                            const Eigen::Vector3d &local_target_vel,
                                            double max_vel,
                                            double max_acc,
                                            double poly_piece_length,
                                            double guide_min_clearance,
                                            double sfc_progress,
                                            double sfc_range,
                                            double sfc_corridor_margin,
                                            const LocalTrajData *local_traj) const
{
  core::PlanningContext context;
  context.drone_id = drone_id;
  context.now = ros::Time::now().toSec();
  context.map_ready = map_ready;
  context.use_corridor = use_corridor;
  context.use_esdf = use_esdf;
  context.grid_map = grid_map;
  context.jps_astar = jps_astar;
  context.odom_pos = odom_pos;
  context.odom_vel = odom_vel;
  context.odom_acc = odom_acc;
  context.global_goal = global_goal;
  context.local_target = local_target;
  context.local_target_vel = local_target_vel;
  context.max_vel = std::max(0.1, max_vel);
  context.max_acc = std::max(0.1, max_acc);
  context.poly_piece_length = std::max(0.1, poly_piece_length);
  context.guide_min_clearance = std::max(0.0, guide_min_clearance);
  context.sfc_progress = std::max(0.1, sfc_progress);
  context.sfc_range = std::max(0.1, sfc_range);
  context.sfc_corridor_margin = std::max(0.0, sfc_corridor_margin);

  if (context.allow_warm_start && local_traj != nullptr && local_traj->traj_id > 0)
  {
    context.warm_start.valid = true;
    context.warm_start.stamp = local_traj->start_time;
  }
  return context;
}

} // namespace ego_planner::runtime
