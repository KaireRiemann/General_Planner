#include <runtime/context_builder.hpp>

#include <ros/ros.h>

namespace ego_planner::runtime
{

core::PlanningContext ContextBuilder::build(int drone_id,
                                            bool map_ready,
                                            bool use_corridor,
                                            bool use_esdf,
                                            const Eigen::Vector3d &odom_pos,
                                            const Eigen::Vector3d &odom_vel,
                                            const Eigen::Vector3d &odom_acc,
                                            const Eigen::Vector3d &global_goal,
                                            const Eigen::Vector3d &local_target,
                                            const Eigen::Vector3d &local_target_vel,
                                            const LocalTrajData *local_traj) const
{
  core::PlanningContext context;
  context.drone_id = drone_id;
  context.now = ros::Time::now().toSec();
  context.map_ready = map_ready;
  context.use_corridor = use_corridor;
  context.use_esdf = use_esdf;
  context.odom_pos = odom_pos;
  context.odom_vel = odom_vel;
  context.odom_acc = odom_acc;
  context.global_goal = global_goal;
  context.local_target = local_target;
  context.local_target_vel = local_target_vel;

  if (local_traj != nullptr && local_traj->traj_id > 0)
  {
    context.warm_start.valid = true;
    context.warm_start.stamp = local_traj->start_time;
  }
  return context;
}

} // namespace ego_planner::runtime

