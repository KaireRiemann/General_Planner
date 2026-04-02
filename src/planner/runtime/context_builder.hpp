#ifndef PLANNER_RUNTIME_CONTEXT_BUILDER_HPP_
#define PLANNER_RUNTIME_CONTEXT_BUILDER_HPP_

#include <core/planning_context.hpp>
#include <traj_utils/plan_container.hpp>

namespace ego_planner::runtime
{

class ContextBuilder
{
public:
  core::PlanningContext build(int drone_id,
                              bool map_ready,
                              bool use_corridor,
                              bool use_esdf,
                              const Eigen::Vector3d &odom_pos,
                              const Eigen::Vector3d &odom_vel,
                              const Eigen::Vector3d &odom_acc,
                              const Eigen::Vector3d &global_goal,
                              const Eigen::Vector3d &local_target,
                              const Eigen::Vector3d &local_target_vel,
                              const LocalTrajData *local_traj) const;
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_CONTEXT_BUILDER_HPP_

