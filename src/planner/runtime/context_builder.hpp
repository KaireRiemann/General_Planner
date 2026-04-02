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
                              bool has_grid_map,
                              bool has_esdf,
                              bool has_jps,
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
                              int guide_sparse_min_inner,
                              int guide_sparse_max_inner,
                              double guide_turn_angle_deg,
                              double sfc_progress,
                              double sfc_range,
                              double sfc_corridor_margin,
                              const LocalTrajData *local_traj) const;
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_CONTEXT_BUILDER_HPP_
