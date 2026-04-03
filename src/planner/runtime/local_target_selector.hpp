#ifndef PLANNER_RUNTIME_LOCAL_TARGET_SELECTOR_HPP_
#define PLANNER_RUNTIME_LOCAL_TARGET_SELECTOR_HPP_

#include <Eigen/Core>

#include <traj_utils/plan_container.hpp>

namespace ego_planner::runtime
{

struct LocalTargetSelection
{
  bool valid{false};
  Eigen::Vector3d local_target_pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_target_vel{Eigen::Vector3d::Zero()};
  bool touch_goal{false};
  double next_glb_t_of_lc_tgt{0.0};
  double previous_glb_t_of_lc_tgt{-1.0};
};

class LocalTargetSelector
{
public:
  bool peekLocalTarget(const TrajContainer &traj_container,
                       double planning_horizon,
                       double max_vel,
                       const Eigen::Vector3d &start_pt,
                       const Eigen::Vector3d &global_end_pt,
                       LocalTargetSelection &selection) const;

  bool commitLocalTarget(TrajContainer &traj_container,
                         const LocalTargetSelection &selection) const;
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_LOCAL_TARGET_SELECTOR_HPP_
