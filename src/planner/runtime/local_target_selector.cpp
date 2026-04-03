#include <runtime/local_target_selector.hpp>

#include <algorithm>

namespace ego_planner::runtime
{

bool LocalTargetSelector::peekLocalTarget(const TrajContainer &traj_container,
                                          double planning_horizon,
                                          double max_vel,
                                          const Eigen::Vector3d &start_pt,
                                          const Eigen::Vector3d &global_end_pt,
                                          LocalTargetSelection &selection) const
{
  selection = LocalTargetSelection{};

  const auto &global_traj = traj_container.global_traj;
  if (global_traj.duration <= 1.0e-6)
  {
    return false;
  }

  const double global_end_time = global_traj.global_start_time + global_traj.duration;
  const double current_glb_t = std::max(global_traj.global_start_time, global_traj.glb_t_of_lc_tgt);
  const double t_step =
      std::max(1.0e-3, planning_horizon / 20.0 / std::max(max_vel, 0.1));

  double target_glb_t = current_glb_t;
  bool found_horizon_target = false;

  for (double t = current_glb_t; t < global_end_time; t += t_step)
  {
    const double local_t = t - global_traj.global_start_time;
    const Eigen::Vector3d pos_t = global_traj.traj.evaluate(local_t, 0);
    if ((pos_t - start_pt).norm() >= planning_horizon)
    {
      selection.local_target_pos = pos_t;
      target_glb_t = t;
      found_horizon_target = true;
      break;
    }
  }

  if (!found_horizon_target)
  {
    selection.local_target_pos = global_end_pt;
    target_glb_t = global_end_time;
    selection.touch_goal = true;
  }

  if (selection.touch_goal)
  {
    selection.local_target_vel = Eigen::Vector3d::Zero();
  }
  else
  {
    const double local_t = std::max(0.0, target_glb_t - global_traj.global_start_time);
    selection.local_target_vel = global_traj.traj.evaluate(local_t, 1);
  }

  selection.previous_glb_t_of_lc_tgt = global_traj.glb_t_of_lc_tgt;
  selection.next_glb_t_of_lc_tgt = target_glb_t;
  selection.valid = true;
  return true;
}

bool LocalTargetSelector::commitLocalTarget(TrajContainer &traj_container,
                                            const LocalTargetSelection &selection) const
{
  if (!selection.valid)
  {
    return false;
  }

  traj_container.global_traj.last_glb_t_of_lc_tgt = selection.previous_glb_t_of_lc_tgt;
  traj_container.global_traj.glb_t_of_lc_tgt = selection.next_glb_t_of_lc_tgt;
  return true;
}

} // namespace ego_planner::runtime
