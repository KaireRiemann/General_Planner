#ifndef PLANNER_TASKS_STATE_TO_STATE_TASK_HPP_
#define PLANNER_TASKS_STATE_TO_STATE_TASK_HPP_

#include <core/task_spec.hpp>

namespace ego_planner::tasks
{

class StateToStateTask
{
public:
  static core::TaskSpec build(const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &start_vel,
                              const Eigen::Vector3d &start_acc,
                              const Eigen::Vector3d &goal_pt,
                              const Eigen::Vector3d &goal_vel,
                              bool touch_goal,
                              bool flag_poly_init,
                              bool flag_random_poly_traj,
                              bool force_plain,
                              bool prefer_corridor,
                              bool prefer_esdf);
};

} // namespace ego_planner::tasks

#endif // PLANNER_TASKS_STATE_TO_STATE_TASK_HPP_
