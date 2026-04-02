#ifndef PLANNER_TASKS_TRACKING_TASK_HPP_
#define PLANNER_TASKS_TRACKING_TASK_HPP_

#include <core/task_spec.hpp>

namespace ego_planner::tasks
{

class TrackingTask
{
public:
  static core::TaskSpec build(const cost_functional::TrackingReference &reference,
                              const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &start_vel,
                              const Eigen::Vector3d &start_acc,
                              bool flag_poly_init,
                              bool flag_random_poly_traj,
                              bool force_plain);
};

} // namespace ego_planner::tasks

#endif // PLANNER_TASKS_TRACKING_TASK_HPP_

