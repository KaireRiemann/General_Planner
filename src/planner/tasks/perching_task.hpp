#ifndef PLANNER_TASKS_PERCHING_TASK_HPP_
#define PLANNER_TASKS_PERCHING_TASK_HPP_

#include <core/task_spec.hpp>

namespace ego_planner::tasks
{

class PerchingTask
{
public:
  static core::TaskSpec build(const Eigen::Vector3d &start_pt,
                              const Eigen::Vector3d &start_vel,
                              const Eigen::Vector3d &start_acc,
                              const Eigen::Vector3d &contact_pt,
                              const Eigen::Vector3d &contact_vel,
                              bool force_plain);
};

} // namespace ego_planner::tasks

#endif // PLANNER_TASKS_PERCHING_TASK_HPP_

