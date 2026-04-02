#ifndef PLANNER_RUNTIME_PLAN_MONITOR_HPP_
#define PLANNER_RUNTIME_PLAN_MONITOR_HPP_

#include <traj_utils/plan_container.hpp>

namespace ego_planner::runtime
{

class PlanMonitor
{
public:
  bool hasValidLocalTraj(const LocalTrajData &local_traj) const
  {
    return local_traj.traj_id > 0 && local_traj.duration > 1.0e-3;
  }
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_PLAN_MONITOR_HPP_

