#ifndef PLANNER_RUNTIME_REPLAN_TRIGGER_HPP_
#define PLANNER_RUNTIME_REPLAN_TRIGGER_HPP_

namespace ego_planner::runtime
{

class ReplanTrigger
{
public:
  bool allowReplan(double now, double last_replan_time, double min_interval) const
  {
    return (last_replan_time < 0.0) || (now - last_replan_time > min_interval);
  }
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_REPLAN_TRIGGER_HPP_

