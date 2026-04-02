#ifndef PLANNER_CORE_RUNTIME_POLICY_HPP_
#define PLANNER_CORE_RUNTIME_POLICY_HPP_

namespace ego_planner::core
{

struct RuntimePolicy
{
  bool touch_goal{false};
  bool allow_replan{true};
  bool allow_warm_start{true};
  bool flag_poly_init{false};
  bool flag_random_poly_traj{false};
  bool preserve_legacy_compatibility{true};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_RUNTIME_POLICY_HPP_
