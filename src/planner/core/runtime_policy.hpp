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

  // Successor planning policy is task/runtime-level policy data.
  // FSM may execute it, but it should not live purely as ad hoc local logic.
  bool enable_keep_current{true};
  bool enable_successor_planning{true};
  double keep_lookahead{0.8};
  double min_rest_time{0.8};
  double successor_lead_time{0.8};
  double successor_horizon_ratio{0.65};
  double successor_target_shift_thresh{0.35};
  double successor_near_goal_hold_radius{0.5};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_RUNTIME_POLICY_HPP_
