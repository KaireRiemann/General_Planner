#ifndef PLANNER_CORE_PLANNING_SOLUTION_HPP_
#define PLANNER_CORE_PLANNING_SOLUTION_HPP_

#include <string>
#include <vector>

#include <traj_utils/minco_types.hpp>
#include <CostFunctionalManager/TrackingSemanticGuide.hpp>

namespace ego_planner::core
{

struct PlanningSolution
{
  bool success{false};
  std::string message;
  bool used_legacy_adapter{true};
  double solve_time_ms{0.0};

  bool touch_goal{false};
  MINCOTraj3D trajectory;

  std::vector<double> yaw_time;
  std::vector<double> yaw_ref;
  bool has_yaw_ref{false};

  bool has_tracking_semantic_guide{false};
  cost_functional::TrackingSemanticGuide tracking_semantic_guide;
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PLANNING_SOLUTION_HPP_

