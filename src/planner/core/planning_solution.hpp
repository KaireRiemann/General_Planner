#ifndef PLANNER_CORE_PLANNING_SOLUTION_HPP_
#define PLANNER_CORE_PLANNING_SOLUTION_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

#include <SpatialMap/SFCCommonTypes.hpp>
#include <traj_utils/minco_types.hpp>
#include <CostFunctionalManager/TrackingSemanticGuide.hpp>

namespace ego_planner::core
{

enum class ActiveSpaceModel : int;

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

  // Final solver-side initialization artifacts are carried in the solution so
  // downstream debug/visualization/runtime logic can rely on one authoritative
  // output instead of planner_manager-local temporaries.
  bool has_init_artifacts{false};
  ActiveSpaceModel active_space_model{static_cast<ActiveSpaceModel>(0)};
  std::string init_source;
  std::vector<Eigen::Vector3d> guide_path;
  std::vector<Eigen::Vector3d> dense_path;
  spatial_map::PolyhedraH corridor_hpolys;
  Eigen::VectorXi corridor_piece_idx;

  bool has_tracking_semantic_guide{false};
  cost_functional::TrackingSemanticGuide tracking_semantic_guide;
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PLANNING_SOLUTION_HPP_
