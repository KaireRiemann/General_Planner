#ifndef PLANNER_CORE_FEASIBLE_SET_SPEC_HPP_
#define PLANNER_CORE_FEASIBLE_SET_SPEC_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

#include <SpatialMap/SFCCommonTypes.hpp>
#include <CostFunctionalManager/TrackingSemanticGuide.hpp>

namespace ego_planner::core
{

enum class FeasibleSetType
{
  NONE = 0,
  CORRIDOR_POLYTOPE,
  VISIBLE_REGION_TUBE,
  TERMINAL_MANIFOLD
};

struct FeasibleSetSpec
{
  FeasibleSetType type{FeasibleSetType::NONE};
  std::string label;

  spatial_map::PolyhedraH corridor;
  std::vector<cost_functional::VisibleFanRegion> visible_regions;

  Eigen::VectorXd manifold_params;
  bool enabled{false};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_FEASIBLE_SET_SPEC_HPP_

