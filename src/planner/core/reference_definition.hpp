#ifndef PLANNER_CORE_REFERENCE_DEFINITION_HPP_
#define PLANNER_CORE_REFERENCE_DEFINITION_HPP_

#include <Eigen/Core>

#include <CostFunctionalManager/TrackingTypes.hpp>

#include <string>
#include <vector>

namespace ego_planner::core
{

enum class ReferenceSemanticType
{
  NONE = 0,
  GUIDE_PATH,
  WAYPOINT_SEQUENCE,
  TRACKING_TRAJECTORY
};

struct ReferenceDefinition
{
  ReferenceSemanticType semantic{ReferenceSemanticType::NONE};
  std::string name{"reference"};
  bool active{false};

  std::vector<Eigen::Vector3d> points;
  std::vector<double> times;
  std::vector<Eigen::Vector3d> velocities;

  cost_functional::TrackingReference tracking_reference;

  bool valid() const
  {
    switch (semantic)
    {
    case ReferenceSemanticType::GUIDE_PATH:
    case ReferenceSemanticType::WAYPOINT_SEQUENCE:
      return points.size() >= 2;
    case ReferenceSemanticType::TRACKING_TRAJECTORY:
      return tracking_reference.valid();
    case ReferenceSemanticType::NONE:
    default:
      return false;
    }
  }
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_REFERENCE_DEFINITION_HPP_
