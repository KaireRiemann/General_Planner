#ifndef PLANNER_CORE_GOAL_DEFINITION_HPP_
#define PLANNER_CORE_GOAL_DEFINITION_HPP_

#include <Eigen/Core>

namespace ego_planner::core
{

enum class GoalSemanticType
{
  FIXED_STATE = 0,
  TERMINAL_SET,
  TERMINAL_MANIFOLD
};

struct StateDefinition
{
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct GoalDefinition
{
  GoalSemanticType semantic{GoalSemanticType::FIXED_STATE};
  StateDefinition state;
  Eigen::VectorXd manifold_params;
  bool touch_goal{false};

  bool isTerminalSet() const
  {
    return semantic == GoalSemanticType::TERMINAL_SET;
  }

  bool isTerminalManifold() const
  {
    return semantic == GoalSemanticType::TERMINAL_MANIFOLD;
  }
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_GOAL_DEFINITION_HPP_
