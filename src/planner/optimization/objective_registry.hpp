#ifndef PLANNER_OPTIMIZATION_OBJECTIVE_REGISTRY_HPP_
#define PLANNER_OPTIMIZATION_OBJECTIVE_REGISTRY_HPP_

#include <cstdint>

namespace ego_planner::optimization
{

enum ObjectiveComponent : uint32_t
{
  OBJ_SMOOTHNESS = 1u << 0,
  OBJ_FEASIBILITY = 1u << 1,
  OBJ_OBSTACLE = 1u << 2,
  OBJ_CORRIDOR = 1u << 3,
  OBJ_TRACKING_DISTANCE = 1u << 4,
  OBJ_TRACKING_VIEW = 1u << 5,
  OBJ_TRACKING_VISIBILITY = 1u << 6,
  OBJ_TERMINAL_SOFT = 1u << 7
};

inline uint32_t enableObjective(uint32_t mask, ObjectiveComponent component)
{
  return mask | static_cast<uint32_t>(component);
}

} // namespace ego_planner::optimization

#endif // PLANNER_OPTIMIZATION_OBJECTIVE_REGISTRY_HPP_

