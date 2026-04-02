#ifndef PLANNER_OPTIMIZATION_CONSTRAINT_REGISTRY_HPP_
#define PLANNER_OPTIMIZATION_CONSTRAINT_REGISTRY_HPP_

#include <cstdint>

namespace ego_planner::optimization
{

enum ConstraintComponent : uint32_t
{
  CON_DYNAMICS = 1u << 0,
  CON_COLLISION = 1u << 1,
  CON_CORRIDOR = 1u << 2,
  CON_SWARM = 1u << 3,
  CON_VISIBLE_REGION = 1u << 4
};

inline uint32_t enableConstraint(uint32_t mask, ConstraintComponent component)
{
  return mask | static_cast<uint32_t>(component);
}

} // namespace ego_planner::optimization

#endif // PLANNER_OPTIMIZATION_CONSTRAINT_REGISTRY_HPP_

