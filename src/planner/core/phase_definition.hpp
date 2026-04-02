#ifndef PLANNER_CORE_PHASE_DEFINITION_HPP_
#define PLANNER_CORE_PHASE_DEFINITION_HPP_

#include <core/goal_definition.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ego_planner::core
{

struct PhaseDefinition
{
  std::string name{"phase"};
  GoalDefinition goal;
  std::vector<std::size_t> active_reference_indices;
  uint32_t objective_mask{0U};
  uint32_t constraint_mask{0U};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PHASE_DEFINITION_HPP_
