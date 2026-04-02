#ifndef PLANNER_CORE_SPACE_MODEL_POLICY_HPP_
#define PLANNER_CORE_SPACE_MODEL_POLICY_HPP_

namespace ego_planner::core
{

enum class SpaceModelPreference
{
  AUTO = 0,
  PLAIN,
  ESDF,
  CORRIDOR,
  VISIBLE_REGION,
  TERMINAL_MANIFOLD
};

struct SpaceModelPolicy
{
  SpaceModelPreference preferred{SpaceModelPreference::AUTO};
  bool allow_plain{true};
  bool allow_esdf{true};
  bool allow_corridor{true};
  bool allow_visible_region{true};
  bool allow_terminal_manifold{true};
  bool force_plain{false};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_SPACE_MODEL_POLICY_HPP_
