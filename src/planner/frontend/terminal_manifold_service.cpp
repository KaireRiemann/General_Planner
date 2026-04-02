#include <frontend/terminal_manifold_service.hpp>

namespace ego_planner::frontend
{

bool TerminalManifoldService::buildDefaultManifold(core::FeasibleSetSpec &set_spec) const
{
  set_spec = core::FeasibleSetSpec{};
  set_spec.type = core::FeasibleSetType::TERMINAL_MANIFOLD;
  set_spec.label = "default_terminal_manifold";
  set_spec.enabled = false;
  return true;
}

} // namespace ego_planner::frontend

