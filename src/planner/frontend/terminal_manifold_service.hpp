#ifndef PLANNER_FRONTEND_TERMINAL_MANIFOLD_SERVICE_HPP_
#define PLANNER_FRONTEND_TERMINAL_MANIFOLD_SERVICE_HPP_

#include <core/feasible_set_spec.hpp>

namespace ego_planner::frontend
{

class TerminalManifoldService
{
public:
  bool buildDefaultManifold(core::FeasibleSetSpec &set_spec) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_TERMINAL_MANIFOLD_SERVICE_HPP_

