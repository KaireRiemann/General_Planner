#ifndef PLANNER_FRONTEND_WARM_START_SERVICE_HPP_
#define PLANNER_FRONTEND_WARM_START_SERVICE_HPP_

#include <core/planning_context.hpp>

namespace ego_planner::frontend
{

class WarmStartService
{
public:
  bool fetch(const core::PlanningContext &context,
             core::WarmStartCache &warm_start) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_WARM_START_SERVICE_HPP_

