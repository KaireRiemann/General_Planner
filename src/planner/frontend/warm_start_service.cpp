#include <frontend/warm_start_service.hpp>

namespace ego_planner::frontend
{

bool WarmStartService::fetch(const core::PlanningContext &context,
                             core::WarmStartCache &warm_start) const
{
  warm_start = context.warm_start;
  return warm_start.valid;
}

} // namespace ego_planner::frontend

