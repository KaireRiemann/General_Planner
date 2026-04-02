#ifndef PLANNER_FRONTEND_VISIBLE_REGION_SERVICE_HPP_
#define PLANNER_FRONTEND_VISIBLE_REGION_SERVICE_HPP_

#include <core/feasible_set_spec.hpp>
#include <CostFunctionalManager/TrackingSemanticGuide.hpp>

namespace ego_planner::frontend
{

class VisibleRegionService
{
public:
  bool buildFromSemanticGuide(const cost_functional::TrackingSemanticGuide &guide,
                              core::FeasibleSetSpec &set_spec) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_VISIBLE_REGION_SERVICE_HPP_

