#include <frontend/visible_region_service.hpp>

namespace ego_planner::frontend
{

bool VisibleRegionService::buildFromSemanticGuide(const cost_functional::TrackingSemanticGuide &guide,
                                                  core::FeasibleSetSpec &set_spec) const
{
  if (!guide.consistent())
  {
    return false;
  }
  set_spec = core::FeasibleSetSpec{};
  set_spec.type = core::FeasibleSetType::VISIBLE_REGION_TUBE;
  set_spec.label = "visible_region_tube";
  set_spec.visible_regions = guide.visible_regions;
  set_spec.enabled = !set_spec.visible_regions.empty();
  return set_spec.enabled;
}

} // namespace ego_planner::frontend

