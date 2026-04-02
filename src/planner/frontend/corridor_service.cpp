#include <frontend/corridor_service.hpp>

namespace ego_planner::frontend
{

bool CorridorService::buildFromGuidePath(const GuidePathArtifact &artifact,
                                         core::FeasibleSetSpec &set_spec) const
{
  if (!artifact.valid())
  {
    return false;
  }

  set_spec = core::FeasibleSetSpec{};
  set_spec.type = core::FeasibleSetType::CORRIDOR_POLYTOPE;
  set_spec.label = "corridor_from_guide";
  set_spec.enabled = true;
  return true;
}

} // namespace ego_planner::frontend

