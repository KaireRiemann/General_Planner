#include <frontend/corridor_service.hpp>

#include <SFCGenerator/sfc_gen.hpp>

namespace ego_planner::frontend
{

bool CorridorService::buildFromGuidePath(const core::PlanningContext &context,
                                         const GuidePathArtifact &artifact,
                                         core::FeasibleSetSpec &set_spec) const
{
  if (!artifact.valid() || !context.grid_map)
  {
    return false;
  }

  spatial_map::PolyhedraH corridor_hpolys;
  std::vector<Eigen::Vector3d> occupied_points;
  context.grid_map->getInflatedOccupiedPoints(occupied_points);
  const Eigen::Vector3d low_corner = context.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d high_corner = context.grid_map->getUpdatedBoxHigh();
  if (!low_corner.allFinite() || !high_corner.allFinite())
  {
    return false;
  }

  sfc_gen::convexCover(artifact.points,
                       occupied_points,
                       low_corner,
                       high_corner,
                       context.sfc_progress,
                       context.sfc_range,
                       corridor_hpolys);
  if (corridor_hpolys.empty())
  {
    return false;
  }

  set_spec = core::FeasibleSetSpec{};
  set_spec.type = core::FeasibleSetType::CORRIDOR_POLYTOPE;
  set_spec.label = "corridor_from_guide";
  set_spec.corridor = corridor_hpolys;
  set_spec.corridor_piece_idx = Eigen::VectorXi::Ones(static_cast<int>(corridor_hpolys.size()));
  set_spec.corridor_seed_path = artifact.points;
  set_spec.corridor_seed_times = artifact.times;
  set_spec.enabled = true;
  return true;
}

} // namespace ego_planner::frontend
