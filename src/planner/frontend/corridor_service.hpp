#ifndef PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_
#define PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_

#include <core/feasible_set_spec.hpp>
#include <frontend/guide_path_service.hpp>
#include <core/planning_context.hpp>

namespace ego_planner::frontend
{

struct CorridorRuntimeConfig
{
  GridMap::Ptr grid_map;
  double sfc_progress{0.75};
  double sfc_range{0.8};
};

class CorridorService
{
public:
  // Solver-compatible frontend entry point for historical SFC construction.
  bool generateSafeFlightCorridor(const CorridorRuntimeConfig &config,
                                  const std::vector<Eigen::Vector3d> &guide_path,
                                  spatial_map::PolyhedraH &corridor_hpolys) const;

  bool buildFromGuidePath(const core::PlanningContext &context,
                          const GuidePathArtifact &artifact,
                          core::FeasibleSetSpec &set_spec) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_
