#ifndef PLANNER_FRONTEND_INIT_ARTIFACT_HPP_
#define PLANNER_FRONTEND_INIT_ARTIFACT_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <string>
#include <vector>

#include <SpatialMap/SFCCommonTypes.hpp>
#include <traj_utils/minco_types.hpp>

namespace ego_planner::frontend
{

// Unified frontend initialization artifact. This is intentionally data-only so
// existing solver code can adopt it incrementally without changing behavior.
struct InitArtifact
{
  MINCOBoundaryState3D head_state{MINCOBoundaryState3D::Zero()};
  MINCOBoundaryState3D tail_state{MINCOBoundaryState3D::Zero()};

  Eigen::MatrixXd inner_points;
  Eigen::VectorXd durations;
  MINCOTraj3D init_traj;

  std::vector<Eigen::Vector3d> dense_path;
  std::vector<Eigen::Vector3d> guide_path;
  spatial_map::PolyhedraH corridor_hpolys;
  Eigen::VectorXi corridor_piece_idx;

  std::string source;
  bool valid{false};
  bool collision_free{true};
  bool inside_corridor{true};
  double min_sdf{0.0};

  void clear()
  {
    *this = InitArtifact{};
  }

  bool hasValidTiming() const
  {
    return durations.size() > 0 && durations.allFinite();
  }

  bool hasValidPieceLayout() const
  {
    return hasValidTiming() &&
           inner_points.cols() == std::max(0, static_cast<int>(durations.size()) - 1);
  }

  bool hasValidCorridorAllocation() const
  {
    if (corridor_hpolys.empty())
    {
      return corridor_piece_idx.size() == 0;
    }

    return corridor_piece_idx.size() == static_cast<int>(corridor_hpolys.size()) &&
           corridor_piece_idx.sum() == durations.size();
  }
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_INIT_ARTIFACT_HPP_
