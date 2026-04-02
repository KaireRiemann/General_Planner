#include <frontend/guide_path_service.hpp>

#include <algorithm>

namespace ego_planner::frontend
{

bool GuidePathService::buildFromWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                                          GuidePathArtifact &artifact) const
{
  artifact.points.clear();
  artifact.times.clear();
  if (waypoints.size() < 2)
  {
    return false;
  }

  artifact.points = waypoints;
  artifact.times.resize(waypoints.size(), 0.0);
  for (std::size_t i = 1; i < waypoints.size(); ++i)
  {
    const double ds = (waypoints[i] - waypoints[i - 1]).norm();
    artifact.times[i] = artifact.times[i - 1] + std::max(0.05, ds);
  }
  return artifact.valid();
}

} // namespace ego_planner::frontend

