#include <frontend/guide_path_service.hpp>

#include <algorithm>
#include <limits>

namespace
{

bool mapWindowReady(const GridMap::Ptr &map)
{
  if (!map)
  {
    return false;
  }
  const Eigen::Vector3d low = map->getUpdatedBoxLow();
  const Eigen::Vector3d high = map->getUpdatedBoxHigh();
  if (!low.allFinite() || !high.allFinite())
  {
    return false;
  }
  const double res = std::max(map->getResolution(), 1.0e-3);
  return ((high - low).array() > 6.0 * res).all();
}

} // namespace

namespace ego_planner::frontend
{

bool GuidePathService::sanitizePoint(const core::PlanningContext &context,
                                     const Eigen::Vector3d &raw,
                                     Eigen::Vector3d &safe) const
{
  if (!context.grid_map || !mapWindowReady(context.grid_map))
  {
    safe = raw;
    return true;
  }

  const double res = std::max(context.grid_map->getResolution(), 1.0e-3);
  const Eigen::Vector3d low = context.grid_map->getUpdatedBoxLow() + Eigen::Vector3d::Constant(2.0 * res);
  const Eigen::Vector3d high = context.grid_map->getUpdatedBoxHigh() - Eigen::Vector3d::Constant(2.0 * res);
  safe = raw.cwiseMax(low).cwiseMin(high);
  if (context.grid_map->getInflateOccupancy(safe) == 0)
  {
    return true;
  }

  const int max_ring = std::max(3, static_cast<int>(std::ceil(1.5 / res)));
  for (int ring = 1; ring <= max_ring; ++ring)
  {
    for (int dx = -ring; dx <= ring; ++dx)
    {
      for (int dy = -ring; dy <= ring; ++dy)
      {
        for (int dz = -ring; dz <= ring; ++dz)
        {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != ring)
          {
            continue;
          }
          Eigen::Vector3d cand = safe + Eigen::Vector3d(dx, dy, dz) * res;
          cand = cand.cwiseMax(low).cwiseMin(high);
          if (context.grid_map->getInflateOccupancy(cand) == 0)
          {
            safe = cand;
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool GuidePathService::buildStateToStateGuide(const core::PlanningContext &context,
                                              const core::TaskSpec &task,
                                              GuidePathArtifact &artifact) const
{
  artifact.points.clear();
  artifact.times.clear();

  if (!task.preferred_guide_path.empty())
  {
    return buildFromWaypoints(task.preferred_guide_path, artifact);
  }

  Eigen::Vector3d safe_start = task.start_pt;
  Eigen::Vector3d safe_goal = task.goal_pt;
  if (!sanitizePoint(context, task.start_pt, safe_start) ||
      !sanitizePoint(context, task.goal_pt, safe_goal))
  {
    return false;
  }

  std::vector<Eigen::Vector3d> path;
  bool search_ok = false;
  if (context.jps_astar != nullptr)
  {
    search_ok = context.jps_astar->search(safe_start, safe_goal, path);
  }

  if (!search_ok)
  {
    path.clear();
    path.push_back(safe_start);
    path.push_back(safe_goal);
  }

  if (path.size() < 2)
  {
    return false;
  }

  if ((path.front() - task.start_pt).norm() > 1.0e-3)
  {
    path.insert(path.begin(), task.start_pt);
  }
  if ((path.back() - task.goal_pt).norm() > 1.0e-3)
  {
    path.push_back(task.goal_pt);
  }

  return buildFromWaypoints(path, artifact);
}

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
