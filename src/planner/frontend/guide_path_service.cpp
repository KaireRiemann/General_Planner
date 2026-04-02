#include <frontend/guide_path_service.hpp>

#include <algorithm>
#include <limits>
#include <set>

#include <cmath>

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

bool sparsifyGuidePath(const ego_planner::core::PlanningContext &context,
                       const std::vector<Eigen::Vector3d> &dense_path,
                       std::vector<Eigen::Vector3d> &sparse_path)
{
  sparse_path.clear();
  if (dense_path.size() < 2)
  {
    return false;
  }

  if (dense_path.size() <= 2)
  {
    sparse_path = dense_path;
    return true;
  }

  std::vector<double> accum_len(dense_path.size(), 0.0);
  for (std::size_t i = 1; i < dense_path.size(); ++i)
  {
    accum_len[i] = accum_len[i - 1] + (dense_path[i] - dense_path[i - 1]).norm();
  }

  const double total_len = accum_len.back();
  const double piece_length = std::max(context.poly_piece_length, 0.2);
  int desired_inner = std::max(0, static_cast<int>(std::round(total_len / piece_length)) - 1);
  desired_inner = std::max(desired_inner, context.guide_sparse_min_inner);
  desired_inner = std::min(desired_inner, context.guide_sparse_max_inner);
  desired_inner = std::min(desired_inner, static_cast<int>(dense_path.size()) - 2);

  std::vector<std::pair<double, int>> turn_candidates;
  const double turn_thresh_rad =
      context.guide_turn_angle_deg * std::acos(-1.0) / 180.0;
  for (int i = 1; i + 1 < static_cast<int>(dense_path.size()); ++i)
  {
    const Eigen::Vector3d vin =
        dense_path[static_cast<std::size_t>(i)] - dense_path[static_cast<std::size_t>(i - 1)];
    const Eigen::Vector3d vout =
        dense_path[static_cast<std::size_t>(i + 1)] - dense_path[static_cast<std::size_t>(i)];
    if (vin.norm() < 1.0e-4 || vout.norm() < 1.0e-4)
    {
      continue;
    }

    const double angle = std::acos(
        std::max(-1.0, std::min(1.0, vin.normalized().dot(vout.normalized()))));
    if (angle >= turn_thresh_rad)
    {
      turn_candidates.emplace_back(angle, i);
    }
  }

  std::sort(turn_candidates.begin(),
            turn_candidates.end(),
            [](const auto &lhs, const auto &rhs)
            {
              return lhs.first > rhs.first;
            });

  std::set<int> selected;
  for (const auto &candidate : turn_candidates)
  {
    if (static_cast<int>(selected.size()) >= desired_inner)
    {
      break;
    }
    selected.insert(candidate.second);
  }

  for (int k = 1; static_cast<int>(selected.size()) < desired_inner && k <= desired_inner; ++k)
  {
    const double target_s =
        total_len * static_cast<double>(k) / static_cast<double>(desired_inner + 1);
    int best_idx = 1;
    double best_err = std::numeric_limits<double>::infinity();
    for (int i = 1; i + 1 < static_cast<int>(dense_path.size()); ++i)
    {
      const double err =
          std::abs(accum_len[static_cast<std::size_t>(i)] - target_s);
      if (err < best_err)
      {
        best_err = err;
        best_idx = i;
      }
    }
    selected.insert(best_idx);
  }

  sparse_path.push_back(dense_path.front());
  for (const int idx : selected)
  {
    sparse_path.push_back(dense_path[static_cast<std::size_t>(idx)]);
  }
  sparse_path.push_back(dense_path.back());
  return sparse_path.size() >= 2;
}

} // namespace

namespace ego_planner::frontend
{

bool GuidePathService::buildStateToStateGuide(const core::PlanningContext &context,
                                              const core::TaskDefinition &task_definition,
                                              GuidePathArtifact &artifact) const
{
  artifact.points.clear();
  artifact.times.clear();

  const auto *guide_ref =
      task_definition.findActiveReference(core::ReferenceSemanticType::GUIDE_PATH);
  if (guide_ref == nullptr)
  {
    guide_ref =
        task_definition.findActiveReference(core::ReferenceSemanticType::WAYPOINT_SEQUENCE);
  }

  if (guide_ref != nullptr && !guide_ref->points.empty())
  {
    std::vector<Eigen::Vector3d> sparse_points;
    const auto &points =
        sparsifyGuidePath(context, guide_ref->points, sparse_points) ? sparse_points : guide_ref->points;
    if (!buildFromWaypoints(points, artifact))
    {
      return false;
    }
    if (guide_ref->times.size() == guide_ref->points.size() &&
        guide_ref->points.size() == points.size())
    {
      artifact.times = guide_ref->times;
    }
    return artifact.valid();
  }

  core::TaskSpec compat_task = task_definition.toTaskSpec();
  return buildStateToStateGuide(context, compat_task, artifact);
}

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
    std::vector<Eigen::Vector3d> sparse_points;
    const auto &points =
        sparsifyGuidePath(context, task.preferred_guide_path, sparse_points)
            ? sparse_points
            : task.preferred_guide_path;
    return buildFromWaypoints(points, artifact);
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

  std::vector<Eigen::Vector3d> sparse_path;
  if (sparsifyGuidePath(context, path, sparse_path))
  {
    path.swap(sparse_path);
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
