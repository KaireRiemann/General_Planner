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

bool pointInsideUpdatedBox(const GridMap::Ptr &map, const Eigen::Vector3d &pt)
{
  if (!map || !mapWindowReady(map))
  {
    return true;
  }
  const Eigen::Vector3d low = map->getUpdatedBoxLow();
  const Eigen::Vector3d high = map->getUpdatedBoxHigh();
  return (pt.array() >= low.array()).all() && (pt.array() <= high.array()).all();
}

bool pointHasClearance(const GridMap::Ptr &map,
                       const Eigen::Vector3d &pt,
                       const double min_clearance)
{
  if (!map || !mapWindowReady(map))
  {
    return true;
  }
  if (!pointInsideUpdatedBox(map, pt))
  {
    return false;
  }
  if (map->getInflateOccupancy(pt) != 0)
  {
    return false;
  }
  if (!map->esdfEnabled())
  {
    return true;
  }
  const double sdf = map->getDistance(pt);
  return std::isfinite(sdf) && sdf >= std::max(0.0, min_clearance);
}

double estimateObstacleClearance(const GridMap::Ptr &map,
                                 const Eigen::Vector3d &pt,
                                 const double search_radius,
                                 Eigen::Vector3d *push_dir)
{
  if (!map || !mapWindowReady(map))
  {
    if (push_dir != nullptr)
    {
      *push_dir = Eigen::Vector3d::Zero();
    }
    return search_radius;
  }

  static const std::vector<Eigen::Vector3d> kDirs = []()
  {
    std::vector<Eigen::Vector3d> dirs;
    dirs.reserve(26);
    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dz = -1; dz <= 1; ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0)
          {
            continue;
          }
          dirs.emplace_back(Eigen::Vector3d(dx, dy, dz).normalized());
        }
      }
    }
    return dirs;
  }();

  const double resolution = std::max(map->getResolution(), 1.0e-3);
  const double max_radius = std::max(search_radius, resolution);
  Eigen::Vector3d accum = Eigen::Vector3d::Zero();

  if (map->getInflateOccupancy(pt) != 0)
  {
    for (const auto &dir : kDirs)
    {
      if (map->getInflateOccupancy(pt + dir * resolution) == 0)
      {
        accum -= dir;
      }
    }
    if (push_dir != nullptr)
    {
      *push_dir = accum.norm() > 1.0e-6 ? accum.normalized() : Eigen::Vector3d::Zero();
    }
    return 0.0;
  }

  double clearance = max_radius;
  for (double radius = resolution; radius <= max_radius + 1.0e-6; radius += resolution)
  {
    bool hit_occupied = false;
    for (const auto &dir : kDirs)
    {
      if (map->getInflateOccupancy(pt + dir * radius) != 0)
      {
        hit_occupied = true;
        accum -= dir / std::max(radius, resolution);
      }
    }
    if (hit_occupied)
    {
      clearance = std::max(0.0, radius - resolution);
      break;
    }
  }

  if (push_dir != nullptr)
  {
    *push_dir = accum.norm() > 1.0e-6 ? accum.normalized() : Eigen::Vector3d::Zero();
  }
  return clearance;
}

bool lineOfSightFree(const ego_planner::core::PlanningContext &context,
                     const Eigen::Vector3d &from,
                     const Eigen::Vector3d &to,
                     const double max_dist)
{
  if (!context.grid_map || !mapWindowReady(context.grid_map))
  {
    return true;
  }

  const double dist = (to - from).norm();
  if (max_dist > 0.0 && dist > max_dist)
  {
    return false;
  }

  const Eigen::Vector3d low = context.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d high = context.grid_map->getUpdatedBoxHigh();
  if ((from.array() < low.array()).any() || (from.array() > high.array()).any() ||
      (to.array() < low.array()).any() || (to.array() > high.array()).any())
  {
    return false;
  }

  if (dist < 1.0e-6)
  {
    return context.grid_map->getInflateOccupancy(from) == 0;
  }

  const double resolution = std::max(context.grid_map->getResolution(), 1.0e-3);
  RayCaster ray_caster;
  if (!ray_caster.setInput((from - low) / resolution, (to - low) / resolution))
  {
    return context.grid_map->getInflateOccupancy(from) == 0 &&
           context.grid_map->getInflateOccupancy(to) == 0;
  }

  Eigen::Vector3d ray_idx;
  while (ray_caster.step(ray_idx))
  {
    const Eigen::Vector3d world_pt =
        low + (ray_idx.array() + 0.5).matrix() * resolution;
    if (context.grid_map->getInflateOccupancy(world_pt) != 0)
    {
      return false;
    }
  }

  return context.grid_map->getInflateOccupancy(to) == 0;
}

bool segmentHasClearance(const GridMap::Ptr &map,
                         const Eigen::Vector3d &from,
                         const Eigen::Vector3d &to,
                         const double min_clearance)
{
  if (!map || !mapWindowReady(map))
  {
    return true;
  }

  const Eigen::Vector3d delta = to - from;
  const double dist = delta.norm();
  const double resolution = std::max(map->getResolution(), 1.0e-3);
  const double sample_step = std::max(0.5 * resolution, 0.02);
  const int sample_num = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));
  const bool use_esdf = map->esdfEnabled();
  const double clearance = std::max(0.0, min_clearance);

  for (int i = 0; i <= sample_num; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_num);
    const Eigen::Vector3d pt = from + ratio * delta;
    if (use_esdf)
    {
      if (!pointHasClearance(map, pt, clearance))
      {
        return false;
      }
    }
    else if (map->getInflateOccupancy(pt) != 0)
    {
      return false;
    }
  }
  return true;
}

bool polylineHasClearance(const GridMap::Ptr &map,
                          const std::vector<Eigen::Vector3d> &path,
                          const double min_clearance)
{
  if (path.size() < 2)
  {
    return false;
  }

  for (std::size_t i = 1; i < path.size(); ++i)
  {
    if (!segmentHasClearance(map, path[i - 1], path[i], min_clearance))
    {
      return false;
    }
  }
  return true;
}

bool sparsifyGuidePathForContext(const ego_planner::core::PlanningContext &context,
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

  if (!polylineHasClearance(context.grid_map, sparse_path, context.guide_min_clearance))
  {
    sparse_path = dense_path;
  }
  return sparse_path.size() >= 2;
}

} // namespace

namespace ego_planner::frontend
{

bool GuidePathService::sanitizeLocalTarget(const GuidePathRuntimeConfig &config,
                                           const Eigen::Vector3d &raw_target,
                                           Eigen::Vector3d &safe_target) const
{
  if (!config.grid_map || !mapWindowReady(config.grid_map))
  {
    safe_target = raw_target;
    return true;
  }

  const double resolution = std::max(config.grid_map->getResolution(), 1.0e-3);
  const Eigen::Vector3d map_low = config.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d map_high = config.grid_map->getUpdatedBoxHigh();
  if (!map_low.allFinite() || !map_high.allFinite() ||
      (map_high.array() <= map_low.array()).any())
  {
    safe_target = raw_target;
    return true;
  }

  const Eigen::Vector3d clamp_low = map_low + Eigen::Vector3d::Constant(2.0 * resolution);
  const Eigen::Vector3d clamp_high = map_high - Eigen::Vector3d::Constant(2.0 * resolution);
  safe_target = raw_target.cwiseMax(clamp_low).cwiseMin(clamp_high);
  if (pointHasClearance(config.grid_map, safe_target, 0.0))
  {
    return true;
  }

  const double sfc_range = std::max(config.sfc_range, 1.5);
  const int max_step = std::max(4, static_cast<int>(std::ceil(sfc_range / resolution)));
  for (int ring = 1; ring <= max_step; ++ring)
  {
    double best_score = -std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_candidate = safe_target;
    bool found = false;

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
          const Eigen::Vector3d candidate =
              (safe_target + Eigen::Vector3d(dx, dy, dz) * resolution).cwiseMax(clamp_low).cwiseMin(clamp_high);
          if (!pointHasClearance(config.grid_map, candidate, 0.0))
          {
            continue;
          }
          const double clearance =
              estimateObstacleClearance(config.grid_map,
                                        candidate,
                                        std::max(config.guide_min_clearance, 3.0 * resolution),
                                        nullptr);
          const double score = clearance - 0.1 * (candidate - raw_target).norm();
          if (!found || score > best_score)
          {
            best_score = score;
            best_candidate = candidate;
            found = true;
          }
        }
      }
    }

    if (found)
    {
      safe_target = best_candidate;
      return true;
    }
  }
  return false;
}

bool GuidePathService::prepareLocalAStarPath(const GuidePathRuntimeConfig &config,
                                             const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &goal_pt,
                                             std::vector<Eigen::Vector3d> &dense_path,
                                             Eigen::Vector3d &safe_goal) const
{
  dense_path.clear();
  safe_goal = goal_pt;
  Eigen::Vector3d safe_start = start_pt;

  if (!sanitizeLocalTarget(config, goal_pt, safe_goal))
  {
    return false;
  }
  if (!sanitizeLocalTarget(config, start_pt, safe_start))
  {
    safe_start = start_pt;
  }
  if ((safe_goal - safe_start).norm() < 1.0e-3)
  {
    dense_path = {start_pt, safe_goal};
    return true;
  }
  if (!config.grid_map || !mapWindowReady(config.grid_map))
  {
    dense_path = {start_pt, safe_goal};
    return true;
  }
  if (!config.jps_astar)
  {
    return false;
  }
  if (!config.jps_astar->search(safe_start, safe_goal, dense_path))
  {
    return false;
  }
  if (dense_path.size() < 2)
  {
    return false;
  }

  dense_path.front() = safe_start;
  dense_path.back() = safe_goal;
  if ((dense_path.front() - start_pt).norm() > 1.0e-3)
  {
    dense_path.insert(dense_path.begin(), start_pt);
  }
  if ((dense_path.back() - safe_goal).norm() > 1.0e-3)
  {
    dense_path.push_back(safe_goal);
  }
  return true;
}

bool GuidePathService::sparsifyGuidePath(const GuidePathRuntimeConfig &config,
                                         const std::vector<Eigen::Vector3d> &dense_path,
                                         std::vector<Eigen::Vector3d> &sparse_path) const
{
  sparse_path.clear();
  if (dense_path.size() < 2)
  {
    return false;
  }

  std::vector<double> accum_len(dense_path.size(), 0.0);
  for (std::size_t i = 1; i < dense_path.size(); ++i)
  {
    accum_len[i] = accum_len[i - 1] + (dense_path[i] - dense_path[i - 1]).norm();
  }

  const double total_len = accum_len.back();
  const double piece_length = std::max(config.poly_piece_length, 0.2);
  int desired_inner = std::max(0, static_cast<int>(std::round(total_len / piece_length)) - 1);
  desired_inner = std::max(desired_inner, config.guide_sparse_min_inner);
  desired_inner = std::min(desired_inner, config.guide_sparse_max_inner);
  desired_inner = std::min(desired_inner, static_cast<int>(dense_path.size()) - 2);

  std::vector<std::pair<double, int>> turn_candidates;
  const double turn_thresh_rad = config.guide_turn_angle_deg * std::acos(-1.0) / 180.0;
  for (int i = 1; i + 1 < static_cast<int>(dense_path.size()); ++i)
  {
    const Eigen::Vector3d vin = dense_path[static_cast<std::size_t>(i)] - dense_path[static_cast<std::size_t>(i - 1)];
    const Eigen::Vector3d vout = dense_path[static_cast<std::size_t>(i + 1)] - dense_path[static_cast<std::size_t>(i)];
    if (vin.norm() < 1.0e-4 || vout.norm() < 1.0e-4)
    {
      continue;
    }
    const double angle = std::acos(std::max(-1.0, std::min(1.0, vin.normalized().dot(vout.normalized()))));
    if (angle >= turn_thresh_rad)
    {
      turn_candidates.emplace_back(angle, i);
    }
  }
  std::sort(turn_candidates.begin(), turn_candidates.end(),
            [](const auto &a, const auto &b) { return a.first > b.first; });

  std::set<int> selected;
  for (const auto &cand : turn_candidates)
  {
    if (static_cast<int>(selected.size()) >= desired_inner)
    {
      break;
    }
    selected.insert(cand.second);
  }

  for (int k = 1; static_cast<int>(selected.size()) < desired_inner && k <= desired_inner; ++k)
  {
    const double target_s = total_len * static_cast<double>(k) / static_cast<double>(desired_inner + 1);
    int best_idx = 1;
    double best_err = std::numeric_limits<double>::infinity();
    for (int i = 1; i + 1 < static_cast<int>(dense_path.size()); ++i)
    {
      const double err = std::abs(accum_len[static_cast<std::size_t>(i)] - target_s);
      if (err < best_err)
      {
        best_err = err;
        best_idx = i;
      }
    }
    selected.insert(best_idx);
  }

  sparse_path.push_back(dense_path.front());
  for (int idx : selected)
  {
    sparse_path.push_back(dense_path[static_cast<std::size_t>(idx)]);
  }
  sparse_path.push_back(dense_path.back());

  if (!polylineHasClearance(config.grid_map, sparse_path, config.guide_min_clearance))
  {
    sparse_path = dense_path;
  }
  return sparse_path.size() >= 2;
}

bool GuidePathService::pathHasClearance(const GuidePathRuntimeConfig &config,
                                        const std::vector<Eigen::Vector3d> &path,
                                        const double min_clearance) const
{
  const double clearance = min_clearance >= 0.0 ? min_clearance : config.guide_min_clearance;
  return polylineHasClearance(config.grid_map, path, clearance);
}

bool GuidePathService::searchStateToStateDensePath(const core::PlanningContext &context,
                                                   const core::TaskDefinition &task_definition,
                                                   std::vector<Eigen::Vector3d> &path) const
{
  const auto *guide_ref =
      task_definition.findActiveReference(core::ReferenceSemanticType::GUIDE_PATH);
  if (guide_ref == nullptr)
  {
    guide_ref =
        task_definition.findActiveReference(core::ReferenceSemanticType::WAYPOINT_SEQUENCE);
  }

  if (guide_ref != nullptr && !guide_ref->points.empty())
  {
    path = guide_ref->points;
    return path.size() >= 2;
  }

  core::TaskSpec compat_task = task_definition.toTaskSpec();
  if (!task_definition.space_model_policy.force_plain &&
      task_definition.space_model_policy.preferred == core::SpaceModelPreference::CORRIDOR)
  {
    compat_task.prefer_corridor = true;
  }
  return searchStateToStateDensePath(context, compat_task, path);
}

bool GuidePathService::searchStateToStateDensePath(const core::PlanningContext &context,
                                                   const core::TaskSpec &task,
                                                   std::vector<Eigen::Vector3d> &path) const
{
  path.clear();

  if (!task.preferred_guide_path.empty())
  {
    path = task.preferred_guide_path;
    return path.size() >= 2;
  }

  Eigen::Vector3d safe_start = task.start_pt;
  Eigen::Vector3d safe_goal = task.goal_pt;
  if (!sanitizePoint(context, task.start_pt, safe_start) ||
      !sanitizePoint(context, task.goal_pt, safe_goal))
  {
    return false;
  }

  bool search_ok = false;
  if (context.jps_astar != nullptr)
  {
    search_ok = context.jps_astar->search(safe_start, safe_goal, path);
  }

  if (!search_ok)
  {
    if (task.prefer_corridor && !task.force_plain)
    {
      return false;
    }
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
  return path.size() >= 2;
}

bool GuidePathService::buildStateToStateGuide(const core::PlanningContext &context,
                                              const core::TaskDefinition &task_definition,
                                              GuidePathArtifact &artifact) const
{
  artifact.points.clear();
  artifact.times.clear();

  std::vector<Eigen::Vector3d> dense_path;
  if (!searchStateToStateDensePath(context, task_definition, dense_path))
  {
    return false;
  }

  std::vector<Eigen::Vector3d> sparse_points;
  const auto &points =
      sparsifyGuidePathForContext(context, dense_path, sparse_points) ? sparse_points : dense_path;
  if (!buildFromWaypoints(points, artifact))
  {
    return false;
  }

  const auto *guide_ref =
      task_definition.findActiveReference(core::ReferenceSemanticType::GUIDE_PATH);
  if (guide_ref == nullptr)
  {
    guide_ref =
        task_definition.findActiveReference(core::ReferenceSemanticType::WAYPOINT_SEQUENCE);
  }
  if (guide_ref != nullptr &&
      guide_ref->times.size() == guide_ref->points.size() &&
      guide_ref->points.size() == points.size())
  {
    artifact.times = guide_ref->times;
  }
  return artifact.valid();
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

  std::vector<Eigen::Vector3d> dense_path;
  if (!searchStateToStateDensePath(context, task, dense_path))
  {
    return false;
  }

  std::vector<Eigen::Vector3d> sparse_path;
  if (sparsifyGuidePathForContext(context, dense_path, sparse_path))
  {
    dense_path.swap(sparse_path);
  }

  return buildFromWaypoints(dense_path, artifact);
}

bool GuidePathService::buildTrackingGuideFromWaypoints(const core::PlanningContext &context,
                                                       const std::vector<Eigen::Vector3d> &waypoints,
                                                       const double connect_dist,
                                                       GuidePathArtifact &artifact) const
{
  artifact.points.clear();
  artifact.times.clear();
  if (waypoints.size() < 2)
  {
    return false;
  }

  std::vector<Eigen::Vector3d> dense_guide;
  dense_guide.reserve(waypoints.size());
  dense_guide.push_back(waypoints.front());
  for (std::size_t i = 1; i < waypoints.size(); ++i)
  {
    const Eigen::Vector3d &next_wp = waypoints[i];
    if ((next_wp - dense_guide.back()).norm() < 1.0e-3)
    {
      continue;
    }

    if (lineOfSightFree(context, dense_guide.back(), next_wp, connect_dist))
    {
      dense_guide.push_back(next_wp);
      continue;
    }

    Eigen::Vector3d safe_start = dense_guide.back();
    Eigen::Vector3d safe_goal = next_wp;
    if (!sanitizePoint(context, dense_guide.back(), safe_start) ||
        !sanitizePoint(context, next_wp, safe_goal) ||
        context.jps_astar == nullptr)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> segment_path;
    if (!context.jps_astar->search(safe_start, safe_goal, segment_path) ||
        segment_path.size() < 2)
    {
      return false;
    }

    if ((segment_path.front() - dense_guide.back()).norm() > 1.0e-3)
    {
      segment_path.insert(segment_path.begin(), dense_guide.back());
    }
    if ((segment_path.back() - next_wp).norm() > 1.0e-3)
    {
      segment_path.push_back(next_wp);
    }

    for (std::size_t j = 1; j < segment_path.size(); ++j)
    {
      if ((segment_path[j] - dense_guide.back()).norm() > 1.0e-3)
      {
        dense_guide.push_back(segment_path[j]);
      }
    }
  }

  if (dense_guide.size() < 2)
  {
    return false;
  }

  std::vector<Eigen::Vector3d> shortcut_path;
  shortcut_path.reserve(dense_guide.size());
  std::size_t anchor_idx = 0;
  shortcut_path.push_back(dense_guide.front());
  while (anchor_idx + 1 < dense_guide.size())
  {
    std::size_t next_idx = anchor_idx + 1;
    for (std::size_t cand = dense_guide.size(); cand-- > anchor_idx + 1;)
    {
      if (lineOfSightFree(context, dense_guide[anchor_idx], dense_guide[cand], -1.0))
      {
        next_idx = cand;
        break;
      }
    }

    if ((dense_guide[next_idx] - shortcut_path.back()).norm() > 1.0e-3)
    {
      shortcut_path.push_back(dense_guide[next_idx]);
    }
    anchor_idx = next_idx;
  }

  std::vector<Eigen::Vector3d> sparse_path;
  const auto &final_points =
      sparsifyGuidePathForContext(context, shortcut_path, sparse_path) ? sparse_path : shortcut_path;
  return buildFromWaypoints(final_points, artifact);
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
