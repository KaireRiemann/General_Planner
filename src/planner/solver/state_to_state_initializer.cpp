#include <solver/state_to_state_initializer.hpp>

#include <ros/ros.h>
#include <SFCGenerator/sfc_gen.hpp>
#include <plan_env/raycast.h>
#include <SpatialMap/CorridorInit.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>

namespace
{

using ego_planner::MINCOBoundaryState3D;
using ego_planner::MINCOTraj3D;

MINCOBoundaryState3D makeBoundaryState(const Eigen::Vector3d &pos,
                                       const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &acc)
{
  MINCOBoundaryState3D state = MINCOBoundaryState3D::Zero();
  if constexpr (MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM > 0)
  {
    state.col(0) = pos;
  }
  if constexpr (MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM > 1)
  {
    state.col(1) = vel;
  }
  if constexpr (MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM > 2)
  {
    state.col(2) = acc;
  }
  return state;
}

MINCOTraj3D generateMINCOTraj(const MINCOBoundaryState3D &headState,
                              const MINCOBoundaryState3D &tailState,
                              const Eigen::MatrixXd &innerPts,
                              const Eigen::VectorXd &durations)
{
  MINCOTraj3D traj;
  traj.generate(innerPts, headState, tailState, durations);
  return traj;
}

bool resamplePolylineByCount(const std::vector<Eigen::Vector3d> &path,
                             const int sample_count,
                             std::vector<Eigen::Vector3d> &samples)
{
  samples.clear();
  if (path.size() < 2 || sample_count < 2)
  {
    return false;
  }

  std::vector<double> accum(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    accum[i] = accum[i - 1] + (path[i] - path[i - 1]).norm();
  }

  const double total_len = accum.back();
  if (total_len < 1.0e-6)
  {
    samples.assign(static_cast<std::size_t>(sample_count), path.front());
    return true;
  }

  samples.reserve(static_cast<std::size_t>(sample_count));
  for (int k = 0; k < sample_count; ++k)
  {
    const double target_s = total_len * static_cast<double>(k) /
                            static_cast<double>(sample_count - 1);
    auto upper = std::lower_bound(accum.begin(), accum.end(), target_s);
    const int idx = static_cast<int>(std::distance(accum.begin(), upper));
    if (idx <= 0)
    {
      samples.push_back(path.front());
      continue;
    }
    if (idx >= static_cast<int>(path.size()))
    {
      samples.push_back(path.back());
      continue;
    }

    const double s0 = accum[static_cast<std::size_t>(idx - 1)];
    const double s1 = accum[static_cast<std::size_t>(idx)];
    const double alpha = (target_s - s0) / std::max(1.0e-9, s1 - s0);
    samples.push_back(path[static_cast<std::size_t>(idx - 1)] +
                      alpha * (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]));
  }
  return true;
}

} // namespace

namespace ego_planner::solver
{

bool StateToStateInitializer::mapWindowReady() const
{
  if (!resources_.grid_map)
  {
    return false;
  }

  const Eigen::Vector3d low = resources_.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d high = resources_.grid_map->getUpdatedBoxHigh();
  if (!low.allFinite() || !high.allFinite())
  {
    return false;
  }

  const double res = std::max(resources_.grid_map->getResolution(), 1.0e-3);
  return ((high - low).array() > 6.0 * res).all();
}

bool StateToStateInitializer::sanitizeLocalTarget(const Eigen::Vector3d &raw_target,
                                                  Eigen::Vector3d &safe_target) const
{
  if (!resources_.grid_map || !mapWindowReady())
  {
    safe_target = raw_target;
    return true;
  }

  const double resolution = std::max(resources_.grid_map->getResolution(), 1.0e-3);
  const Eigen::Vector3d map_low = resources_.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d map_high = resources_.grid_map->getUpdatedBoxHigh();
  if (!map_low.allFinite() || !map_high.allFinite() ||
      (map_high.array() <= map_low.array()).any())
  {
    safe_target = raw_target;
    return true;
  }

  const Eigen::Vector3d clamp_low = map_low + Eigen::Vector3d::Constant(2.0 * resolution);
  const Eigen::Vector3d clamp_high = map_high - Eigen::Vector3d::Constant(2.0 * resolution);
  safe_target = raw_target.cwiseMax(clamp_low).cwiseMin(clamp_high);
  if (resources_.grid_map->getInflateOccupancy(safe_target) == 0)
  {
    return true;
  }

  const double sfc_range = std::max(resources_.sfc_range, 1.5);
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
          if (resources_.grid_map->getInflateOccupancy(candidate) != 0)
          {
            continue;
          }
          const double clearance =
              estimateObstacleClearance(candidate,
                                        std::max(resources_.guide_min_clearance, 3.0 * resolution),
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

bool StateToStateInitializer::sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
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
  const double piece_length = std::max(resources_.plan_params ? resources_.plan_params->polyTraj_piece_length : 0.2, 0.2);
  int desired_inner = std::max(0, static_cast<int>(std::round(total_len / piece_length)) - 1);
  desired_inner = std::max(desired_inner, resources_.guide_sparse_min_inner);
  desired_inner = std::min(desired_inner, resources_.guide_sparse_max_inner);
  desired_inner = std::min(desired_inner, static_cast<int>(dense_path.size()) - 2);

  std::vector<std::pair<double, int>> turn_candidates;
  const double turn_thresh_rad = resources_.guide_turn_angle_deg * M_PI / 180.0;
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
  return sparse_path.size() >= 2;
}

double StateToStateInitializer::estimateObstacleClearance(const Eigen::Vector3d &pt,
                                                          double search_radius,
                                                          Eigen::Vector3d *push_dir) const
{
  if (!resources_.grid_map || !mapWindowReady())
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

  const double resolution = std::max(resources_.grid_map->getResolution(), 1.0e-3);
  const double max_radius = std::max(search_radius, resolution);
  Eigen::Vector3d accum = Eigen::Vector3d::Zero();

  if (resources_.grid_map->getInflateOccupancy(pt) != 0)
  {
    for (const auto &dir : kDirs)
    {
      if (resources_.grid_map->getInflateOccupancy(pt + dir * resolution) == 0)
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
      if (resources_.grid_map->getInflateOccupancy(pt + dir * radius) != 0)
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

bool StateToStateInitializer::lineOfSightFree(const Eigen::Vector3d &from,
                                              const Eigen::Vector3d &to,
                                              double max_dist) const
{
  if (!resources_.grid_map || !mapWindowReady())
  {
    return true;
  }

  const double dist = (to - from).norm();
  if (max_dist > 0.0 && dist > max_dist)
  {
    return false;
  }

  const Eigen::Vector3d low = resources_.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d high = resources_.grid_map->getUpdatedBoxHigh();
  const auto inside = [&](const Eigen::Vector3d &pt) -> bool
  {
    return (pt.array() >= low.array()).all() && (pt.array() <= high.array()).all();
  };
  if (!inside(from) || !inside(to))
  {
    return false;
  }
  if (dist < 1.0e-6)
  {
    return resources_.grid_map->getInflateOccupancy(from) == 0;
  }

  const double resolution = std::max(resources_.grid_map->getResolution(), 1.0e-3);
  RayCaster ray_caster;
  if (!ray_caster.setInput((from - low) / resolution, (to - low) / resolution))
  {
    return resources_.grid_map->getInflateOccupancy(from) == 0 &&
           resources_.grid_map->getInflateOccupancy(to) == 0;
  }

  Eigen::Vector3d ray_idx;
  while (ray_caster.step(ray_idx))
  {
    const Eigen::Vector3d world_pt =
        low + (ray_idx.array() + 0.5).matrix() * resolution;
    if (resources_.grid_map->getInflateOccupancy(world_pt) != 0)
    {
      return false;
    }
  }
  return resources_.grid_map->getInflateOccupancy(to) == 0;
}

double StateToStateInitializer::computeTrajectoryMinSdf(const MINCOTraj3D &traj) const
{
  if (!resources_.grid_map || !resources_.plan_params)
  {
    return 0.0;
  }

  const double total_duration = traj.getTotalDuration();
  const double dt = std::max(0.01, std::min(0.05, resources_.grid_map->getResolution() /
                                                      std::max(resources_.plan_params->max_vel_, 0.1)));
  double min_sdf = std::numeric_limits<double>::infinity();

  for (double t = 0.0; t <= total_duration + 1.0e-6; t += dt)
  {
    const double sample_t = std::min(t, total_duration);
    const Eigen::Vector3d pt = traj.evaluate(sample_t, 0);
    if (resources_.grid_map->esdfEnabled())
    {
      min_sdf = std::min(min_sdf, resources_.grid_map->getDistance(pt));
    }
    else
    {
      min_sdf = std::min(min_sdf,
                         estimateObstacleClearance(pt,
                                                   std::max(resources_.guide_min_clearance,
                                                            3.0 * resources_.grid_map->getResolution()),
                                                   nullptr));
    }
  }
  return std::isfinite(min_sdf) ? min_sdf : 0.0;
}

bool StateToStateInitializer::assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                                              Eigen::MatrixXd &inner_pts,
                                                              Eigen::VectorXd &durations,
                                                              std::vector<double> *inner_clearances) const
{
  if (anchors.size() < 2 || resources_.plan_params == nullptr)
  {
    return false;
  }

  std::vector<Eigen::Vector3d> expanded;
  expanded.reserve(anchors.size() * 2);
  expanded.push_back(anchors.front());

  const double piece_length = std::max(resources_.plan_params->polyTraj_piece_length, 0.2);
  for (std::size_t i = 1; i < anchors.size(); ++i)
  {
    const Eigen::Vector3d &p0 = anchors[i - 1];
    const Eigen::Vector3d &p1 = anchors[i];
    const double seg_len = (p1 - p0).norm();
    const int split_num = std::max(1, static_cast<int>(std::ceil(seg_len / piece_length)));
    for (int k = 1; k <= split_num; ++k)
    {
      const double ratio = static_cast<double>(k) / static_cast<double>(split_num);
      const Eigen::Vector3d pt = p0 + ratio * (p1 - p0);
      if ((pt - expanded.back()).norm() > 1.0e-4)
      {
        expanded.push_back(pt);
      }
    }
  }

  if (expanded.size() < 2)
  {
    return false;
  }
  if (expanded.size() == 2)
  {
    expanded.insert(expanded.begin() + 1, 0.5 * (expanded.front() + expanded.back()));
  }

  const int piece_num = static_cast<int>(expanded.size()) - 1;
  durations.resize(piece_num);
  inner_pts.resize(3, std::max(0, piece_num - 1));
  if (inner_clearances != nullptr)
  {
    inner_clearances->clear();
    inner_clearances->reserve(std::max(0, piece_num - 1));
  }

  for (int i = 0; i < piece_num; ++i)
  {
    const double seg_len = (expanded[i + 1] - expanded[i]).norm();
    durations(i) = std::max(seg_len / std::max(resources_.plan_params->max_vel_, 0.1), 0.2);
    if (i < piece_num - 1)
    {
      inner_pts.col(i) = expanded[i + 1];
      if (inner_clearances != nullptr && resources_.grid_map)
      {
        inner_clearances->push_back(
            estimateObstacleClearance(expanded[i + 1],
                                      std::max(resources_.guide_min_clearance * 1.5,
                                               2.0 * resources_.grid_map->getResolution()),
                                      nullptr));
      }
    }
  }
  return true;
}

bool StateToStateInitializer::buildInitStateFromGuidePath(const Eigen::Vector3d &start_pt,
                                                          const Eigen::Vector3d &start_vel,
                                                          const Eigen::Vector3d &start_acc,
                                                          const Eigen::Vector3d &target_pt,
                                                          const Eigen::Vector3d &target_vel,
                                                          const std::vector<Eigen::Vector3d> &guide_path,
                                                          MINCOTraj3D &init_traj,
                                                          Eigen::MatrixXd &inner_pts,
                                                          Eigen::VectorXd &durations,
                                                          MINCOBoundaryState3D &head_state,
                                                          MINCOBoundaryState3D &tail_state) const
{
  if (guide_path.size() < 2)
  {
    return false;
  }
  std::vector<Eigen::Vector3d> anchors = guide_path;
  anchors.front() = start_pt;
  anchors.back() = target_pt;
  head_state = makeBoundaryState(start_pt, start_vel, start_acc);
  tail_state = makeBoundaryState(target_pt, target_vel, Eigen::Vector3d::Zero());

  if (!assembleInitialGuessFromAnchors(anchors, inner_pts, durations, nullptr))
  {
    return false;
  }
  if (!init_traj.generate(inner_pts, head_state, tail_state, durations))
  {
    return false;
  }
  return init_traj.getTotalDuration() > 1.0e-6;
}

bool StateToStateInitializer::applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                                          Eigen::VectorXd &durations) const
{
  if (warm_durations.size() <= 0 || durations.size() <= 0 ||
      !warm_durations.allFinite() || !durations.allFinite())
  {
    return false;
  }

  const double warm_total = warm_durations.sum();
  const double init_total = durations.sum();
  if (warm_total <= 1.0e-6 || init_total <= 1.0e-6)
  {
    return false;
  }

  const Eigen::VectorXd warm_safe = warm_durations.cwiseMax(0.03);
  Eigen::VectorXd adjusted = durations.cwiseMax(0.03);
  if (warm_safe.size() == adjusted.size())
  {
    adjusted = 0.35 * adjusted + 0.65 * warm_safe;
  }
  else
  {
    const double scale = std::min(2.5, std::max(0.4, warm_total / init_total));
    adjusted *= scale;
  }

  const double adjusted_sum = adjusted.sum();
  if (adjusted_sum <= 1.0e-6)
  {
    return false;
  }
  adjusted *= warm_total / adjusted_sum;
  durations = adjusted.cwiseMax(0.03);
  return true;
}

bool StateToStateInitializer::buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                                             const Eigen::Vector3d &start_vel,
                                                             const Eigen::Vector3d &goal_pt,
                                                             const spatial_map::PolyhedraH &corridor_hpolys,
                                                             Eigen::MatrixXd &inner_pts,
                                                             Eigen::VectorXd &durations,
                                                             Eigen::VectorXi &corridor_piece_idx,
                                                             std::vector<Eigen::Vector3d> &transition_points,
                                                             std::vector<double> &inner_clearances) const
{
  transition_points.clear();
  inner_clearances.clear();
  inner_pts.resize(3, 0);
  durations.resize(0);
  corridor_piece_idx.resize(0);

  if (corridor_hpolys.empty() || resources_.plan_params == nullptr)
  {
    return false;
  }

  const double piece_length = std::max(resources_.plan_params->polyTraj_piece_length, 0.2);
  const double alloc_speed = std::max(0.9 * resources_.plan_params->max_vel_, 0.35);
  std::vector<Eigen::Vector3d> short_path;
  if (!spatial_map::buildCorridorInit(start_pt,
                                      start_vel,
                                      goal_pt,
                                      corridor_hpolys,
                                      piece_length,
                                      alloc_speed,
                                      inner_pts,
                                      durations,
                                      &transition_points,
                                      &short_path,
                                      &corridor_piece_idx))
  {
    return false;
  }

  inner_clearances.resize(std::max(0, static_cast<int>(inner_pts.cols())));
  const double clearance_probe_radius =
      std::max(resources_.guide_min_clearance,
               2.0 * (resources_.grid_map ? resources_.grid_map->getResolution() : 0.1));
  for (int i = 0; i < inner_pts.cols(); ++i)
  {
    inner_clearances[i] = estimateObstacleClearance(inner_pts.col(i),
                                                    clearance_probe_radius,
                                                    nullptr);
  }
  return true;
}

bool StateToStateInitializer::generateSafeFlightCorridor(const std::vector<Eigen::Vector3d> &guide_path,
                                                         spatial_map::PolyhedraH &corridor_hpolys) const
{
  corridor_hpolys.clear();
  if (guide_path.size() < 2 || !resources_.grid_map)
  {
    return false;
  }

  const Eigen::Vector3d low_corner = resources_.grid_map->getUpdatedBoxLow();
  const Eigen::Vector3d high_corner = resources_.grid_map->getUpdatedBoxHigh();
  std::vector<Eigen::Vector3d> occupied_points;
  resources_.grid_map->getInflatedOccupiedPoints(occupied_points);

  sfc_gen::convexCover(guide_path,
                       occupied_points,
                       low_corner,
                       high_corner,
                       resources_.sfc_progress,
                       resources_.sfc_range,
                       corridor_hpolys);
  sfc_gen::shortCut(corridor_hpolys);
  return !corridor_hpolys.empty();
}

bool StateToStateInitializer::prepareLocalAStarPath(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Vector3d &goal_pt,
                                                    std::vector<Eigen::Vector3d> &dense_path,
                                                    Eigen::Vector3d &safe_goal) const
{
  dense_path.clear();
  safe_goal = goal_pt;
  Eigen::Vector3d safe_start = start_pt;

  if (!sanitizeLocalTarget(goal_pt, safe_goal))
  {
    return false;
  }
  if (!sanitizeLocalTarget(start_pt, safe_start))
  {
    safe_start = start_pt;
  }
  if ((safe_goal - safe_start).norm() < 1.0e-3)
  {
    dense_path = {start_pt, safe_goal};
    return true;
  }
  if (!resources_.grid_map || !mapWindowReady())
  {
    dense_path = {start_pt, safe_goal};
    return true;
  }
  if (!resources_.jps_astar)
  {
    return false;
  }
  if (!resources_.jps_astar->search(safe_start, safe_goal, dense_path))
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

bool StateToStateInitializer::prepareLocalGuideAndCorridor(const Eigen::Vector3d &start_pt,
                                                           const Eigen::Vector3d &start_vel,
                                                           const Eigen::Vector3d &goal_pt,
                                                           std::vector<Eigen::Vector3d> &guide_path,
                                                           spatial_map::PolyhedraH &corridor_hpolys,
                                                           std::vector<Eigen::Vector3d> *dense_path) const
{
  (void)start_vel;
  guide_path.clear();
  corridor_hpolys.clear();

  Eigen::Vector3d safe_goal;
  std::vector<Eigen::Vector3d> local_dense_path;
  if (!prepareLocalAStarPath(start_pt, goal_pt, local_dense_path, safe_goal))
  {
    return false;
  }

  std::vector<Eigen::Vector3d> sparse_path;
  if (!sparsifyGuidePath(local_dense_path, sparse_path))
  {
    return false;
  }

  guide_path = sparse_path;
  if (!generateSafeFlightCorridor(guide_path, corridor_hpolys))
  {
    guide_path = local_dense_path;
    if (!generateSafeFlightCorridor(guide_path, corridor_hpolys))
    {
      return false;
    }
  }
  if (dense_path != nullptr)
  {
    *dense_path = local_dense_path;
  }
  return true;
}

bool StateToStateInitializer::computeInitState(const Eigen::Vector3d &start_pt,
                                               const Eigen::Vector3d &start_vel,
                                               const Eigen::Vector3d &start_acc,
                                               const Eigen::Vector3d &local_target_pt,
                                               const Eigen::Vector3d &local_target_vel,
                                               const bool flag_polyInit,
                                               const bool flag_randomPolyTraj,
                                               const double &ts,
                                               MINCOTraj3D &initTraj,
                                               Eigen::MatrixXd &outInnerPts,
                                               Eigen::VectorXd &outDurations,
                                               MINCOBoundaryState3D &headState,
                                               MINCOBoundaryState3D &tailState,
                                               const core::PlanningContext *planning_context) const
{
  if (resources_.traj_container == nullptr || resources_.plan_params == nullptr)
  {
    return false;
  }

  static bool flag_first_call = true;
  const int fail_count = resources_.continuous_failures_count ? *resources_.continuous_failures_count : 0;

  if (flag_first_call || flag_polyInit)
  {
    flag_first_call = false;

    Eigen::MatrixXd innerPs(3, 0);
    Eigen::VectorXd piece_dur_vec;
    int piece_nums;
    constexpr double init_of_init_totaldur = 2.0;
    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(local_target_pt, local_target_vel, Eigen::Vector3d::Zero());

    if (!flag_randomPolyTraj)
    {
      piece_nums = 1;
      piece_dur_vec.resize(1);
      piece_dur_vec(0) = init_of_init_totaldur;
    }
    else
    {
      Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
      Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
      innerPs.resize(3, 1);
      innerPs = (start_pt + local_target_pt) / 2 +
                (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
                    horizen_dir * 0.8 * (-0.978 / (fail_count + 0.989) + 0.989) +
                (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
                    vertical_dir * 0.4 * (-0.978 / (fail_count + 0.989) + 0.989);
      piece_nums = 2;
      piece_dur_vec.resize(2);
      piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
    }

    MINCOTraj3D initOfInitTraj = generateMINCOTraj(headState, tailState, innerPs, piece_dur_vec);
    double dist = (headState.col(0) - tailState.col(0)).norm();
    piece_nums = std::round(dist / resources_.plan_params->polyTraj_piece_length);
    if (piece_nums < 2)
      piece_nums = 2;
    double piece_dur = init_of_init_totaldur / (double)piece_nums;
    piece_dur_vec.resize(piece_nums);
    piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
    innerPs.resize(3, piece_nums - 1);

    int id = 0;
    double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
    for (double t = t_s; t < t_e; t += piece_dur)
    {
      innerPs.col(id++) = initOfInitTraj.evaluate(t, 0);
    }
    if (id != piece_nums - 1)
    {
      return false;
    }

    initTraj = generateMINCOTraj(headState, tailState, innerPs, piece_dur_vec);
    outInnerPts = innerPs;
    outDurations = piece_dur_vec;
  }
  else
  {
    const double current_glb_t =
        (planning_context != nullptr && planning_context->has_local_target_progress_preview)
            ? planning_context->preview_glb_t_of_lc_tgt
            : resources_.traj_container->global_traj.glb_t_of_lc_tgt;
    const double previous_glb_t =
        (planning_context != nullptr && planning_context->has_local_target_progress_preview)
            ? planning_context->preview_last_glb_t_of_lc_tgt
            : resources_.traj_container->global_traj.last_glb_t_of_lc_tgt;

    if (previous_glb_t < 0.0)
    {
      return false;
    }

    double passed_t_on_lctraj = ros::Time::now().toSec() - resources_.traj_container->local_traj.start_time;
    double t_to_lc_end = resources_.traj_container->local_traj.duration - passed_t_on_lctraj;
    if (t_to_lc_end < 0)
    {
      return false;
    }

    double t_to_lc_tgt = t_to_lc_end +
                         (current_glb_t - previous_glb_t);
    double dist = (start_pt - local_target_pt).norm();
    int piece_nums = std::ceil(dist / resources_.plan_params->polyTraj_piece_length);
    if (piece_nums < 2)
      piece_nums = 2;

    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(local_target_pt, local_target_vel, Eigen::Vector3d::Zero());

    Eigen::MatrixXd innerPs(3, piece_nums - 1);
    Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);

    double t = piece_dur_vec(0);
    for (int i = 0; i < piece_nums - 1; ++i)
    {
      if (t < t_to_lc_end)
      {
        innerPs.col(i) = resources_.traj_container->local_traj.traj.evaluate(t + passed_t_on_lctraj, 0);
      }
      else if (t <= t_to_lc_tgt)
      {
        double glb_t = t - t_to_lc_end +
                       previous_glb_t -
                       resources_.traj_container->global_traj.global_start_time;
        innerPs.col(i) = resources_.traj_container->global_traj.traj.evaluate(glb_t, 0);
      }
      else
      {
        return false;
      }
      t += piece_dur_vec(i + 1);
    }

    initTraj = generateMINCOTraj(headState, tailState, innerPs, piece_dur_vec);
    outInnerPts = innerPs;
    outDurations = piece_dur_vec;
  }
  return true;
}

bool StateToStateInitializer::improveCorridorSeedByTimeScaling(const MINCOBoundaryState3D &head_state,
                                                               const MINCOBoundaryState3D &tail_state,
                                                               const Eigen::MatrixXd &inner_pts,
                                                               Eigen::VectorXd &durations,
                                                               const spatial_map::PolyhedraH &corridor_hpolys,
                                                               MINCOTraj3D &traj) const
{
  if (!traj.generate(inner_pts, head_state, tail_state, durations))
  {
    return false;
  }
  const auto seedIsFeasible = [&](const MINCOTraj3D &candidate) -> bool
  {
    return resources_.optimizer != nullptr &&
           resources_.optimizer->isTrajectoryCollisionFree(candidate) &&
           resources_.optimizer->isTrajectoryInsideCorridor(candidate, corridor_hpolys, 0.0);
  };

  if (seedIsFeasible(traj))
  {
    return true;
  }

  const Eigen::VectorXd base_durations = durations;
  const std::array<double, 7> scale_candidates{{1.25, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0}};
  for (double scale : scale_candidates)
  {
    const Eigen::VectorXd scaled_durations = base_durations * scale;
    MINCOTraj3D scaled_traj;
    if (!scaled_traj.generate(inner_pts, head_state, tail_state, scaled_durations))
    {
      continue;
    }
    durations = scaled_durations;
    traj = scaled_traj;
    if (seedIsFeasible(traj))
    {
      return true;
    }
  }
  return false;
}

bool StateToStateInitializer::initializePlain(const core::PlanningProblem &problem,
                                              StateToStateInitializationResult &result) const
{
  const core::TaskSpec &task = problem.task;
  const Eigen::Vector3d start_pt = problem.start_boundary.position;
  const Eigen::Vector3d start_vel = problem.start_boundary.velocity;
  const Eigen::Vector3d start_acc = problem.start_boundary.acceleration;
  const Eigen::Vector3d goal_pt = problem.terminal_boundary.position;
  const Eigen::Vector3d goal_vel = problem.terminal_boundary.velocity;
  const Eigen::Vector3d goal_acc = problem.terminal_boundary.acceleration;
  const double ts = resources_.plan_params ? resources_.plan_params->polyTraj_piece_length /
                                             std::max(resources_.plan_params->max_vel_, 0.1)
                                           : 1.0;

  Eigen::Vector3d safe_target_pt = goal_pt;
  if (!sanitizeLocalTarget(goal_pt, safe_target_pt))
  {
    result.failure_type = StateToStateInitFailureType::LOCAL_TARGET_INVALID;
    result.failure_reason = "unable to sanitize compiled local target";
    return false;
  }

  if (!computeInitState(start_pt,
                        start_vel,
                        start_acc,
                        safe_target_pt,
                        goal_vel,
                        task.flag_poly_init,
                        task.flag_random_poly_traj,
                        ts,
                        result.init_traj,
                        result.inner_points,
                        result.durations,
                        result.head_state,
                        result.tail_state,
                        &problem.context))
  {
    result.failure_reason = "failed to build plain initial trajectory with stable helper";
    return false;
  }
  result.tail_state = makeBoundaryState(safe_target_pt,
                                        goal_vel,
                                        goal_acc.allFinite() ? goal_acc : Eigen::Vector3d::Zero());
  result.init_traj = generateMINCOTraj(result.head_state,
                                       result.tail_state,
                                       result.inner_points,
                                       result.durations);

  result.guide_path = problem.references.guide_path;
  result.dense_path = result.guide_path;
  if (result.guide_path.size() >= 2)
  {
    result.guide_path.front() = start_pt;
    result.guide_path.back() = safe_target_pt;
    result.dense_path = result.guide_path;
  }
  result.init_source = "stable_helper";
  result.success = result.init_traj.getTotalDuration() > 1.0e-6;
  return result.success;
}

bool StateToStateInitializer::initializeEsdf(const core::PlanningProblem &problem,
                                             StateToStateInitializationResult &result) const
{
  const core::TaskSpec &task = problem.task;
  const Eigen::Vector3d start_pt = problem.start_boundary.position;
  const Eigen::Vector3d start_vel = problem.start_boundary.velocity;
  const Eigen::Vector3d start_acc = problem.start_boundary.acceleration;
  const Eigen::Vector3d goal_pt = problem.terminal_boundary.position;
  const Eigen::Vector3d goal_vel = problem.terminal_boundary.velocity;
  const Eigen::Vector3d goal_acc = problem.terminal_boundary.acceleration;
  const double ts = resources_.plan_params ? resources_.plan_params->polyTraj_piece_length /
                                             std::max(resources_.plan_params->max_vel_, 0.1)
                                           : 1.0;

  Eigen::Vector3d safe_goal = goal_pt;
  if (problem.references.guide_path.size() >= 2)
  {
    result.init_source = "compiled_hint";
    result.dense_path = problem.references.guide_path;
    result.dense_path.front() = start_pt;
    result.dense_path.back() = safe_goal;
  }
  else if (!prepareLocalAStarPath(start_pt, goal_pt, result.dense_path, safe_goal))
  {
    result.failure_reason = "failed to prepare local A* path for ESDF init";
    return false;
  }

  if (!sparsifyGuidePath(result.dense_path, result.guide_path))
  {
    result.guide_path = result.dense_path;
  }
  if (result.guide_path.size() < 2)
  {
    result.failure_reason = "failed to build stable ESDF guide path";
    return false;
  }

  if (!buildInitStateFromGuidePath(start_pt,
                                   start_vel,
                                   start_acc,
                                   safe_goal,
                                   goal_vel,
                                   result.guide_path,
                                   result.init_traj,
                                   result.inner_points,
                                   result.durations,
                                   result.head_state,
                                   result.tail_state))
  {
    result.failure_reason = "failed to build guide-based ESDF initial trajectory";
    return false;
  }
  result.tail_state = makeBoundaryState(safe_goal,
                                        goal_vel,
                                        goal_acc.allFinite() ? goal_acc : Eigen::Vector3d::Zero());
  result.init_traj = generateMINCOTraj(result.head_state,
                                       result.tail_state,
                                       result.inner_points,
                                       result.durations);

  bool warm_start_used = false;
  if (!task.flag_poly_init)
  {
    MINCOTraj3D warm_traj;
    Eigen::MatrixXd warm_inner_pts;
    Eigen::VectorXd warm_durations;
    MINCOBoundaryState3D warm_head, warm_tail;
    if (computeInitState(start_pt,
                         start_vel,
                         start_acc,
                         safe_goal,
                         goal_vel,
                         false,
                         false,
                         ts,
                         warm_traj,
                         warm_inner_pts,
                         warm_durations,
                         warm_head,
                         warm_tail,
                         &problem.context))
    {
      std::vector<Eigen::Vector3d> warm_anchors;
      if (warm_durations.size() > 0 &&
          resamplePolylineByCount(result.guide_path, warm_durations.size() + 1, warm_anchors))
      {
        Eigen::MatrixXd warm_inner_from_guide(3, std::max(0, static_cast<int>(warm_anchors.size()) - 2));
        for (int i = 1; i + 1 < static_cast<int>(warm_anchors.size()); ++i)
        {
          warm_inner_from_guide.col(i - 1) = warm_anchors[static_cast<std::size_t>(i)];
        }
        MINCOTraj3D mixed_init_traj;
        if (mixed_init_traj.generate(warm_inner_from_guide, result.head_state, result.tail_state, warm_durations))
        {
          result.inner_points = warm_inner_from_guide;
          result.durations = warm_durations;
          result.init_traj = mixed_init_traj;
          warm_start_used = true;
          if (result.init_source == "compiled_hint")
          {
            result.init_source = "mixed";
          }
        }
      }
    }
  }

  result.init_min_sdf = computeTrajectoryMinSdf(result.init_traj);
  const double esdf_tol = resources_.grid_map ? -std::max(0.02, 0.5 * resources_.grid_map->getResolution()) : 0.0;
  result.init_collision_free = result.init_min_sdf >= esdf_tol;
  if (warm_start_used && result.init_source == "stable_helper")
  {
    result.init_source = "mixed";
  }
  result.success = result.init_traj.getTotalDuration() > 1.0e-6;
  return result.success;
}

bool StateToStateInitializer::initializeCorridor(const core::PlanningProblem &problem,
                                                 StateToStateInitializationResult &result) const
{
  const core::TaskSpec &task = problem.task;
  const Eigen::Vector3d start_pt = problem.start_boundary.position;
  const Eigen::Vector3d start_vel = problem.start_boundary.velocity;
  const Eigen::Vector3d start_acc = problem.start_boundary.acceleration;
  const Eigen::Vector3d goal_pt = problem.terminal_boundary.position;
  const Eigen::Vector3d goal_vel = problem.terminal_boundary.velocity;
  const Eigen::Vector3d goal_acc = problem.terminal_boundary.acceleration;
  const double ts = resources_.plan_params ? resources_.plan_params->polyTraj_piece_length /
                                             std::max(resources_.plan_params->max_vel_, 0.1)
                                           : 1.0;

  Eigen::Vector3d safe_target_pt = goal_pt;
  if (!sanitizeLocalTarget(goal_pt, safe_target_pt))
  {
    result.failure_type = StateToStateInitFailureType::LOCAL_TARGET_INVALID;
    result.failure_reason = "unable to sanitize compiled local target";
    return false;
  }

  result.head_state = makeBoundaryState(start_pt, start_vel, start_acc);
  result.tail_state = makeBoundaryState(safe_target_pt,
                                        goal_vel,
                                        goal_acc.allFinite() ? goal_acc : Eigen::Vector3d::Zero());

  const auto *corridor_set = [&problem]() -> const core::FeasibleSetSpec *
  {
    for (const auto &set : problem.feasible_sets)
    {
      if (set.enabled &&
          set.type == core::FeasibleSetType::CORRIDOR_POLYTOPE &&
          !set.corridor.empty())
      {
        return &set;
      }
    }
    return nullptr;
  }();

  auto tryCorridorInitialization =
      [&](const std::vector<Eigen::Vector3d> &candidate_guide_path,
          const std::vector<Eigen::Vector3d> &candidate_dense_path,
          const spatial_map::PolyhedraH &candidate_corridor,
          const Eigen::VectorXi &candidate_piece_idx,
          const std::string &candidate_source,
          bool authoritative_failure) -> bool
  {
    std::vector<Eigen::Vector3d> transition_points;
    std::vector<double> inner_clearances;
    Eigen::MatrixXd candidate_inner_pts;
    Eigen::VectorXd candidate_durations;
    Eigen::VectorXi candidate_alloc = candidate_piece_idx;
    MINCOTraj3D candidate_traj;

    if (!buildCorridorAwareInitialGuess(start_pt,
                                        start_vel,
                                        safe_target_pt,
                                        candidate_corridor,
                                        candidate_inner_pts,
                                        candidate_durations,
                                        candidate_alloc,
                                        transition_points,
                                        inner_clearances))
    {
      if (authoritative_failure)
      {
        result.failure_type = StateToStateInitFailureType::CORRIDOR_INIT;
        result.failure_reason = "failed to build GCOPTER-style corridor initial guess";
      }
      return false;
    }

    bool warm_timing_used = false;
    if (!task.flag_poly_init)
    {
      MINCOTraj3D warm_traj;
      Eigen::MatrixXd warm_inner_pts;
      Eigen::VectorXd warm_durations;
      MINCOBoundaryState3D warm_head, warm_tail;
      if (computeInitState(start_pt,
                           start_vel,
                           start_acc,
                           safe_target_pt,
                           goal_vel,
                           false,
                           false,
                           ts,
                           warm_traj,
                           warm_inner_pts,
                           warm_durations,
                           warm_head,
                           warm_tail,
                           &problem.context) &&
          applyWarmStartTimingProfile(warm_durations, candidate_durations))
      {
        warm_timing_used = true;
      }
    }

    if (!candidate_traj.generate(candidate_inner_pts, result.head_state, result.tail_state, candidate_durations) ||
        candidate_traj.getTotalDuration() <= 1.0e-6)
    {
      if (authoritative_failure)
      {
        result.failure_type = StateToStateInitFailureType::CORRIDOR_INIT;
        result.failure_reason = "failed to generate corridor-seeded MINCO trajectory";
      }
      return false;
    }

    const bool time_scaling_feasible = improveCorridorSeedByTimeScaling(result.head_state,
                                                                        result.tail_state,
                                                                        candidate_inner_pts,
                                                                        candidate_durations,
                                                                        candidate_corridor,
                                                                        candidate_traj);
    const bool collision_free = resources_.optimizer &&
                                resources_.optimizer->isTrajectoryCollisionFree(candidate_traj);
    const bool inside_corridor = resources_.optimizer &&
                                 resources_.optimizer->isTrajectoryInsideCorridor(candidate_traj,
                                                                                  candidate_corridor,
                                                                                  0.0);
    if (!collision_free || !inside_corridor)
    {
      if (authoritative_failure)
      {
        result.failure_type = StateToStateInitFailureType::CORRIDOR_INIT;
        result.failure_reason = "corridor seed remains infeasible after conservative timing/scaling";
      }
      return false;
    }

    result.init_source = candidate_source;
    result.guide_path = candidate_guide_path;
    result.dense_path = candidate_dense_path.empty() ? candidate_guide_path : candidate_dense_path;
    result.corridor_hpolys = candidate_corridor;
    result.corridor_piece_idx = candidate_alloc;
    result.inner_points = candidate_inner_pts;
    result.durations = candidate_durations;
    result.init_traj = candidate_traj;
    result.corridor_warm_timing_used = warm_timing_used;
    result.corridor_time_scaling_feasible = time_scaling_feasible;
    result.init_collision_free = collision_free;
    result.init_inside_corridor = inside_corridor;
    result.init_min_sdf = computeTrajectoryMinSdf(candidate_traj);
    return true;
  };

  if (corridor_set != nullptr && !corridor_set->corridor.empty())
  {
    result.compiler_hint_attempted = true;
    std::vector<Eigen::Vector3d> hint_guide =
        corridor_set->corridor_seed_path.size() >= 2 ? corridor_set->corridor_seed_path
                                                     : problem.references.guide_path;
    if (hint_guide.size() >= 2)
    {
      hint_guide.front() = start_pt;
      hint_guide.back() = safe_target_pt;
    }
    result.compiler_hint_succeeded = tryCorridorInitialization(hint_guide,
                                                               hint_guide,
                                                               corridor_set->corridor,
                                                               corridor_set->corridor_piece_idx,
                                                               "compiled_hint",
                                                               false);
  }
  else if (problem.references.guide_path.size() >= 2)
  {
    result.compiler_hint_attempted = true;
    std::vector<Eigen::Vector3d> hint_guide = problem.references.guide_path;
    hint_guide.front() = start_pt;
    hint_guide.back() = safe_target_pt;

    std::vector<Eigen::Vector3d> sparse_hint_guide = hint_guide;
    spatial_map::PolyhedraH hint_corridor;
    if (sparsifyGuidePath(sparse_hint_guide, sparse_hint_guide) &&
        generateSafeFlightCorridor(sparse_hint_guide, hint_corridor))
    {
      hint_guide = sparse_hint_guide;
    }
    else if (!generateSafeFlightCorridor(hint_guide, hint_corridor))
    {
      hint_corridor.clear();
    }

    if (!hint_corridor.empty())
    {
      result.compiler_hint_succeeded = tryCorridorInitialization(hint_guide,
                                                                 hint_guide,
                                                                 hint_corridor,
                                                                 Eigen::VectorXi(),
                                                                 "mixed",
                                                                 false);
    }
  }

  if (result.compiler_hint_succeeded)
  {
    result.success = true;
    return true;
  }

  result.stable_helper_attempted = true;
  std::vector<Eigen::Vector3d> guide_path;
  spatial_map::PolyhedraH corridor_hpolys;
  std::vector<Eigen::Vector3d> dense_path;
  if (!prepareLocalGuideAndCorridor(start_pt, start_vel, safe_target_pt, guide_path, corridor_hpolys, &dense_path))
  {
    result.failure_type = StateToStateInitFailureType::CORRIDOR_GENERATION;
    result.failure_reason = "failed to build corridor guide/corridor from stable helper";
    return false;
  }

  result.stable_helper_succeeded = tryCorridorInitialization(guide_path,
                                                             dense_path,
                                                             corridor_hpolys,
                                                             Eigen::VectorXi(),
                                                             result.compiler_hint_attempted ? "mixed" : "stable_helper",
                                                             true);
  result.success = result.stable_helper_succeeded;
  return result.success;
}

bool StateToStateInitializer::initialize(const core::PlanningProblem &problem,
                                         StateToStateInitializationResult &result) const
{
  result = StateToStateInitializationResult{};
  result.selected_mode = problem.active_space_model;

  bool ok = false;

  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    ok = initializeCorridor(problem, result);
    break;
  case core::ActiveSpaceModel::ESDF:
    ok = initializeEsdf(problem, result);
    break;
  case core::ActiveSpaceModel::PLAIN:
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  default:
    ok = initializePlain(problem, result);
    break;
  }

  if (ok)
  {
    if (result.message.empty())
    {
      result.message = "initialized via " + result.init_source;
    }
  }
  else if (result.message.empty())
  {
    result.message = result.failure_reason;
  }

  return ok;
}

} // namespace ego_planner::solver
