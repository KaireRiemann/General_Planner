#include <plan_manage/planner_manager.h>
#include <SFCGenerator/sfc_gen.hpp>
#include <SFCGenerator/geo_utils.hpp>
#include <SFCGenerator/quickhull.hpp>
#include "visualization_msgs/Marker.h"

#include <array>
#include <cmath>
#include <limits>
#include <set>

namespace ego_planner
{
  namespace
  {
    struct EdgeLess
    {
      bool operator()(const std::pair<int, int> &lhs, const std::pair<int, int> &rhs) const
      {
        return lhs.first < rhs.first || (lhs.first == rhs.first && lhs.second < rhs.second);
      }
    };

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

    void appendCorridorVisualization(const Eigen::MatrixX4d &hpoly,
                                     std::vector<Eigen::Vector3d> &triangle_vertices,
                                     std::vector<Eigen::Vector3d> &edge_vertices)
    {
      Eigen::Matrix3Xd vpoly;
      if (!geo_utils::enumerateVs(hpoly, vpoly) || vpoly.cols() < 4)
      {
        return;
      }

      quickhull::QuickHull<double> qh;
      const double qhull_eps = std::min(1.0e-6, quickhull::defaultEps<double>());
      const auto hull = qh.getConvexHull(vpoly.data(),
                                         static_cast<std::size_t>(vpoly.cols()),
                                         true,
                                         true,
                                         qhull_eps);
      const auto &indices = hull.getIndexBuffer();
      if (indices.size() < 3)
      {
        return;
      }

      std::set<std::pair<int, int>, EdgeLess> unique_edges;
      for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
      {
        const int ia = static_cast<int>(indices[i]);
        const int ib = static_cast<int>(indices[i + 1]);
        const int ic = static_cast<int>(indices[i + 2]);

        triangle_vertices.emplace_back(vpoly.col(ia));
        triangle_vertices.emplace_back(vpoly.col(ib));
        triangle_vertices.emplace_back(vpoly.col(ic));

        unique_edges.emplace(std::min(ia, ib), std::max(ia, ib));
        unique_edges.emplace(std::min(ib, ic), std::max(ib, ic));
        unique_edges.emplace(std::min(ic, ia), std::max(ic, ia));
      }

      for (const auto &edge : unique_edges)
      {
        edge_vertices.emplace_back(vpoly.col(edge.first));
        edge_vertices.emplace_back(vpoly.col(edge.second));
      }
    }

    void buildCorridorVisualization(const spatial_map::PolyhedraH &corridor_hpolys,
                                    std::vector<Eigen::Vector3d> &triangle_vertices,
                                    std::vector<Eigen::Vector3d> &edge_vertices)
    {
      triangle_vertices.clear();
      edge_vertices.clear();
      for (const auto &hpoly : corridor_hpolys)
      {
        appendCorridorVisualization(hpoly, triangle_vertices, edge_vertices);
      }
    }

    Eigen::MatrixX4d makeAxisAlignedBox(const Eigen::Vector3d &center,
                                        const Eigen::Vector3d &half_size)
    {
      Eigen::MatrixX4d box(6, 4);
      box.setZero();
      box(0, 0) = 1.0;
      box(0, 3) = -(center.x() + half_size.x());
      box(1, 0) = -1.0;
      box(1, 3) = center.x() - half_size.x();
      box(2, 1) = 1.0;
      box(2, 3) = -(center.y() + half_size.y());
      box(3, 1) = -1.0;
      box(3, 3) = center.y() - half_size.y();
      box(4, 2) = 1.0;
      box(4, 3) = -(center.z() + half_size.z());
      box(5, 2) = -1.0;
      box(5, 3) = center.z() - half_size.z();
      return box;
    }

    Eigen::MatrixXd sampleTrajectoryForDisplay(const MINCOTraj3D &traj,
                                               const double dt)
    {
      const double total_t = traj.getTotalDuration();
      const double clamped_dt = std::max(dt, 1.0e-3);
      const int sample_num =
          std::max(2, static_cast<int>(std::ceil(total_t / clamped_dt)) + 1);

      Eigen::MatrixXd pts(3, sample_num);
      for (int i = 0; i < sample_num; ++i)
      {
        const double ratio = (sample_num <= 1) ? 0.0 :
                             static_cast<double>(i) / static_cast<double>(sample_num - 1);
        pts.col(i) = traj.evaluate(ratio * total_t, 0);
      }
      return pts;
    }

    bool improveCorridorSeedByTimeScaling(const PolyTrajOptimizer *optimizer,
                                          const MINCOBoundaryState3D &head_state,
                                          const MINCOBoundaryState3D &tail_state,
                                          const Eigen::MatrixXd &inner_pts,
                                          Eigen::VectorXd &durations,
                                          const spatial_map::PolyhedraH &corridor_hpolys,
                                          MINCOTraj3D &traj)
    {
      if (!traj.generate(inner_pts, head_state, tail_state, durations))
      {
        return false;
      }

      const auto seedIsFeasible = [&](const MINCOTraj3D &candidate) -> bool
      {
        return optimizer != nullptr &&
               optimizer->isTrajectoryCollisionFree(candidate) &&
               optimizer->isTrajectoryInsideCorridor(candidate, corridor_hpolys, 0.0);
      };

      if (seedIsFeasible(traj))
      {
        return true;
      }

      const Eigen::VectorXd base_durations = durations;
      const std::array<double, 5> scale_candidates{{1.25, 1.5, 2.0, 3.0, 4.0}};
      bool found_feasible = false;

      for (const double scale : scale_candidates)
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
          found_feasible = true;
          break;
        }
      }

      return found_feasible;
    }
  } // namespace


  EGOPlannerManager::EGOPlannerManager() {}
  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("manager/use_sfc_corridor", use_sfc_corridor_, false);
    nh.param("manager/use_esdf", use_esdf_, false);
    nh.param("manager/sfc_path_timeout", sfc_path_timeout_, 0.2);
    nh.param("manager/sfc_progress", sfc_progress_, 0.75);
    nh.param("manager/sfc_range", sfc_range_, 0.8);
    nh.param("manager/sfc_launch_dist", sfc_launch_dist_, 0.8);
    nh.param("manager/sfc_reuse_goal_tol", sfc_reuse_goal_tol_, 0.6);
    nh.param("manager/sfc_corridor_margin", sfc_corridor_margin_, 0.05);
    nh.param("manager/sfc_near_goal_radius", sfc_near_goal_radius_, 0.8);
    nh.param("manager/guide_min_clearance", guide_min_clearance_, 0.35);
    nh.param("manager/guide_push_step", guide_push_step_, 0.08);
    nh.param("manager/guide_push_max_iter", guide_push_max_iter_, 4);
    nh.param("manager/guide_smooth_weight", guide_smooth_weight_, 0.25);
    nh.param("manager/guide_sparse_min_inner", guide_sparse_min_inner_, 3);
    nh.param("manager/guide_sparse_max_inner", guide_sparse_max_inner_, 5);
    nh.param("manager/guide_turn_angle_deg", guide_turn_angle_deg_, 25.0);
    nh.param("manager/warm_start_prefix_time", warm_start_prefix_time_, 0.5);
    nh.param("manager/warm_start_prefix_max_points", warm_start_prefix_max_points_, 4);
    const char *mode_name = use_sfc_corridor_ ? "sfc_corridor" : (use_esdf_ ? "esdf" : "guide_points");
    ROS_INFO("Local planner obstacle mode: %s", mode_name);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    simple_astar_.reset(new SimpleAStar(grid_map_, 0.0));
    simple_astar_->setTimeOut(sfc_path_timeout_);

    local_astar_.reset(new AStar);
    {
      const Eigen::Vector3d low_corner = grid_map_->getUpdatedBoxLow();
      const Eigen::Vector3d high_corner = grid_map_->getUpdatedBoxHigh();
      const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
      const double search_step = std::max(resolution * 2.0, 0.2);
      const double horizontal_radius = std::max(pp_.planning_horizen_, 4.0) + 2.0;
      const double vertical_span = std::max(high_corner.z() - low_corner.z(), 3.0) + 1.0;

      local_astar_pool_size_.x() =
          std::max(21, static_cast<int>(std::ceil(2.0 * horizontal_radius / search_step)) + 1);
      local_astar_pool_size_.y() = local_astar_pool_size_.x();
      local_astar_pool_size_.z() =
          std::max(21, static_cast<int>(std::ceil(vertical_span / search_step)) + 1);

      local_astar_->initGridMap(grid_map_, local_astar_pool_size_);
      local_astar_->setSearchTimeOut(sfc_path_timeout_);
    }

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  void EGOPlannerManager::reportCorridorFailure(CorridorFailureType type,
                                                const std::string &detail)
  {
    last_corridor_failure_type_ = type;

    switch (type)
    {
    case FAIL_NONE:
      last_corridor_failure_tag_ = "NONE";
      return;
    case FAIL_LOCAL_TARGET_INVALID:
      last_corridor_failure_tag_ = "FAIL_LOCAL_TARGET_INVALID";
      break;
    case FAIL_GUIDE_PATH_TOO_CLOSE:
      last_corridor_failure_tag_ = "FAIL_GUIDE_PATH_TOO_CLOSE";
      break;
    case FAIL_CORRIDOR_GENERATION:
      last_corridor_failure_tag_ = "FAIL_CORRIDOR_GENERATION";
      break;
    case FAIL_CORRIDOR_INIT:
      last_corridor_failure_tag_ = "FAIL_CORRIDOR_INIT";
      break;
    case FAIL_CORRIDOR_OPT:
      last_corridor_failure_tag_ = "FAIL_CORRIDOR_OPT";
      break;
    default:
      last_corridor_failure_tag_ = "FAIL_UNKNOWN";
      break;
    }

    if (detail.empty())
    {
      ROS_WARN("%s", last_corridor_failure_tag_.c_str());
    }
    else
    {
      ROS_WARN("%s: %s", last_corridor_failure_tag_.c_str(), detail.c_str());
    }
  }

  bool EGOPlannerManager::sanitizeLocalTarget(const Eigen::Vector3d &raw_target,
                                              Eigen::Vector3d &safe_target) const
  {
    if (!grid_map_)
    {
      safe_target = raw_target;
      return true;
    }

    const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
    const Eigen::Vector3d clamp_low =
        grid_map_->getUpdatedBoxLow() + Eigen::Vector3d::Constant(2.0 * resolution);
    const Eigen::Vector3d clamp_high =
        grid_map_->getUpdatedBoxHigh() - Eigen::Vector3d::Constant(2.0 * resolution);

    safe_target = raw_target.cwiseMax(clamp_low).cwiseMin(clamp_high);
    if (grid_map_->getInflateOccupancy(safe_target) == 0)
    {
      return true;
    }

    const int max_step = std::max(4, static_cast<int>(std::ceil(std::max(sfc_range_, 1.5) / resolution)));
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
            if (grid_map_->getInflateOccupancy(candidate) != 0)
            {
              continue;
            }

            const double clearance =
                estimateObstacleClearance(candidate,
                                          std::max(guide_min_clearance_, 3.0 * resolution),
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

    ROS_WARN("FAIL_LOCAL_TARGET_INVALID");
    return false;
  }

  double EGOPlannerManager::estimateObstacleClearance(const Eigen::Vector3d &pt,
                                                      double search_radius,
                                                      Eigen::Vector3d *push_dir) const
  {
    if (!grid_map_)
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
            Eigen::Vector3d dir(static_cast<double>(dx),
                                static_cast<double>(dy),
                                static_cast<double>(dz));
            dirs.push_back(dir.normalized());
          }
        }
      }
      return dirs;
    }();

    const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
    const double max_radius = std::max(search_radius, resolution);
    Eigen::Vector3d accum = Eigen::Vector3d::Zero();

    if (grid_map_->getInflateOccupancy(pt) != 0)
    {
      for (const auto &dir : kDirs)
      {
        if (grid_map_->getInflateOccupancy(pt + dir * resolution) == 0)
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
        if (grid_map_->getInflateOccupancy(pt + dir * radius) != 0)
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

  void EGOPlannerManager::evaluateGuidePathQuality(const std::vector<Eigen::Vector3d> &path,
                                                   double &min_clearance,
                                                   double &avg_clearance,
                                                   double &path_length) const
  {
    min_clearance = std::numeric_limits<double>::infinity();
    avg_clearance = 0.0;
    path_length = 0.0;

    if (path.empty())
    {
      min_clearance = 0.0;
      return;
    }

    int sample_count = 0;
    const double query_radius = std::max(2.5 * grid_map_->getResolution(), guide_min_clearance_ * 1.5);
    for (std::size_t i = 0; i < path.size(); ++i)
    {
      const double clearance = estimateObstacleClearance(path[i], query_radius, nullptr);
      min_clearance = std::min(min_clearance, clearance);
      avg_clearance += clearance;
      ++sample_count;

      if (i > 0)
      {
        const Eigen::Vector3d delta = path[i] - path[i - 1];
        path_length += delta.norm();
        if (delta.norm() > 1.0e-3)
        {
          const Eigen::Vector3d mid = 0.5 * (path[i] + path[i - 1]);
          const double mid_clearance = estimateObstacleClearance(mid, query_radius, nullptr);
          min_clearance = std::min(min_clearance, mid_clearance);
          avg_clearance += mid_clearance;
          ++sample_count;
        }
      }
    }

    avg_clearance = sample_count > 0 ? avg_clearance / static_cast<double>(sample_count) : 0.0;
    if (!std::isfinite(min_clearance))
    {
      min_clearance = 0.0;
    }
  }

  double EGOPlannerManager::computeTrajectoryMinSdf(const MINCOTraj3D &traj) const
  {
    if (!grid_map_)
    {
      return 0.0;
    }

    const double total_duration = traj.getTotalDuration();
    const double dt = std::max(0.01, std::min(0.05, grid_map_->getResolution() / std::max(pp_.max_vel_, 0.1)));
    double min_sdf = std::numeric_limits<double>::infinity();

    for (double t = 0.0; t <= total_duration + 1.0e-6; t += dt)
    {
      const double sample_t = std::min(t, total_duration);
      const Eigen::Vector3d pt = traj.evaluate(sample_t, 0);
      if (grid_map_->esdfEnabled())
      {
        min_sdf = std::min(min_sdf, grid_map_->getDistance(pt));
      }
      else
      {
        min_sdf = std::min(min_sdf,
                           estimateObstacleClearance(pt,
                                                     std::max(guide_min_clearance_, 3.0 * grid_map_->getResolution()),
                                                     nullptr));
      }
    }

    return std::isfinite(min_sdf) ? min_sdf : 0.0;
  }

  bool EGOPlannerManager::sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
                                          std::vector<Eigen::Vector3d> &sparse_path) const
  {
    sparse_path.clear();
    if (dense_path.size() < 2)
    {
      return false;
    }

    sparse_path.push_back(dense_path.front());

    std::vector<double> accum_len(dense_path.size(), 0.0);
    for (std::size_t i = 1; i < dense_path.size(); ++i)
    {
      accum_len[i] = accum_len[i - 1] + (dense_path[i] - dense_path[i - 1]).norm();
    }
    const double total_len = accum_len.back();

    int desired_inner = std::max(guide_sparse_min_inner_, 3);
    desired_inner = std::min(desired_inner, guide_sparse_max_inner_);
    desired_inner = std::min(desired_inner, static_cast<int>(dense_path.size()) - 2);
    if (total_len < std::max(1.0, 1.5 * pp_.polyTraj_piece_length))
    {
      desired_inner = std::min(desired_inner, 2);
    }

    std::vector<std::pair<double, int>> turning_candidates;
    for (int i = 1; i + 1 < static_cast<int>(dense_path.size()); ++i)
    {
      const Eigen::Vector3d vin = dense_path[i] - dense_path[i - 1];
      const Eigen::Vector3d vout = dense_path[i + 1] - dense_path[i];
      if (vin.norm() < 1.0e-3 || vout.norm() < 1.0e-3)
        continue;

      const double cos_ang = std::max(-1.0, std::min(1.0,
          vin.normalized().dot(vout.normalized())));
      const double ang_deg = std::acos(cos_ang) * 180.0 / M_PI;
      if (ang_deg >= guide_turn_angle_deg_)
      {
        turning_candidates.emplace_back(ang_deg, i);
      }
    }

    std::sort(turning_candidates.begin(), turning_candidates.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });

    std::set<int> chosen_ids;
    for (const auto &cand : turning_candidates)
    {
      if (static_cast<int>(chosen_ids.size()) >= desired_inner)
        break;
      chosen_ids.insert(cand.second);
    }

    for (int k = 1; static_cast<int>(chosen_ids.size()) < desired_inner && k <= desired_inner; ++k)
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
      chosen_ids.insert(best_idx);
    }

    for (int idx : chosen_ids)
    {
      sparse_path.push_back(dense_path[static_cast<std::size_t>(idx)]);
    }

    std::sort(sparse_path.begin() + 1, sparse_path.end(),
              [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
              {
                auto findNearestS = [&](const Eigen::Vector3d &p)
                {
                  double best = 0.0;
                  double best_d = std::numeric_limits<double>::infinity();
                  for (std::size_t i = 0; i < dense_path.size(); ++i)
                  {
                    double d = (dense_path[i] - p).squaredNorm();
                    if (d < best_d)
                    {
                      best_d = d;
                      best = accum_len[i];
                    }
                  }
                  return best;
                };
                return findNearestS(a) < findNearestS(b);
              });

    sparse_path.push_back(dense_path.back());

    ROS_INFO("Guide path sparsify: dense=%zu sparse=%zu", dense_path.size(), sparse_path.size());
    return sparse_path.size() >= 2;
  }

  bool EGOPlannerManager::shapeGuidePathClearance(const std::vector<Eigen::Vector3d> &raw_path,
                                                  std::vector<Eigen::Vector3d> &shaped_path,
                                                  std::vector<double> &point_clearances) const
  {
    shaped_path = raw_path;
    point_clearances.clear();

    if (raw_path.size() < 2 || !grid_map_)
    {
      point_clearances.assign(raw_path.size(), 0.0);
      return raw_path.size() >= 2;
    }

    const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
    const double search_radius = std::max(guide_min_clearance_ * 1.8, 3.0 * resolution);
    const double min_seg_len = std::max(0.15, 0.25 * std::max(pp_.polyTraj_piece_length, 0.2));
    const Eigen::Vector3d clamp_low = grid_map_->getUpdatedBoxLow() + Eigen::Vector3d::Constant(2.0 * resolution);
    const Eigen::Vector3d clamp_high = grid_map_->getUpdatedBoxHigh() - Eigen::Vector3d::Constant(2.0 * resolution);

    const auto clampPoint = [&](const Eigen::Vector3d &pt) -> Eigen::Vector3d
    {
      return pt.cwiseMax(clamp_low).cwiseMin(clamp_high);
    };

    const auto isSafeSegment = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) -> bool
    {
      return sfc_gen::lineOfSight(a, b, grid_map_.get(), 0.5 * resolution);
    };

    for (std::size_t i = 1; i + 1 < shaped_path.size(); ++i)
    {
      Eigen::Vector3d point = shaped_path[i];
      for (int iter = 0; iter < guide_push_max_iter_; ++iter)
      {
        Eigen::Vector3d push_dir = Eigen::Vector3d::Zero();
        const double clearance = estimateObstacleClearance(point, search_radius, &push_dir);
        if (clearance >= guide_min_clearance_)
        {
          break;
        }

        const Eigen::Vector3d tangent = shaped_path[i + 1] - shaped_path[i - 1];
        if (tangent.norm() > 1.0e-6)
        {
          const Eigen::Vector3d tangent_dir = tangent.normalized();
          push_dir -= tangent_dir * push_dir.dot(tangent_dir) * 0.8;
        }

        if (push_dir.norm() < 1.0e-6)
        {
          Eigen::Vector3d ortho(-(shaped_path[i + 1] - shaped_path[i - 1]).y(),
                                (shaped_path[i + 1] - shaped_path[i - 1]).x(),
                                0.0);
          if (ortho.norm() > 1.0e-6)
          {
            push_dir = ortho.normalized();
          }
        }

        if (push_dir.norm() < 1.0e-6)
        {
          break;
        }

        bool accepted = false;
        double step = std::max(guide_push_step_, 0.5 * resolution);
        for (int trial = 0; trial < 3 && !accepted; ++trial)
        {
          const Eigen::Vector3d candidate = clampPoint(point + step * push_dir.normalized());
          if (grid_map_->getInflateOccupancy(candidate) == 0 &&
              isSafeSegment(shaped_path[i - 1], candidate) &&
              isSafeSegment(candidate, shaped_path[i + 1]))
          {
            point = candidate;
            accepted = true;
          }
          else
          {
            step *= 0.5;
          }
        }

        if (!accepted)
        {
          break;
        }
      }
      shaped_path[i] = point;
    }

    for (int pass = 0; pass < 2; ++pass)
    {
      std::vector<Eigen::Vector3d> smoothed = shaped_path;
      for (std::size_t i = 1; i + 1 < shaped_path.size(); ++i)
      {
        const Eigen::Vector3d candidate =
            clampPoint((1.0 - guide_smooth_weight_) * shaped_path[i] +
                       0.5 * guide_smooth_weight_ * (shaped_path[i - 1] + shaped_path[i + 1]));
        if ((candidate - shaped_path[i - 1]).norm() < min_seg_len ||
            (shaped_path[i + 1] - candidate).norm() < min_seg_len)
        {
          continue;
        }
        if (grid_map_->getInflateOccupancy(candidate) == 0 &&
            isSafeSegment(shaped_path[i - 1], candidate) &&
            isSafeSegment(candidate, shaped_path[i + 1]))
        {
          smoothed[i] = candidate;
        }
      }
      shaped_path.swap(smoothed);
    }

    point_clearances.resize(shaped_path.size(), 0.0);
    for (std::size_t i = 0; i < shaped_path.size(); ++i)
    {
      point_clearances[i] = estimateObstacleClearance(shaped_path[i], search_radius, nullptr);
    }

    shaped_path.front() = raw_path.front();
    shaped_path.back() = raw_path.back();
    return shaped_path.size() >= 2;
  }

  bool EGOPlannerManager::findCorridorTransitionPoint(const spatial_map::PolyhedronH &left_poly,
                                                      const spatial_map::PolyhedronH &right_poly,
                                                      const std::vector<Eigen::Vector3d> &guide_path,
                                                      std::size_t &search_cursor,
                                                      Eigen::Vector3d &transition_point) const
  {
    if (guide_path.size() < 2)
    {
      return false;
    }

    const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
    const std::size_t start_seg = std::min(search_cursor, guide_path.size() - 2);
    for (std::size_t seg = start_seg; seg + 1 < guide_path.size(); ++seg)
    {
      const Eigen::Vector3d &a = guide_path[seg];
      const Eigen::Vector3d &b = guide_path[seg + 1];
      const int sample_num = std::max(2, static_cast<int>(std::ceil((b - a).norm() / resolution)));
      for (int i = 0; i <= sample_num; ++i)
      {
        const double ratio = static_cast<double>(i) / static_cast<double>(sample_num);
        const Eigen::Vector3d candidate = a + ratio * (b - a);
        if (pointInsidePolytope(candidate, left_poly, 0.0) &&
            pointInsidePolytope(candidate, right_poly, 0.0))
        {
          transition_point = candidate;
          search_cursor = seg;
          return true;
        }
      }
    }

    Eigen::MatrixX4d overlap_h(left_poly.rows() + right_poly.rows(), 4);
    overlap_h.topRows(left_poly.rows()) = left_poly;
    overlap_h.bottomRows(right_poly.rows()) = right_poly;
    if (geo_utils::findInterior(overlap_h, transition_point))
    {
      return true;
    }

    Eigen::Vector3d left_inner = Eigen::Vector3d::Zero();
    Eigen::Vector3d right_inner = Eigen::Vector3d::Zero();
    if (!geo_utils::findInterior(left_poly, left_inner) ||
        !geo_utils::findInterior(right_poly, right_inner))
    {
      return false;
    }

    double best_violation = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point = left_inner;
    for (int i = 0; i <= 40; ++i)
    {
      const double ratio = static_cast<double>(i) / 40.0;
      const Eigen::Vector3d candidate = left_inner + ratio * (right_inner - left_inner);
      double violation = -std::numeric_limits<double>::infinity();

      for (int r = 0; r < left_poly.rows(); ++r)
      {
        violation = std::max(violation,
                             left_poly.row(r).head<3>().dot(candidate) + left_poly(r, 3));
      }
      for (int r = 0; r < right_poly.rows(); ++r)
      {
        violation = std::max(violation,
                             right_poly.row(r).head<3>().dot(candidate) + right_poly(r, 3));
      }

      if (violation < best_violation)
      {
        best_violation = violation;
        best_point = candidate;
      }
    }

    if (best_violation <= std::max(grid_map_->getResolution(), 0.05))
    {
      transition_point = best_point;
      return true;
    }

    return false;
  }

  bool EGOPlannerManager::assembleInitialGuessFromAnchors(const std::vector<Eigen::Vector3d> &anchors,
                                                          Eigen::MatrixXd &inner_pts,
                                                          Eigen::VectorXd &durations,
                                                          std::vector<double> *inner_clearances) const
  {
    if (anchors.size() < 2)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> expanded;
    expanded.reserve(anchors.size() * 2);
    expanded.push_back(anchors.front());

    const double piece_length = std::max(pp_.polyTraj_piece_length, 0.2);
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
      durations(i) = std::max(seg_len / std::max(pp_.max_vel_, 0.1), 0.2);
      if (i < piece_num - 1)
      {
        inner_pts.col(i) = expanded[i + 1];
        if (inner_clearances != nullptr)
        {
          inner_clearances->push_back(
              estimateObstacleClearance(expanded[i + 1],
                                        std::max(guide_min_clearance_ * 1.5, 2.0 * grid_map_->getResolution()),
                                        nullptr));
        }
      }
    }

    return true;
  }

  bool EGOPlannerManager::buildInitStateFromGuidePath(const Eigen::Vector3d &start_pt,
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

  bool EGOPlannerManager::buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                                         const Eigen::Vector3d &goal_pt,
                                                         const std::vector<Eigen::Vector3d> &guide_path,
                                                         const spatial_map::PolyhedraH &corridor_hpolys,
                                                         Eigen::MatrixXd &inner_pts,
                                                         Eigen::VectorXd &durations,
                                                         std::vector<Eigen::Vector3d> &transition_points,
                                                         std::vector<double> &inner_clearances) const
  {
    transition_points.clear();
    inner_clearances.clear();
    if (corridor_hpolys.empty())
    {
      return false;
    }

    std::vector<Eigen::Vector3d> anchors;
    anchors.reserve(corridor_hpolys.size() + 2);
    anchors.push_back(start_pt);

    std::size_t search_cursor = 0;
    const double min_anchor_spacing = std::max(0.25, 0.5 * std::max(pp_.polyTraj_piece_length, 0.2));

    for (std::size_t i = 0; i + 1 < corridor_hpolys.size(); ++i)
    {
      Eigen::Vector3d transition = Eigen::Vector3d::Zero();
      if (!findCorridorTransitionPoint(corridor_hpolys[i],
                                       corridor_hpolys[i + 1],
                                       guide_path,
                                       search_cursor,
                                       transition))
      {
        return false;
      }

      if ((transition - anchors.back()).norm() < min_anchor_spacing)
      {
        continue;
      }

      if ((goal_pt - transition).norm() < 0.15)
      {
        continue;
      }

      transition_points.push_back(transition);
      anchors.push_back(transition);
    }

    if ((goal_pt - anchors.back()).norm() < 1.0e-3)
    {
      return false;
    }
    anchors.push_back(goal_pt);

    return assembleInitialGuessFromAnchors(anchors, inner_pts, durations, &inner_clearances);
  }

  bool EGOPlannerManager::buildPrefixWarmStartFromCurrentTraj(const Eigen::Vector3d &start_pt,
                                                              const Eigen::Vector3d &goal_pt,
                                                              const spatial_map::PolyhedraH &corridor_hpolys,
                                                              const std::vector<Eigen::Vector3d> &transition_points,
                                                              Eigen::MatrixXd &inner_pts,
                                                              Eigen::VectorXd &durations,
                                                              std::vector<Eigen::Vector3d> &prefix_points,
                                                              std::vector<double> &inner_clearances) const
  {
    prefix_points.clear();
    inner_clearances.clear();

    if (traj_.local_traj.duration <= 1.0e-3)
    {
      return false;
    }

    const double t_now = ros::Time::now().toSec();
    const double t_cur = t_now - traj_.local_traj.start_time;
    if (t_cur >= traj_.local_traj.duration - 0.05)
    {
      return false;
    }

    const double prefix_horizon = std::min(std::max(warm_start_prefix_time_, 0.1),
                                           traj_.local_traj.duration - t_cur);
    if (prefix_horizon <= 0.05)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> prefix_anchors;
    std::vector<double> prefix_durations;
    prefix_anchors.push_back(start_pt);

    double prev_t = t_cur;
    const int max_points = std::max(1, warm_start_prefix_max_points_);
    const double min_anchor_spacing = std::max(0.15, 0.35 * std::max(pp_.polyTraj_piece_length, 0.2));

    for (int i = 1; i <= max_points; ++i)
    {
      double sample_t = t_cur + prefix_horizon * static_cast<double>(i) / static_cast<double>(max_points);
      sample_t = std::min(sample_t, traj_.local_traj.duration);
      const Eigen::Vector3d sample_pt = traj_.local_traj.traj.evaluate(sample_t, 0);

      if (grid_map_->getInflateOccupancy(sample_pt) != 0 ||
          !pointInsideCorridor(sample_pt, corridor_hpolys, 0.0))
      {
        break;
      }

      if ((sample_pt - prefix_anchors.back()).norm() < min_anchor_spacing)
      {
        continue;
      }

      prefix_points.push_back(sample_pt);
      prefix_anchors.push_back(sample_pt);
      prefix_durations.push_back(std::max(sample_t - prev_t, 0.1));
      prev_t = sample_t;
    }

    if (prefix_durations.empty())
    {
      return false;
    }

    std::vector<Eigen::Vector3d> tail_anchors;
    tail_anchors.push_back(prefix_anchors.back());
    double last_goal_dist = (goal_pt - tail_anchors.back()).norm();
    const double min_tail_spacing = std::max(0.2, 0.4 * std::max(pp_.polyTraj_piece_length, 0.2));

    for (const auto &transition : transition_points)
    {
      if ((transition - tail_anchors.back()).norm() < min_tail_spacing)
      {
        continue;
      }

      const double goal_dist = (goal_pt - transition).norm();
      if (goal_dist + 0.05 >= last_goal_dist)
      {
        continue;
      }

      tail_anchors.push_back(transition);
      last_goal_dist = goal_dist;
    }

    if ((goal_pt - tail_anchors.back()).norm() < 1.0e-3)
    {
      return false;
    }
    tail_anchors.push_back(goal_pt);

    Eigen::MatrixXd tail_inner_pts;
    Eigen::VectorXd tail_durations;
    std::vector<double> tail_clearances;
    if (!assembleInitialGuessFromAnchors(tail_anchors, tail_inner_pts, tail_durations, &tail_clearances))
    {
      return false;
    }

    const int total_piece_num = static_cast<int>(prefix_durations.size()) + tail_durations.size();
    durations.resize(total_piece_num);
    for (std::size_t i = 0; i < prefix_durations.size(); ++i)
    {
      durations(static_cast<int>(i)) = prefix_durations[i];
    }
    durations.tail(tail_durations.size()) = tail_durations;

    inner_pts.resize(3, total_piece_num - 1);
    int col = 0;
    for (const auto &pt : prefix_points)
    {
      inner_pts.col(col++) = pt;
      inner_clearances.push_back(
          estimateObstacleClearance(pt,
                                    std::max(guide_min_clearance_ * 1.5, 2.0 * grid_map_->getResolution()),
                                    nullptr));
    }
    for (int i = 0; i < tail_inner_pts.cols(); ++i)
    {
      inner_pts.col(col++) = tail_inner_pts.col(i);
      inner_clearances.push_back(tail_clearances[static_cast<std::size_t>(i)]);
    }

    return col == inner_pts.cols();
  }

  bool EGOPlannerManager::searchLocalGuidePath(
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &goal_pt,
      std::vector<Eigen::Vector3d> &guide_path)
  {
    if (!local_astar_)
    {
      return false;
    }

    const double search_step = std::max(grid_map_->getResolution() * 2.0, 0.2);
    ASTAR_RET ret = local_astar_->AstarSearch(search_step, start_pt, goal_pt);
    if (ret == ASTAR_RET::INIT_ERR)
    {
      const Eigen::Vector3d delta = (goal_pt - start_pt).cwiseAbs();
      const double horizontal_radius =
          std::max(pp_.planning_horizen_, delta.head<2>().norm()) + std::max(2.0 * sfc_range_, 2.0);
      const double vertical_span =
          std::max(grid_map_->getUpdatedBoxHigh().z() - grid_map_->getUpdatedBoxLow().z(),
                   delta.z() + 2.0);

      Eigen::Vector3i retry_pool = local_astar_pool_size_;
      retry_pool.x() =
          std::max(retry_pool.x(), static_cast<int>(std::ceil(2.0 * horizontal_radius / search_step)) + 1);
      retry_pool.y() =
          std::max(retry_pool.y(), static_cast<int>(std::ceil(2.0 * horizontal_radius / search_step)) + 1);
      retry_pool.z() =
          std::max(retry_pool.z(), static_cast<int>(std::ceil(vertical_span / search_step)) + 3);

      if ((retry_pool.array() > local_astar_pool_size_.array()).any())
      {
        local_astar_.reset(new AStar);
        local_astar_->initGridMap(grid_map_, retry_pool);
        local_astar_->setSearchTimeOut(sfc_path_timeout_);
        local_astar_pool_size_ = retry_pool;
        ret = local_astar_->AstarSearch(search_step, start_pt, goal_pt);
      }
    }

    if (ret != ASTAR_RET::SUCCESS)
    {
      return false;
    }

    guide_path = local_astar_->getPath();
    if (guide_path.size() < 2)
    {
      return false;
    }
    guide_path.front() = start_pt;
    guide_path.back() = goal_pt;
    return true;
  }

  bool EGOPlannerManager::generateSafeFlightCorridor(
      const std::vector<Eigen::Vector3d> &guide_path,
      spatial_map::PolyhedraH &corridor_hpolys) const
  {
    corridor_hpolys.clear();
    if (guide_path.size() < 2)
    {
      return false;
    }

    const Eigen::Vector3d low_corner = grid_map_->getUpdatedBoxLow();
    const Eigen::Vector3d high_corner = grid_map_->getUpdatedBoxHigh();

    std::vector<Eigen::Vector3d> occupied_points;
    grid_map_->getInflatedOccupiedPoints(occupied_points);

    sfc_gen::convexCover(guide_path,
                         occupied_points,
                         low_corner,
                         high_corner,
                         sfc_progress_,
                         sfc_range_,
                         corridor_hpolys);
    sfc_gen::shortCut(corridor_hpolys);

    if (corridor_hpolys.empty())
    {
      ROS_WARN("Failed to generate valid corridor data.");
      return false;
    }
    return true;
  }

  Eigen::Vector3d EGOPlannerManager::computeLaunchPoint(const Eigen::Vector3d& start_pt,
                                                      const Eigen::Vector3d& start_vel) const
  {
    const double v_norm = start_vel.norm();
    if (v_norm < 0.3)
      return start_pt;

    const Eigen::Vector3d dir = start_vel / v_norm;
    return start_pt + sfc_launch_dist_ * dir;
  }

  bool EGOPlannerManager::prepareLocalGuidePath(
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &goal_pt,
      std::vector<Eigen::Vector3d> &guide_path)
  {
    (void)start_vel;
    guide_path.clear();
    reportCorridorFailure(FAIL_NONE, "");

    if ((goal_pt - start_pt).norm() < 1.0e-3)
    {
      guide_path = {start_pt, goal_pt};
      return true;
    }

    std::vector<Eigen::Vector3d> dense_path;
    if (!searchLocalGuidePath(start_pt, goal_pt, dense_path))
    {
      reportCorridorFailure(FAIL_GUIDE_PATH_TOO_CLOSE, "local A* guide path search failed");
      return false;
    }

    std::vector<Eigen::Vector3d> shaped_path;
    std::vector<double> point_clearances;
    const bool have_shaped = shapeGuidePathClearance(dense_path, shaped_path, point_clearances);

    double dense_min_clearance = 0.0;
    double dense_avg_clearance = 0.0;
    double dense_length = 0.0;
    double shaped_min_clearance = 0.0;
    double shaped_avg_clearance = 0.0;
    double shaped_length = 0.0;
    evaluateGuidePathQuality(dense_path, dense_min_clearance, dense_avg_clearance, dense_length);
    if (have_shaped)
    {
      evaluateGuidePathQuality(shaped_path, shaped_min_clearance, shaped_avg_clearance, shaped_length);
    }
    else
    {
      shaped_path = dense_path;
      shaped_min_clearance = dense_min_clearance;
      shaped_avg_clearance = dense_avg_clearance;
      shaped_length = dense_length;
    }

    ROS_INFO("Guide path quality: dense[min=%.3f avg=%.3f len=%.3f] shaped[min=%.3f avg=%.3f len=%.3f]",
             dense_min_clearance, dense_avg_clearance, dense_length,
             shaped_min_clearance, shaped_avg_clearance, shaped_length);

    const std::vector<Eigen::Vector3d> &path_for_sparse =
        (shaped_min_clearance > dense_min_clearance + 0.05) ? shaped_path : dense_path;

    if (!sparsifyGuidePath(path_for_sparse, guide_path))
    {
      reportCorridorFailure(FAIL_GUIDE_PATH_TOO_CLOSE, "failed to sparsify local guide path");
      return false;
    }

    //visualization_->displayGlobalPathList(dense_path, 0.08, 0);
    //visualization_->displayInitPathList(guide_path, 0.12, 0);
    return true;
  }

 bool EGOPlannerManager::prepareLocalGuideAndCorridor(
    const Eigen::Vector3d &start_pt,
    const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &goal_pt,
    std::vector<Eigen::Vector3d> &guide_path,
    spatial_map::PolyhedraH &corridor_hpolys,
    bool force_refresh)
  {
    (void)force_refresh;

    guide_path.clear();
    corridor_hpolys.clear();
    reportCorridorFailure(FAIL_NONE, "");

    Eigen::Vector3d safe_goal;
    std::vector<Eigen::Vector3d> dense_path;
    if (!prepareLocalAStarPath(start_pt, goal_pt, dense_path, safe_goal))
    {
      reportCorridorFailure(FAIL_GUIDE_PATH_TOO_CLOSE, "local A* path preparation failed");
      return false;
    }

    if (!sparsifyGuidePath(dense_path, guide_path))
    {
      reportCorridorFailure(FAIL_GUIDE_PATH_TOO_CLOSE, "failed to sparsify local guide path");
      return false;
    }

    MINCOBoundaryState3D headState, tailState;
    Eigen::MatrixXd innerPts;
    Eigen::VectorXd durations;
    MINCOTraj3D initTraj;
    if (!buildInitTrajectoryFromGuidePath(start_pt,
                                          start_vel,
                                          Eigen::Vector3d::Zero(),
                                          safe_goal,
                                          Eigen::Vector3d::Zero(),
                                          guide_path,
                                          initTraj,
                                          innerPts,
                                          durations,
                                          headState,
                                          tailState))
    {
      reportCorridorFailure(FAIL_CORRIDOR_INIT, "failed to build init trajectory for corridor seed");
      return false;
    }

    std::vector<Eigen::Vector3d> corridor_seed;
    if (!generateCorridorFromInitTrajectory(initTraj, corridor_hpolys, &corridor_seed))
    {
      reportCorridorFailure(FAIL_CORRIDOR_GENERATION, "failed to generate corridor from init trajectory");
      return false;
    }

    corridor_seed_start_ = start_pt;
    corridor_seed_goal_ = safe_goal;

    if (visualization_)
    {
      visualization_->displayGlobalPathList(dense_path, 0.08, 0);
      visualization_->displayFrontendList(guide_path, 0.12, 0);
      //visualization_->displayInitPathList(corridor_seed, 0.06, 1);

      std::vector<Eigen::Vector3d> tri, edges;
      buildCorridorVisualization(corridor_hpolys, tri, edges);
      visualization_->displayCorridor(tri, edges, 0);
    }

    return true;
  }

  bool EGOPlannerManager::buildGuideInitialGuess(
    const std::vector<Eigen::Vector3d> &guide_path,
    Eigen::MatrixXd &inner_pts,
    Eigen::VectorXd &durations) const
  {
    if (guide_path.size() < 2)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> anchors;
    anchors.reserve(guide_path.size() * 2);
    anchors.push_back(guide_path.front());

    const double piece_length = std::max(pp_.polyTraj_piece_length, 0.2);
    for (std::size_t i = 1; i < guide_path.size(); ++i)
    {
      const Eigen::Vector3d &p0 = guide_path[i - 1];
      const Eigen::Vector3d &p1 = guide_path[i];
      const double seg_len = (p1 - p0).norm();
      if (seg_len < 1.0e-4)
      {
        continue;
      }

      const int midpoint_num = std::max(0, static_cast<int>(std::ceil(seg_len / piece_length)) - 1);
      for (int k = 1; k <= midpoint_num; ++k)
      {
        const double ratio = static_cast<double>(k) / static_cast<double>(midpoint_num + 1);
        anchors.push_back(p0 + ratio * (p1 - p0));
      }
      anchors.push_back(p1);
    }

    if (anchors.size() < 2)
    {
      return false;
    }

    return assembleInitialGuessFromAnchors(anchors, inner_pts, durations, nullptr);
  }

  bool EGOPlannerManager::pointInsidePolytope(const Eigen::Vector3d& pt,
                                            const spatial_map::PolyhedronH& hpoly,
                                            double margin) const
  {
    for (int i = 0; i < hpoly.rows(); ++i)
    {
      if (hpoly.row(i).head<3>().dot(pt) + hpoly(i, 3) > -margin)
        return false;
    }
    return true;
  }

  bool EGOPlannerManager::pointInsideCorridor(const Eigen::Vector3d& pt,
                                              const spatial_map::PolyhedraH& corridor,
                                              double margin) const
  {
    for (const auto& poly : corridor)
    {
      if (pointInsidePolytope(pt, poly, margin))
        return true;
    }
    return false;
  }

  //warm start for corridor generate
  bool EGOPlannerManager::buildWarmStartFromCurrentTraj(
    const Eigen::Vector3d& start_pt,
    const Eigen::Vector3d& goal_pt,
    Eigen::MatrixXd& inner_pts,
    Eigen::VectorXd& durations) const
  {
    if (traj_.local_traj.duration <= 1e-3) return false;

    const double t_now = ros::Time::now().toSec();
    const double t_cur = t_now - traj_.local_traj.start_time;
    if (t_cur >= traj_.local_traj.duration - 0.1) return false;

    const int piece_num = std::max(2, static_cast<int>(std::ceil((goal_pt - start_pt).norm() / std::max(pp_.polyTraj_piece_length, 0.2))));
    inner_pts.resize(3, piece_num - 1);
    durations = Eigen::VectorXd::Constant(piece_num, std::max((traj_.local_traj.duration - t_cur) / piece_num, 0.2));

    for (int i = 0; i < piece_num - 1; ++i)
    {
      const double ts = t_cur + (i + 1) * (traj_.local_traj.duration - t_cur) / piece_num;
      inner_pts.col(i) = traj_.local_traj.traj.evaluate(std::min(ts, traj_.local_traj.duration), 0);
    }
    return true;
  }

  // bool EGOPlannerManager::reboundReplan(
  //     const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
  //     const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
  //     const Eigen::Vector3d &local_target_vel, const std::vector<Eigen::Vector3d> &guide_path,
  //     const spatial_map::PolyhedraH &corridor_hpolys, const bool touch_goal)
  // {
  //   ros::Time t_start = ros::Time::now();
  //   ros::Duration t_init, t_opt;

  //   std::cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << replan_seq_++ << "\033[0m" << std::endl;

  //   ploy_traj_opt_->setIfTouchGoal(touch_goal);

  //   MINCOTraj3D initTraj;
  //   Eigen::MatrixXd innerPts;
  //   Eigen::VectorXd durations;
  //   MINCOBoundaryState3D headState, tailState;

  //   const Eigen::Vector3d corridor_target_pt =
  //       guide_path.empty() ? local_target_pt : guide_path.back();
  //   const Eigen::Vector3d corridor_target_vel =
  //       ((corridor_target_pt - local_target_pt).norm() > 1.0e-3) ? Eigen::Vector3d::Zero() : local_target_vel;

  //   headState = makeBoundaryState(start_pt, start_vel, start_acc);
  //   tailState = makeBoundaryState(corridor_target_pt, corridor_target_vel, Eigen::Vector3d::Zero());

  //   ROS_WARN("Corridor endpoint coverage: start=%s tail=%s",
  //            pointInsideCorridor(start_pt, corridor_hpolys, 0.0) ? "yes" : "no",
  //            pointInsideCorridor(corridor_target_pt, corridor_hpolys, 0.0) ? "yes" : "no");

  //   if (!buildInitStateFromGuidePath(start_pt,
  //                                    start_vel,
  //                                    start_acc,
  //                                    corridor_target_pt,
  //                                    corridor_target_vel,
  //                                    guide_path,
  //                                    initTraj,
  //                                    innerPts,
  //                                    durations,
  //                                    headState,
  //                                    tailState))
  //   {
  //     reportCorridorFailure(FAIL_CORRIDOR_INIT, "unable to build corridor initial guess");
  //     return false;
  //   }

  //   const bool init_seed_feasible = improveCorridorSeedByTimeScaling(ploy_traj_opt_.get(),
  //                                                                    headState,
  //                                                                    tailState,
  //                                                                    innerPts,
  //                                                                    durations,
  //                                                                    corridor_hpolys,
  //                                                                    initTraj);
  //   if (initTraj.getTotalDuration() <= 1.0e-6)
  //   {
  //     ROS_ERROR("Failed to generate corridor-seeded MINCO trajectory.");
  //     reportCorridorFailure(FAIL_CORRIDOR_INIT, "seed trajectory generation returned empty trajectory");
  //     return false;
  //   }

  //   // ROS_WARN("Corridor init seed feasibility: inside=%s collision_free=%s scaled_feasible=%s",
  //   //          ploy_traj_opt_->isTrajectoryInsideCorridor(initTraj, corridor_hpolys, 0.0) ? "yes" : "no",
  //   //          ploy_traj_opt_->isTrajectoryCollisionFree(initTraj) ? "yes" : "no",
  //   //          init_seed_feasible ? "yes" : "no");
  //   // ROS_INFO("INIT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
  //   //          ploy_traj_opt_->isTrajectoryCollisionFree(initTraj) ? "yes" : "no",
  //   //          computeTrajectoryMinSdf(initTraj),
  //   //          ploy_traj_opt_->isTrajectoryInsideCorridor(initTraj, corridor_hpolys, 0.0) ? "yes" : "no");

  //   Eigen::MatrixXd cstr_pts = initTraj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
  //   t_init = ros::Time::now() - t_start;

  //   std::vector<Eigen::Vector3d> point_set;
  //   for (int i = 0; i < cstr_pts.cols(); ++i)
  //   {
  //     point_set.push_back(cstr_pts.col(i));
  //   }
  //   visualization_->displayInitPathList(point_set, 0.2, 0);

  //   t_start = ros::Time::now();

  //   double final_cost = 0.0;
  //   const bool flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
  //                                                                innerPts, durations,
  //                                                                corridor_hpolys, final_cost);

  //   t_opt = ros::Time::now() - t_start;

  //   std::cout << "Success=" << (flag_success ? "yes" : "no") << std::endl;
  //   if (flag_success)
  //   {
  //     static double sum_time = 0.0;
  //     static int count_success = 0;
  //     sum_time += (t_init + t_opt).toSec();
  //     count_success++;

  //     MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
  //     ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f inside_corridor=%s",
  //              computeTrajectoryMinSdf(opt_traj),
  //              ploy_traj_opt_->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
  //     setLocalTrajFromOpt(opt_traj, touch_goal);
  //     cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
  //     visualization_->displayOptimalList(cstr_pts, 0);

  //     printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
  //            (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);
  //     continous_failures_count_ = 0;
  //   }
  //   else
  //   {
  //     reportCorridorFailure(FAIL_CORRIDOR_OPT, "corridor optimizer rejected the candidate trajectory");
  //     MINCOTraj3D fail_traj = ploy_traj_opt_->getTrajectory();
  //     ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
  //              ploy_traj_opt_->isTrajectoryCollisionFree(fail_traj) ? "yes" : "no",
  //              computeTrajectoryMinSdf(fail_traj),
  //              ploy_traj_opt_->isTrajectoryInsideCorridor(fail_traj, corridor_hpolys, 0.0) ? "yes" : "no");
  //     cstr_pts = sampleTrajectoryForDisplay(fail_traj, 0.02);
  //     visualization_->displayFailedList(cstr_pts, 0);
  //     continous_failures_count_++;
  //   }

  //   return flag_success;
  // }



  static MINCOTraj3D generateMINCOTraj(
      const MINCOBoundaryState3D &headState,
      const MINCOBoundaryState3D &tailState,
      const Eigen::MatrixXd &innerPts,
      const Eigen::VectorXd &durations)
  {
    MINCOTraj3D traj;
    if (!traj.generate(innerPts, headState, tailState, durations))
    {
      ROS_ERROR("Failed to generate MINCO trajectory. innerPts=[%ld x %ld], duration_size=%ld",
                static_cast<long>(innerPts.rows()),
                static_cast<long>(innerPts.cols()),
                static_cast<long>(durations.size()));
    }
    return traj;
  }

  static double getMaxVelRate(const MINCOTraj3D &traj)
  {
    double maxVel = 0.0;
    double dt = 0.01;
    double t_total = traj.getTotalDuration();
    
    for (double t = 0.0; t <= t_total; t += dt)
    {
      double vel = traj.evaluate(t, 1).norm();
      if (vel > maxVel)
        maxVel = vel;
    }
    return maxVel;
  }

  bool EGOPlannerManager::prepareLocalAStarPath(const Eigen::Vector3d &start_pt,
                                              const Eigen::Vector3d &goal_pt,
                                              std::vector<Eigen::Vector3d> &dense_path,
                                              Eigen::Vector3d &safe_goal) const
  {
    dense_path.clear();
    safe_goal = goal_pt;

    if (!sanitizeLocalTarget(goal_pt, safe_goal))
    {
      ROS_WARN("FAIL_LOCAL_TARGET_INVALID: cannot sanitize local target");
      return false;
    }

    if ((safe_goal - start_pt).norm() < 1.0e-3)
    {
      dense_path = {start_pt, safe_goal};
      return true;
    }

    if (!grid_map_)
    {
      dense_path = {start_pt, safe_goal};
      return true;
    }

    const double step = std::max(grid_map_->getResolution() * 0.5, 0.05);
    if (sfc_gen::lineOfSight(start_pt, safe_goal, grid_map_.get(), step))
    {
      dense_path = {start_pt, safe_goal};
      return true;
    }

    if (!simple_astar_)
    {
      ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: simple_astar_ is null");
      return false;
    }

    if (!simple_astar_->search(start_pt, safe_goal, dense_path))
    {
      ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: 3D A* failed");
      return false;
    }

    if (dense_path.size() < 2)
    {
      ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: 3D A* returned too short path");
      return false;
    }

    return true;
  }

  bool EGOPlannerManager::buildInitTrajectoryFromGuidePath(
    const Eigen::Vector3d &start_pt,
    const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &start_acc,
    const Eigen::Vector3d &target_pt,
    const Eigen::Vector3d &target_vel,
    const std::vector<Eigen::Vector3d> &sparse_path,
    MINCOTraj3D &init_traj,
    Eigen::MatrixXd &innerPts,
    Eigen::VectorXd &durations,
    MINCOBoundaryState3D &headState,
    MINCOBoundaryState3D &tailState)
  {
    if (sparse_path.size() < 2)
    {
      ROS_WARN("buildInitTrajectoryFromGuidePath: sparse_path too short");
      return false;
    }

    std::vector<Eigen::Vector3d> working_path = sparse_path;
    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(target_pt, target_vel, Eigen::Vector3d::Zero());

    auto fillDecisionVariables = [&](const std::vector<Eigen::Vector3d> &path_pts,
                                    Eigen::MatrixXd &pts,
                                    Eigen::VectorXd &durs)
    {
      const int piece_num = static_cast<int>(path_pts.size()) - 1;
      pts.resize(3, std::max(0, piece_num - 1));
      durs.resize(piece_num);

      for (int i = 0; i < piece_num; ++i)
      {
        const Eigen::Vector3d p0 = path_pts[static_cast<std::size_t>(i)];
        const Eigen::Vector3d p1 = path_pts[static_cast<std::size_t>(i + 1)];
        const double seg_len = (p1 - p0).norm();
        durs(i) = std::max(seg_len / std::max(pp_.max_vel_, 0.1), 0.2);

        if (i < piece_num - 1)
        {
          pts.col(i) = p1;
        }
      }
    };

    for (int refine = 0; refine < 3; ++refine)
    {
      fillDecisionVariables(working_path, innerPts, durations);
      if (!init_traj.generate(innerPts, headState, tailState, durations))
      {
        ROS_WARN("buildInitTrajectoryFromGuidePath: MINCO generate failed");
        return false;
      }

      if (!ploy_traj_opt_ || ploy_traj_opt_->isTrajectoryCollisionFree(init_traj))
      {
        ROS_INFO("buildInitTrajectoryFromGuidePath: piece_num=%ld inner=%ld totalT=%.3f",
                static_cast<long>(durations.size()),
                static_cast<long>(innerPts.cols()),
                durations.sum());
        return true;
      }

      if (working_path.size() >= 12)
      {
        break;
      }

      std::vector<Eigen::Vector3d> refined;
      refined.reserve(2 * working_path.size());
      refined.push_back(working_path.front());
      for (std::size_t i = 1; i < working_path.size(); ++i)
      {
        const Eigen::Vector3d &a = working_path[i - 1];
        const Eigen::Vector3d &b = working_path[i];
        if ((b - a).norm() > std::max(grid_map_->getResolution() * 2.0, 0.3))
        {
          refined.push_back(0.5 * (a + b));
        }
        refined.push_back(b);
      }
      working_path.swap(refined);
    }

    ROS_WARN("buildInitTrajectoryFromGuidePath: init traj is still colliding after refinement");
    return false;
  }

  bool EGOPlannerManager::generateCorridorFromInitTrajectory(
    const MINCOTraj3D &init_traj,
    spatial_map::PolyhedraH &corridor_hpolys,
    std::vector<Eigen::Vector3d> *seed_pts) const
  {
    corridor_hpolys.clear();
    if (seed_pts != nullptr)
    {
      seed_pts->clear();
    }

    const double total_t = init_traj.getTotalDuration();
    if (total_t < 1.0e-3)
    {
      ROS_WARN("generateCorridorFromInitTrajectory: init traj too short");
      return false;
    }

    const double sample_dt = std::max(0.05,
        std::min(0.25, pp_.polyTraj_piece_length / std::max(pp_.max_vel_, 0.1)));

    std::vector<Eigen::Vector3d> seed_path;
    seed_path.reserve(static_cast<std::size_t>(std::ceil(total_t / sample_dt)) + 2);

    Eigen::Vector3d last_pt = init_traj.evaluate(0.0, 0);
    seed_path.push_back(last_pt);

    for (double t = sample_dt; t < total_t; t += sample_dt)
    {
      const Eigen::Vector3d pt = init_traj.evaluate(t, 0);
      if ((pt - last_pt).norm() > std::max(grid_map_->getResolution(), 0.08))
      {
        seed_path.push_back(pt);
        last_pt = pt;
      }
    }

    const Eigen::Vector3d end_pt = init_traj.evaluate(total_t, 0);
    if ((end_pt - seed_path.back()).norm() > 1.0e-3)
    {
      seed_path.push_back(end_pt);
    }

    if (seed_pts != nullptr)
    {
      *seed_pts = seed_path;
    }

    if (seed_path.size() < 2)
    {
      ROS_WARN("generateCorridorFromInitTrajectory: not enough seed points");
      return false;
    }

    if (!generateSafeFlightCorridor(seed_path, corridor_hpolys))
    {
      ROS_WARN("generateCorridorFromInitTrajectory: generateSafeFlightCorridor failed");
      return false;
    }

    ROS_INFO("generateCorridorFromInitTrajectory: seed_pts=%zu corridor_polys=%zu",
            seed_path.size(), corridor_hpolys.size());

    return !corridor_hpolys.empty();
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal,
      const bool force_plain)
  {
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    std::cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << replan_seq_++ << "\033[0m" << std::endl;

    /*** STEP 1: INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    MINCOTraj3D initTraj;
    Eigen::MatrixXd innerPts;
    Eigen::VectorXd durations;
    MINCOBoundaryState3D headState, tailState;
    spatial_map::PolyhedraH corridor_hpolys;
    Eigen::Vector3d safe_target_pt = local_target_pt;

    if (!sanitizeLocalTarget(local_target_pt, safe_target_pt))
    {
      reportCorridorFailure(FAIL_LOCAL_TARGET_INVALID, "unable to sanitize local target before planning");
      return false;
    }

    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(safe_target_pt, local_target_vel, Eigen::Vector3d::Zero());

    const bool use_corridor = use_sfc_corridor_ && !force_plain;
    const bool use_esdf = use_esdf_ && !use_corridor && !force_plain;

    if (use_corridor)
    {
      std::vector<Eigen::Vector3d> guide_path;
      if (!prepareLocalGuideAndCorridor(start_pt, start_vel, safe_target_pt, guide_path, corridor_hpolys) ||
          !buildInitStateFromGuidePath(start_pt,
                                       start_vel,
                                       start_acc,
                                       guide_path.back(),
                                       Eigen::Vector3d::Zero(),
                                       guide_path,
                                       initTraj,
                                       innerPts,
                                       durations,
                                       headState,
                                       tailState))
      {
        return false;
      }
      const bool init_seed_feasible = improveCorridorSeedByTimeScaling(ploy_traj_opt_.get(),
                                                                       headState,
                                                                       tailState,
                                                                       innerPts,
                                                                       durations,
                                                                       corridor_hpolys,
                                                                       initTraj);
      if (initTraj.getTotalDuration() <= 1.0e-6)
      {
        ROS_ERROR("Failed to generate corridor-seeded MINCO trajectory.");
        return false;
      }

      ROS_INFO("INIT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
               ploy_traj_opt_->isTrajectoryCollisionFree(initTraj) ? "yes" : "no",
               computeTrajectoryMinSdf(initTraj),
               ploy_traj_opt_->isTrajectoryInsideCorridor(initTraj, corridor_hpolys, 0.0) ? "yes" : "no");
      ROS_INFO("Corridor seed time-scaling feasibility: %s",
               init_seed_feasible ? "yes" : "no");
    }
    else if (use_esdf)
    {
      std::vector<Eigen::Vector3d> guide_path;
      // if (!prepareLocalGuidePath(start_pt, start_vel, safe_target_pt, guide_path))
      // {
      //   return false;
      // }
      // double guide_min_clearance = 0.0;
      // double guide_avg_clearance = 0.0;
      // double guide_length = 0.0;
      // evaluateGuidePathQuality(guide_path, guide_min_clearance, guide_avg_clearance, guide_length);
      // if (guide_min_clearance < 0.3 * guide_min_clearance_)
      // {
      //   ROS_WARN("WARN_GUIDE_PATH_TOO_CLOSE_BUT_CONTINUE_ESDF");
      // }
      Eigen::Vector3d safe_goal = local_target_pt;
      if (!sanitizeLocalTarget(local_target_pt, safe_goal))
      {
        ROS_WARN("FAIL_LOCAL_TARGET_INVALID: cannot sanitize local target");
        return false;
      }
      if(!simple_astar_->search(start_pt,safe_goal,guide_path))
      {
        ROS_WARN("Retry Path Planning!");
        if(!simple_astar_->search(start_pt,safe_goal,guide_path))
        {
          ROS_WARN("Path Searching Failed !");
        }
        else
        {
          ROS_WARN("Retry Success !");
        }
      }
      else
      {
        visualization_->displayFrontendList(guide_path, 0.05,1);
        if (!buildInitStateFromGuidePath(start_pt,
                                       start_vel,
                                       start_acc,
                                       safe_target_pt,
                                       local_target_vel,
                                       guide_path,
                                       initTraj,
                                       innerPts,
                                       durations,
                                       headState,
                                       tailState))
        {
          return false;
        }
        ROS_INFO("INIT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f",
                ploy_traj_opt_->isTrajectoryCollisionFree(initTraj) ? "yes" : "no",
                computeTrajectoryMinSdf(initTraj));
      }
    }
    else
    {
      if (!computeInitState(start_pt, start_vel, start_acc, safe_target_pt, local_target_vel,
                            flag_polyInit, flag_randomPolyTraj, ts,
                            initTraj, innerPts, durations, headState, tailState))
      {
        return false;
      }
    }

    Eigen::MatrixXd cstr_pts = initTraj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    std::vector<std::pair<int, int>> segments;
    if (!use_corridor && !use_esdf)
    {
      if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initTraj, cstr_pts, true) == PolyTrajOptimizer::CHK_RET::ERR)
      {
        return false;
      }
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    std::vector<std::vector<Eigen::Vector3d>> vis_trajs;

    if (use_corridor)
    {
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, durations,
                                                        corridor_hpolys, final_cost);

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f inside_corridor=%s",
                 computeTrajectoryMinSdf(opt_traj),
                 ploy_traj_opt_->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
        setLocalTrajFromOpt(opt_traj, touch_goal);
        cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
        visualization_->displayOptimalList(cstr_pts, 0);
      }
      else
      {
        const MINCOTraj3D &opt_traj = ploy_traj_opt_->getTrajectory();
        ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
                 ploy_traj_opt_->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
                 computeTrajectoryMinSdf(opt_traj),
                 ploy_traj_opt_->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
      }
    }
    else if (use_esdf)
    {
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectoryWithDistanceField(headState, tailState,
                                                                         innerPts, durations, final_cost);

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f",
                 computeTrajectoryMinSdf(opt_traj));
        setLocalTrajFromOpt(opt_traj, touch_goal);
        cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
        visualization_->displayOptimalList(cstr_pts, 0);
      }
      else
      {
        const MINCOTraj3D &opt_traj = ploy_traj_opt_->getTrajectory();
        ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f",
                 ploy_traj_opt_->isTrajectoryCollisionFree(opt_traj) ? "yes" : "no",
                 computeTrajectoryMinSdf(opt_traj));
      }
    }
    else if (pp_.use_multitopology_trajs)
    {
      std::vector<Types::ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      double final_cost, min_cost = 999999.0;
      MINCOTraj3D best_traj;

      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        ploy_traj_opt_->setUseMultitopologyTrajs(true);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, durations, final_cost))
        {
          success[i] = true;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_traj = ploy_traj_opt_->getTrajectory();
            flag_success = true;
          }

          MINCOTraj3D vis_traj = ploy_traj_opt_->getTrajectory();
          Eigen::MatrixXd ctrl_pts_temp = vis_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> vis_pts;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
            vis_pts.push_back(ctrl_pts_temp.col(j));
          vis_trajs.push_back(vis_pts);
        }
      }

      t_opt = ros::Time::now() - t_start;

      if (trajs.size() > 1)
      {
        std::cout << "\033[1;33m" << "multi-trajs=" << trajs.size() << ",\033[1;0m"
                  << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << std::endl;
      }

      visualization_->displayMultiOptimalPathList(vis_trajs, 0.1);

      if (flag_success)
      {
        setLocalTrajFromOpt(best_traj, touch_goal);
        cstr_pts = best_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
        visualization_->displayOptimalList(cstr_pts, 0);
      }
    }
    else
    {
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, durations, final_cost);

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        setLocalTrajFromOpt(opt_traj, touch_goal);
        
        cstr_pts = opt_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
        visualization_->displayOptimalList(cstr_pts, 0);
      }
    }

    /*** STEP 3: Store and display results ***/
    std::cout << "Success=" << (flag_success ? "yes" : "no") << std::endl;
    if (flag_success)
    {
      static double sum_time = 0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).toSec();
      count_success++;
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);

      continous_failures_count_ = 0;
    }
    else
    {
      MINCOTraj3D fail_traj = ploy_traj_opt_->getTrajectory();
      if (use_corridor || use_esdf)
      {
        cstr_pts = sampleTrajectoryForDisplay(fail_traj, 0.02);
      }
      else
      {
        cstr_pts = fail_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      }
      visualization_->displayFailedList(cstr_pts, 0);

      continous_failures_count_++;
    }

    return flag_success;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      MINCOTraj3D &initTraj, Eigen::MatrixXd &outInnerPts, Eigen::VectorXd &outDurations,
      MINCOBoundaryState3D &headState, MINCOBoundaryState3D &tailState)
  {
    static bool flag_first_call = true;

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
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      // Generate init of init trajectory
      MINCOTraj3D initOfInitTraj = generateMINCOTraj(headState, tailState, innerPs, piece_dur_vec);

      // Generate the real init trajectory
      double dist = (headState.col(0) - tailState.col(0)).norm();
      piece_nums = round(dist / pp_.polyTraj_piece_length);
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
        ROS_ERROR("Should not happen! x_x");
        return false;
      }

      initTraj = generateMINCOTraj(headState, tailState, innerPs, piece_dur_vec);

      outInnerPts = innerPs;
      outDurations = piece_dur_vec;
    }
    else
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if (t_to_lc_end < 0)
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      double dist = (start_pt - local_target_pt).norm();
      int piece_nums = ceil(dist / pp_.polyTraj_piece_length);
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
          innerPs.col(i) = traj_.local_traj.traj.evaluate(t + passed_t_on_lctraj, 0);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.evaluate(glb_t, 0);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initTraj = generateMINCOTraj(headState, tailState, innerPs, piece_dur_vec);
      outInnerPts = innerPs;
      outDurations = piece_dur_vec;
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;

    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      double local_t = t - traj_.global_traj.global_start_time;
      Eigen::Vector3d pos_t = traj_.global_traj.traj.evaluate(local_t, 0); // 0 为位置
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5)
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      double local_t = t - traj_.global_traj.global_start_time;
      local_target_vel = traj_.global_traj.traj.evaluate(local_t, 1); 
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const MINCOTraj3D &traj, const bool touch_goal)
  {
    // Eigen::MatrixXd cps = traj.getControlPoints().transpose();
    
    // PtsChk_t pts_to_check;
    // bool ret = ploy_traj_opt_->computePointsToCheck(traj, Types::ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    
    // if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    // {
    //   traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());
    // }

    // return ret;
    traj_.setLocalTraj(traj,ros::Time::now().toSec());
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    MINCOBoundaryState3D headState = makeBoundaryState(stop_pos, ZERO, ZERO);
    MINCOBoundaryState3D tailState = headState;
    Eigen::MatrixXd innerPs = stop_pos; // 3x1
    Eigen::VectorXd durs = Eigen::Vector2d(1.0, 1.0);

    MINCOTraj3D stopTraj = generateMINCOTraj(headState, tailState, innerPs, durs);
    setLocalTrajFromOpt(stopTraj, false);

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9)
      return false;
    if (traj_.swarm_traj[drone_id].drone_id != drone_id)
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = std::max(my_traj_start_time, other_traj_start_time);
    double t_end = std::min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                            other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      Eigen::Vector3d my_pos = traj_.local_traj.traj.evaluate(t - my_traj_start_time, 0);
      Eigen::Vector3d other_pos = traj_.swarm_traj[drone_id].traj.evaluate(t - other_traj_start_time, 0);
      
      if ((my_pos - other_pos).norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance))
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    MINCOBoundaryState3D headState = makeBoundaryState(start_pos, start_vel, start_acc);
    MINCOBoundaryState3D tailState = makeBoundaryState(waypoints.back(), end_vel, end_acc);
    Eigen::MatrixXd innerPts(3, 0);

    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
        innerPts.col(i) = waypoints[i];
    }

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    MINCOTraj3D globalTraj;
    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalTraj = generateMINCOTraj(headState, tailState, innerPts, time_vec);

      if (getMaxVelRate(globalTraj) < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", getMaxVelRate(globalTraj));
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalTraj, time_now.toSec());

    return true;
  }

} // namespace ego_planner
