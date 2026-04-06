#include <plan_manage/planner_manager.h>
#include <SFCGenerator/sfc_gen.hpp>
#include <SFCGenerator/geo_utils.hpp>
#include <SFCGenerator/quickhull.hpp>
#include "visualization_msgs/Marker.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
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

    const char *managerDefaultModeString(const bool use_corridor, const bool use_esdf)
    {
      if (use_corridor)
      {
        return "CORRIDOR";
      }
      if (use_esdf)
      {
        return "ESDF";
      }
      return "PLAIN";
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
      const std::array<double, 7> scale_candidates{{1.25, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0}};
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
        const double target_s = (sample_count == 1)
                                    ? 0.0
                                    : total_len * static_cast<double>(k) / static_cast<double>(sample_count - 1);
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
        const double denom = std::max(1.0e-9, s1 - s0);
        const double alpha = (target_s - s0) / denom;
        samples.push_back(path[static_cast<std::size_t>(idx - 1)] +
                          alpha * (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]));
      }
      return samples.size() == static_cast<std::size_t>(sample_count);
    }

    Eigen::Vector3d rotateOnYaw(const Eigen::Vector3d &dir, const double angle_rad)
    {
      Eigen::Vector3d rotated = dir;
      const double c = std::cos(angle_rad);
      const double s = std::sin(angle_rad);
      rotated.x() = c * dir.x() - s * dir.y();
      rotated.y() = s * dir.x() + c * dir.y();
      rotated.z() = 0.0;
      return rotated;
    }

    void fillReferenceVelocities(const std::vector<double> &times,
                                 const std::vector<Eigen::Vector3d> &positions,
                                 std::vector<Eigen::Vector3d> &velocities)
    {
      velocities.assign(positions.size(), Eigen::Vector3d::Zero());
      if (positions.size() < 2 || times.size() != positions.size())
      {
        return;
      }

      for (std::size_t i = 0; i + 1 < positions.size(); ++i)
      {
        const double dt = std::max(1.0e-3, times[i + 1] - times[i]);
        velocities[i] = (positions[i + 1] - positions[i]) / dt;
      }
      velocities.back() = velocities[velocities.size() - 2];
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
    nh.param("manager/sfc_corridor_margin", sfc_corridor_margin_, 0.05);
    nh.param("manager/jps_jump_max_cells", jps_jump_max_cells_, 6);
    nh.param("manager/jps_near_obs_radius", jps_near_obs_radius_, 1);
    nh.param("manager/guide_min_clearance", guide_min_clearance_, 0.35);
    nh.param("manager/guide_sparse_min_inner", guide_sparse_min_inner_, 2);
    nh.param("manager/guide_sparse_max_inner", guide_sparse_max_inner_, 5);
    nh.param("manager/guide_turn_angle_deg", guide_turn_angle_deg_, 25.0);
    nh.param("optimization/tracking_distance_min", tracking_distance_min_, 1.5);
    nh.param("optimization/tracking_distance_max", tracking_distance_max_, 4.0);
    nh.param("manager/tracking_anchor_future_time", tracking_anchor_future_time_, 1.0);
    nh.param("manager/tracking_anchor_max_future_time", tracking_anchor_max_future_time_, 2.0);
    nh.param("manager/tracking_anchor_dir_hysteresis", tracking_anchor_dir_hysteresis_, 0.35);
    nh.param("manager/tracking_anchor_side_angle_deg", tracking_anchor_side_angle_deg_, 35.0);
    nh.param("manager/tracking_viewpoint_dt", tracking_viewpoint_dt_, 0.6);
    nh.param("manager/tracking_viewpoint_max_num", tracking_viewpoint_max_num_, 5);
    nh.param("manager/tracking_viewpoint_yaw_step_deg", tracking_viewpoint_yaw_step_deg_, 20.0);
    nh.param("manager/tracking_viewpoint_connect_dist", tracking_viewpoint_connect_dist_, 1.5);
    nh.param("manager/tracking_viewpoint_clearance", tracking_viewpoint_clearance_, 0.15);
    nh.param("manager/tracking_hypothesis_topk", tracking_hypothesis_topk_, 3);
    nh.param("manager/tracking_time_align_alpha", tracking_time_align_alpha_, 0.55);
    nh.param("manager/tracking_visible_yaw_half_span_deg", tracking_visible_yaw_half_span_deg_, 35.0);
    nh.param("manager/tracking_visible_z_half_span", tracking_visible_z_half_span_, 0.50);
    nh.param("manager/enable_compiled_state2state", enable_compiled_state2state_, true);
    nh.param("manager/allow_compiled_state2state_legacy_fallback",
             allow_compiled_state2state_legacy_fallback_,
             false);
    tracking_distance_min_ = std::max(0.0, tracking_distance_min_);
    tracking_distance_max_ = std::max(tracking_distance_min_ + 0.1, tracking_distance_max_);
    tracking_anchor_future_time_ = std::max(0.0, tracking_anchor_future_time_);
    tracking_anchor_max_future_time_ = std::max(tracking_anchor_future_time_, tracking_anchor_max_future_time_);
    tracking_anchor_dir_hysteresis_ = std::max(0.0, std::min(0.95, tracking_anchor_dir_hysteresis_));
    tracking_anchor_side_angle_deg_ = std::max(0.0, std::min(85.0, tracking_anchor_side_angle_deg_));
    tracking_viewpoint_dt_ = std::max(0.15, tracking_viewpoint_dt_);
    tracking_viewpoint_max_num_ = std::max(2, tracking_viewpoint_max_num_);
    tracking_viewpoint_yaw_step_deg_ = std::max(5.0, std::min(60.0, tracking_viewpoint_yaw_step_deg_));
    tracking_viewpoint_connect_dist_ = std::max(0.3, tracking_viewpoint_connect_dist_);
    tracking_viewpoint_clearance_ = std::max(0.0, tracking_viewpoint_clearance_);
    const char *mode_name = managerDefaultModeString(use_sfc_corridor_, use_esdf_);
    ROS_INFO("Manager default obstacle preference: %s", mode_name);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    jps_astar_.reset(new JPSAStar(grid_map_, 0.0));
    jps_astar_->setTimeOut(sfc_path_timeout_);
    jps_astar_->setJumpMaxCells(jps_jump_max_cells_);
    jps_astar_->setJumpNearObsRadius(jps_near_obs_radius_);

    tracking_vrg_.reset(new VisibleRegionGraph(grid_map_));
    VisibleRegionGraph::Config vrg_cfg;
    vrg_cfg.tracking_distance_min = tracking_distance_min_;
    vrg_cfg.tracking_distance_max = tracking_distance_max_;
    vrg_cfg.viewpoint_dt = tracking_viewpoint_dt_;
    vrg_cfg.viewpoint_max_num = tracking_viewpoint_max_num_;
    vrg_cfg.viewpoint_yaw_step_deg = tracking_viewpoint_yaw_step_deg_;
    vrg_cfg.viewpoint_connect_dist = tracking_viewpoint_connect_dist_;
    vrg_cfg.viewpoint_clearance = std::max(guide_min_clearance_, tracking_viewpoint_clearance_);
    vrg_cfg.path_timeout = sfc_path_timeout_;
    vrg_cfg.jps_jump_max_cells = jps_jump_max_cells_;
    vrg_cfg.jps_near_obs_radius = jps_near_obs_radius_;
    vrg_cfg.edge_visibility_margin_min = 0.25 * std::max(guide_min_clearance_, tracking_viewpoint_clearance_);
    vrg_cfg.edge_distance_band_slack = 0.30;
    tracking_vrg_->setConfig(vrg_cfg);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);

  }

  bool EGOPlannerManager::mapWindowReady() const
  {
    if (!grid_map_)
    {
      return false;
    }

    const Eigen::Vector3d low = grid_map_->getUpdatedBoxLow();
    const Eigen::Vector3d high = grid_map_->getUpdatedBoxHigh();
    if (!low.allFinite() || !high.allFinite())
    {
      return false;
    }

    const double res = std::max(grid_map_->getResolution(), 1.0e-3);
    const Eigen::Vector3d span = high - low;
    return (span.array() > 6.0 * res).all();
  }

  bool EGOPlannerManager::corridorModeEnabled()
  {
    return use_sfc_corridor_;
  }

  bool EGOPlannerManager::esdfModeEnabled()
  {
    return use_esdf_;
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

    if (!mapWindowReady())
    {
      safe_target = raw_target;
      return true;
    }

    const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
    const Eigen::Vector3d map_low = grid_map_->getUpdatedBoxLow();
    const Eigen::Vector3d map_high = grid_map_->getUpdatedBoxHigh();
    if (!map_low.allFinite() || !map_high.allFinite() ||
        (map_high.array() <= map_low.array()).any())
    {
      safe_target = raw_target;
      return true;
    }

    const Eigen::Vector3d clamp_low =
        map_low + Eigen::Vector3d::Constant(2.0 * resolution);
    const Eigen::Vector3d clamp_high =
        map_high - Eigen::Vector3d::Constant(2.0 * resolution);

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

  bool EGOPlannerManager::sparsifyGuidePath(const std::vector<Eigen::Vector3d> &dense_path,
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
    const double piece_length = std::max(pp_.polyTraj_piece_length, 0.2);
    int desired_inner = std::max(0, static_cast<int>(std::round(total_len / piece_length)) - 1);
    desired_inner = std::max(desired_inner, guide_sparse_min_inner_);
    desired_inner = std::min(desired_inner, guide_sparse_max_inner_);
    desired_inner = std::min(desired_inner, static_cast<int>(dense_path.size()) - 2);

    std::vector<std::pair<double, int>> turn_candidates;
    const double turn_thresh_rad = guide_turn_angle_deg_ * M_PI / 180.0;
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
              [](const auto &a, const auto &b)
              { return a.first > b.first; });

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

    ROS_INFO("Guide path sparsification: dense=%zu sparse=%zu",
             dense_path.size(), sparse_path.size());
    return sparse_path.size() >= 2;
  }

  double EGOPlannerManager::estimateObstacleClearance(const Eigen::Vector3d &pt,
                                                      double search_radius,
                                                      Eigen::Vector3d *push_dir) const
  {
    if (!grid_map_ || !mapWindowReady())
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

  bool EGOPlannerManager::lineOfSightFree(const Eigen::Vector3d &from,
                                          const Eigen::Vector3d &to,
                                          double max_dist) const
  {
    if (!grid_map_ || !mapWindowReady())
    {
      return true;
    }

    const double dist = (to - from).norm();
    if (max_dist > 0.0 && dist > max_dist)
    {
      return false;
    }

    const Eigen::Vector3d low = grid_map_->getUpdatedBoxLow();
    const Eigen::Vector3d high = grid_map_->getUpdatedBoxHigh();
    const auto inside = [&](const Eigen::Vector3d &pt) -> bool
    {
      return (pt.array() >= low.array()).all() &&
             (pt.array() <= high.array()).all();
    };

    if (!inside(from) || !inside(to))
    {
      return false;
    }

    if (dist < 1.0e-6)
    {
      return grid_map_->getInflateOccupancy(from) == 0;
    }

    const double resolution = std::max(grid_map_->getResolution(), 1.0e-3);
    RayCaster ray_caster;
    if (!ray_caster.setInput((from - low) / resolution, (to - low) / resolution))
    {
      return grid_map_->getInflateOccupancy(from) == 0 &&
             grid_map_->getInflateOccupancy(to) == 0;
    }

    Eigen::Vector3d ray_idx;
    while (ray_caster.step(ray_idx))
    {
      const Eigen::Vector3d world_pt =
          low + (ray_idx.array() + 0.5).matrix() * resolution;
      if (grid_map_->getInflateOccupancy(world_pt) != 0)
      {
        return false;
      }
    }

    return grid_map_->getInflateOccupancy(to) == 0;
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

  bool EGOPlannerManager::applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                                      Eigen::VectorXd &durations) const
  {
    if (warm_durations.size() <= 0 || durations.size() <= 0)
    {
      return false;
    }
    if (!warm_durations.allFinite() || !durations.allFinite())
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
    adjusted = adjusted.cwiseMax(0.03);

    durations = adjusted;
    return true;
  }

  bool EGOPlannerManager::buildCorridorAwareInitialGuess(const Eigen::Vector3d &start_pt,
                                                        const Eigen::Vector3d &start_vel,
                                                        const Eigen::Vector3d &goal_pt,
                                                        const spatial_map::PolyhedraH &corridor_hpolys, Eigen::MatrixXd &inner_pts,
                                                        Eigen::VectorXd &durations, Eigen::VectorXi &corridor_piece_idx,
                                                        std::vector<Eigen::Vector3d> &transition_points,
                                                        std::vector<double> &inner_clearances) const
  {
    transition_points.clear();
    inner_clearances.clear();
    inner_pts.resize(3, 0);
    durations.resize(0);
    corridor_piece_idx.resize(0);

    if (corridor_hpolys.empty())
    {
      return false;
    }

    const double piece_length = std::max(pp_.polyTraj_piece_length, 0.2);
    const double alloc_speed = std::max(0.9 * pp_.max_vel_, 0.35);

    std::vector<Eigen::Vector3d> short_path;
    if (!spatial_map::buildCorridorInit(
            start_pt,
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
      ROS_WARN("buildCorridorAwareInitialGuess: corridor init failed.");
      return false;
    }

    if (corridor_piece_idx.size() != static_cast<int>(corridor_hpolys.size()) ||
        corridor_piece_idx.sum() != durations.size())
    {
      ROS_WARN("buildCorridorAwareInitialGuess: piece_idx mismatch, piece_idx_size=%ld corridor=%zu piece_sum=%d durations=%ld",
               static_cast<long>(corridor_piece_idx.size()),
               corridor_hpolys.size(),
               corridor_piece_idx.sum(),
               static_cast<long>(durations.size()));
    }

    inner_clearances.resize(std::max(0, static_cast<int>(inner_pts.cols())));
    const double clearance_probe_radius =
        std::max(guide_min_clearance_, 2.0 * (grid_map_ ? grid_map_->getResolution() : 0.1));
    for (int i = 0; i < inner_pts.cols(); ++i)
    {
      inner_clearances[i] = estimateObstacleClearance(
          inner_pts.col(i),
          clearance_probe_radius,
          nullptr);
    }

    ROS_INFO("buildCorridorAwareInitialGuess: pieces=%ld inner=%ld totalT=%.3f transitions=%ld short_path=%ld alloc_speed=%.3f conservative_timing=yes",
             static_cast<long>(durations.size()),
             static_cast<long>(inner_pts.cols()),
             durations.sum(),
             static_cast<long>(transition_points.size()),
             static_cast<long>(short_path.size()),
             alloc_speed);

    return true;
  }

  bool EGOPlannerManager::generateTrackingSafeFlightCorridor(
      const cost_functional::TrackingSemanticGuide &semantic_guide,
      spatial_map::PolyhedraH &corridor_hpolys,
      Eigen::VectorXi &corridor_piece_idx) const
  {
    corridor_hpolys.clear();
    corridor_piece_idx.resize(0);
    if (!semantic_guide.consistent() || semantic_guide.corridor_seed_path.size() < 2)
    {
      return false;
    }

    const Eigen::Vector3d low_corner = grid_map_->getUpdatedBoxLow();
    const Eigen::Vector3d high_corner = grid_map_->getUpdatedBoxHigh();
    std::vector<Eigen::Vector3d> occupied_points;
    grid_map_->getInflatedOccupiedPoints(occupied_points);

    // Tracking corridor must preserve semantic temporal order. Avoid geometric shortcutting here.
    sfc_gen::convexCover(semantic_guide.corridor_seed_path,
                         occupied_points,
                         low_corner,
                         high_corner,
                         sfc_progress_,
                         sfc_range_,
                         corridor_hpolys);
    if (corridor_hpolys.empty())
    {
      return false;
    }

    corridor_piece_idx = Eigen::VectorXi::Ones(static_cast<int>(corridor_hpolys.size()));
    return true;
  }

  bool EGOPlannerManager::buildTimeAlignedTrackingInitialGuess(
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &goal_pt,
      const cost_functional::TrackingSemanticGuide &semantic_guide,
      const spatial_map::PolyhedraH &corridor_hpolys,
      Eigen::MatrixXd &inner_pts,
      Eigen::VectorXd &durations,
      Eigen::VectorXi &corridor_piece_idx,
      std::vector<double> *inner_clearances) const
  {
    inner_pts.resize(3, 0);
    durations.resize(0);
    corridor_piece_idx.resize(0);
    if (inner_clearances != nullptr)
    {
      inner_clearances->clear();
    }

    if (!semantic_guide.consistent() || semantic_guide.corridor_seed_path.size() < 2)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> guide_path = semantic_guide.corridor_seed_path;
    std::vector<double> guide_times = semantic_guide.corridor_seed_times;
    guide_path.front() = start_pt;
    guide_path.back() = goal_pt;
    if (!guide_times.empty())
    {
      const double t0 = guide_times.front();
      for (double &t : guide_times)
      {
        t = std::max(0.0, t - t0);
      }
      guide_times.front() = 0.0;
    }

    const double prefix_horizon = 0.8;
    const int prefix_max_points = 3;
    const double now = ros::Time::now().toSec();
    std::vector<Eigen::Vector3d> prefix_points;
    std::vector<double> prefix_times;
    if (traj_.local_traj.duration > 1.0e-3 &&
        now >= traj_.local_traj.start_time &&
        now <= traj_.local_traj.end_time)
    {
      const double t_local_now = std::max(0.0, now - traj_.local_traj.start_time);
      const double t_local_end = std::min(traj_.local_traj.duration, t_local_now + prefix_horizon);
      bool prefix_valid = true;
      prefix_points.push_back(start_pt);
      prefix_times.push_back(0.0);

      for (int i = 1; i <= prefix_max_points; ++i)
      {
        const double ratio = static_cast<double>(i) / static_cast<double>(prefix_max_points + 1);
        const double t_sample = t_local_now + ratio * (t_local_end - t_local_now);
        if (t_sample >= traj_.local_traj.duration - 1.0e-3)
        {
          break;
        }

        const Eigen::Vector3d pt = traj_.local_traj.traj.evaluate(t_sample, 0);
        const double semantic_t = std::max(0.0, t_sample - t_local_now);
        cost_functional::VisibleFanRegion region;
        if (!cost_functional::semantic_guide::sampleVisibleFanRegion(semantic_guide, semantic_t, region) ||
            !cost_functional::semantic_guide::pointInsideVisibleFan(
                pt, region, 0.05, 8.0 * M_PI / 180.0, 0.08) ||
            !pointInsideCorridor(pt, corridor_hpolys, 0.0) ||
            !lineOfSightFree(pt, region.target_position))
        {
          prefix_valid = false;
          break;
        }

        prefix_points.push_back(pt);
        prefix_times.push_back(semantic_t);
      }

      if (prefix_valid && prefix_points.size() > 1)
      {
        const double last_prefix_t = prefix_times.back();
        std::vector<Eigen::Vector3d> merged_path;
        std::vector<double> merged_times;
        merged_path.reserve(prefix_points.size() + guide_path.size());
        merged_times.reserve(prefix_times.size() + guide_times.size());
        merged_path.push_back(start_pt);
        merged_times.push_back(0.0);
        for (std::size_t i = 1; i < prefix_points.size(); ++i)
        {
          merged_path.push_back(prefix_points[i]);
          merged_times.push_back(prefix_times[i]);
        }
        for (std::size_t i = 1; i < guide_path.size(); ++i)
        {
          if (i < guide_times.size() && guide_times[i] <= last_prefix_t + 0.05)
          {
            continue;
          }
          if ((guide_path[i] - merged_path.back()).norm() > 1.0e-3)
          {
            merged_path.push_back(guide_path[i]);
            merged_times.push_back(i < guide_times.size()
                                       ? std::max(last_prefix_t + 0.05, guide_times[i])
                                       : last_prefix_t + 0.10);
          }
        }
        guide_path.swap(merged_path);
        guide_times.swap(merged_times);
      }
    }

    if (guide_path.size() < 2 || guide_times.size() != guide_path.size())
    {
      return false;
    }

    std::vector<Eigen::Vector3d> expanded_path;
    std::vector<double> expanded_times;
    expanded_path.reserve(guide_path.size() * 2);
    expanded_times.reserve(guide_times.size() * 2);

    const double piece_length = std::max(pp_.polyTraj_piece_length, 0.25);
    for (std::size_t i = 0; i + 1 < guide_path.size(); ++i)
    {
      const Eigen::Vector3d &p0 = guide_path[i];
      const Eigen::Vector3d &p1 = guide_path[i + 1];
      const double t0 = guide_times[i];
      const double t1 = std::max(t0 + 0.05, guide_times[i + 1]);
      const double seg_len = (p1 - p0).norm();
      const int split_num = std::max(1, static_cast<int>(std::ceil(seg_len / piece_length)));
      if (i == 0)
      {
        expanded_path.push_back(p0);
        expanded_times.push_back(t0);
      }
      for (int k = 1; k <= split_num; ++k)
      {
        const double alpha = static_cast<double>(k) / static_cast<double>(split_num);
        expanded_path.push_back((1.0 - alpha) * p0 + alpha * p1);
        expanded_times.push_back((1.0 - alpha) * t0 + alpha * t1);
      }
    }

    if (expanded_path.size() == 2)
    {
      expanded_path.insert(expanded_path.begin() + 1, 0.5 * (expanded_path.front() + expanded_path.back()));
      expanded_times.insert(expanded_times.begin() + 1, 0.5 * (expanded_times.front() + expanded_times.back()));
    }
    if (expanded_path.size() < 2)
    {
      return false;
    }

    const double alpha_time = std::max(0.0, std::min(1.0, tracking_time_align_alpha_));
    const double v_nom = std::max(0.65 * pp_.max_vel_, 0.35);
    const int piece_num = static_cast<int>(expanded_path.size()) - 1;
    durations.resize(piece_num);
    inner_pts.resize(3, std::max(0, piece_num - 1));
    if (inner_clearances != nullptr)
    {
      inner_clearances->reserve(std::max(0, piece_num - 1));
    }

    for (int i = 0; i < piece_num; ++i)
    {
      const Eigen::Vector3d p0 = expanded_path[static_cast<std::size_t>(i)];
      const Eigen::Vector3d p1 = expanded_path[static_cast<std::size_t>(i + 1)];
      const double dt_sem = std::max(0.05, expanded_times[static_cast<std::size_t>(i + 1)] -
                                               expanded_times[static_cast<std::size_t>(i)]);
      const double dt_geo = std::max(0.05, (p1 - p0).norm() / v_nom);
      durations(i) = alpha_time * dt_sem + (1.0 - alpha_time) * dt_geo;
      if (i < piece_num - 1)
      {
        inner_pts.col(i) = expanded_path[static_cast<std::size_t>(i + 1)];
        if (inner_clearances != nullptr)
        {
          inner_clearances->push_back(
              estimateObstacleClearance(inner_pts.col(i),
                                        std::max(guide_min_clearance_,
                                                 2.0 * (grid_map_ ? grid_map_->getResolution() : 0.1)),
                                        nullptr));
        }
      }
    }

    corridor_piece_idx = Eigen::VectorXi::Zero(static_cast<int>(corridor_hpolys.size()));
    if (corridor_piece_idx.size() == 1)
    {
      corridor_piece_idx(0) = piece_num;
    }
    else if (corridor_piece_idx.size() > 1)
    {
      for (int i = 0; i < piece_num; ++i)
      {
        const double ratio = (static_cast<double>(i) + 0.5) / static_cast<double>(piece_num);
        const int poly_id =
            std::min(static_cast<int>(corridor_piece_idx.size()) - 1,
                     std::max(0, static_cast<int>(std::floor(ratio * corridor_piece_idx.size()))));
        corridor_piece_idx(poly_id) += 1;
      }
    }

    return durations.size() > 0 && corridor_piece_idx.sum() == durations.size();
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

    // ROS_INFO("generateSafeFlightCorridor: path_pts=%zu corridor_polys=%zu",
    //          guide_path.size(), corridor_hpolys.size());
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
    (void)start_vel;
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

    std::vector<Eigen::Vector3d> sparse_path;
    if (!sparsifyGuidePath(dense_path, sparse_path))
    {
      reportCorridorFailure(FAIL_GUIDE_PATH_TOO_CLOSE, "failed to sparsify local A* path for corridor");
      return false;
    }

    guide_path = sparse_path;
    if (!generateSafeFlightCorridor(guide_path, corridor_hpolys))
    {
      ROS_WARN("Corridor generation from sparse guide path failed, fallback to dense guide path.");
      guide_path = dense_path;
      if (!generateSafeFlightCorridor(guide_path, corridor_hpolys))
      {
        reportCorridorFailure(FAIL_CORRIDOR_GENERATION, "failed to generate safe flight corridor from guide path");
        return false;
      }
    }

    if (visualization_)
    {
      visualization_->displayGlobalPathList(dense_path, 0.08, 0);
      visualization_->displayFrontendList(guide_path, 0.12, 0);

      std::vector<Eigen::Vector3d> tri, edges;
      buildCorridorVisualization(corridor_hpolys, tri, edges);
      visualization_->displayCorridor(tri, edges, 0);
    }

    return true;
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
    Eigen::Vector3d safe_start = start_pt;

    if (!sanitizeLocalTarget(goal_pt, safe_goal))
    {
      ROS_WARN("FAIL_LOCAL_TARGET_INVALID: cannot sanitize local target");
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

    if (!grid_map_ || !mapWindowReady())
    {
      dense_path = {start_pt, safe_goal};
      return true;
    }

    if (!jps_astar_)
    {
      ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: jps_astar_ is null");
      return false;
    }

    if (!jps_astar_->search(safe_start, safe_goal, dense_path))
    {
      ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: 3D A* failed, start=[%.2f %.2f %.2f] safe_start=[%.2f %.2f %.2f] goal=[%.2f %.2f %.2f] safe_goal=[%.2f %.2f %.2f]",
               start_pt.x(), start_pt.y(), start_pt.z(),
               safe_start.x(), safe_start.y(), safe_start.z(),
               goal_pt.x(), goal_pt.y(), goal_pt.z(),
               safe_goal.x(), safe_goal.y(), safe_goal.z());
      return false;
    }

    if (dense_path.size() < 2)
    {
      ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: 3D A* returned too short path");
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

  bool EGOPlannerManager::buildTrackingAnchorCandidates(
      const cost_functional::TrackingReference &reference,
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      std::vector<Eigen::Vector3d> &anchor_candidates,
      std::vector<Eigen::Vector3d> *anchor_target_vels,
      std::vector<double> *anchor_times) const
  {
    anchor_candidates.clear();
    if (anchor_target_vels != nullptr)
    {
      anchor_target_vels->clear();
    }
    if (anchor_times != nullptr)
    {
      anchor_times->clear();
    }

    if (!reference.valid())
    {
      return false;
    }

    const double desired_dist = 0.5 * (tracking_distance_min_ + tracking_distance_max_);
    const double horizon_end = std::max(0.0, reference.t_ref.back());
    std::vector<double> sample_times{
        0.0,
        std::min(tracking_anchor_future_time_, horizon_end),
        std::min(tracking_anchor_max_future_time_, horizon_end),
        std::min(0.5 * horizon_end, tracking_anchor_max_future_time_),
        horizon_end};

    std::sort(sample_times.begin(), sample_times.end());
    sample_times.erase(std::unique(sample_times.begin(),
                                   sample_times.end(),
                                   [](double a, double b)
                                   { return std::abs(a - b) < 1.0e-3; }),
                       sample_times.end());
    if (sample_times.empty())
    {
      return false;
    }

    Eigen::Vector3d sticky_dir = have_tracking_anchor_dir_
                                     ? last_tracking_anchor_dir_
                                     : Eigen::Vector3d::UnitX();
    sticky_dir.z() = 0.0;
    if (sticky_dir.head<2>().norm() < 1.0e-3)
    {
      sticky_dir = Eigen::Vector3d::UnitX();
    }
    sticky_dir.normalize();

    for (double t_query : sample_times)
    {
      Eigen::Vector3d ref_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d ref_vel = Eigen::Vector3d::Zero();
      if (!cost_functional::sampleTrackingReference(reference, t_query, ref_pos, ref_vel))
      {
        continue;
      }

      Eigen::Vector3d anchor_dir = start_pt - ref_pos;
      anchor_dir.z() = 0.0;
      if (anchor_dir.head<2>().norm() < 0.5)
      {
        anchor_dir = -ref_vel;
        anchor_dir.z() = 0.0;
      }
      if (anchor_dir.head<2>().norm() < 0.3)
      {
        anchor_dir = -start_vel;
        anchor_dir.z() = 0.0;
      }
      if (anchor_dir.head<2>().norm() < 1.0e-3)
      {
        anchor_dir = sticky_dir;
      }
      else
      {
        anchor_dir.normalize();
        if (have_tracking_anchor_dir_ &&
            anchor_dir.dot(sticky_dir) < -tracking_anchor_dir_hysteresis_)
        {
          anchor_dir = sticky_dir;
        }
      }

      std::vector<Eigen::Vector3d> dir_candidates;
      dir_candidates.reserve(4);
      dir_candidates.push_back(anchor_dir);

      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      const double side_angle_rad = tracking_anchor_side_angle_deg_ * kDegToRad;
      if (side_angle_rad > 1.0e-3)
      {
        dir_candidates.push_back(rotateOnYaw(anchor_dir, side_angle_rad).normalized());
        dir_candidates.push_back(rotateOnYaw(anchor_dir, -side_angle_rad).normalized());
      }

      if (have_tracking_anchor_dir_)
      {
        dir_candidates.push_back(sticky_dir);
      }

      for (const auto &dir_candidate : dir_candidates)
      {
        Eigen::Vector3d anchor = ref_pos + desired_dist * dir_candidate;
        anchor.z() = ref_pos.z();

        bool duplicate = false;
        for (const auto &cand : anchor_candidates)
        {
          if ((cand - anchor).norm() < 0.25)
          {
            duplicate = true;
            break;
          }
        }
        if (duplicate)
        {
          continue;
        }

        anchor_candidates.push_back(anchor);
        if (anchor_target_vels != nullptr)
        {
          anchor_target_vels->push_back(ref_vel);
        }
        if (anchor_times != nullptr)
        {
          anchor_times->push_back(t_query);
        }
      }
    }

    return !anchor_candidates.empty();
  }

  bool EGOPlannerManager::buildTrackingViewpointSeries(
      const cost_functional::TrackingReference &reference,
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      std::vector<Eigen::Vector3d> &target_samples,
      std::vector<Eigen::Vector3d> &viewpoint_series,
      std::vector<Eigen::Vector3d> *viewpoint_target_vels,
      std::vector<double> *viewpoint_times) const
  {
    target_samples.clear();
    viewpoint_series.clear();
    if (viewpoint_target_vels != nullptr)
    {
      viewpoint_target_vels->clear();
    }
    if (viewpoint_times != nullptr)
    {
      viewpoint_times->clear();
    }

    if (!reference.valid())
    {
      return false;
    }

    const double desired_dist = 0.5 * (tracking_distance_min_ + tracking_distance_max_);
    const double horizon_end = std::max(0.0, reference.t_ref.back());
    std::vector<double> sample_times;
    sample_times.reserve(static_cast<std::size_t>(tracking_viewpoint_max_num_));

    if (horizon_end < 1.0e-3)
    {
      sample_times.push_back(0.0);
    }
    else
    {
      const int desired_count = std::max(
          2,
          std::min(tracking_viewpoint_max_num_,
                   static_cast<int>(std::ceil(horizon_end / tracking_viewpoint_dt_)) + 1));
      for (int i = 0; i < desired_count; ++i)
      {
        const double ratio = (desired_count <= 1)
                                 ? 0.0
                                 : static_cast<double>(i) / static_cast<double>(desired_count - 1);
        sample_times.push_back(ratio * horizon_end);
      }
    }

    std::sort(sample_times.begin(), sample_times.end());
    sample_times.erase(std::unique(sample_times.begin(),
                                   sample_times.end(),
                                   [](double a, double b)
                                   { return std::abs(a - b) < 1.0e-3; }),
                       sample_times.end());

    struct ViewpointCandidate
    {
      Eigen::Vector3d viewpoint{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_vel{Eigen::Vector3d::Zero()};
      double t{0.0};
      double base_score{-std::numeric_limits<double>::infinity()};
      double dp_score{-std::numeric_limits<double>::infinity()};
      int parent{-1};
    };

    Eigen::Vector3d sticky_dir = have_tracking_anchor_dir_
                                     ? last_tracking_anchor_dir_
                                     : Eigen::Vector3d::UnitX();
    sticky_dir.z() = 0.0;
    if (sticky_dir.head<2>().norm() < 1.0e-3)
    {
      sticky_dir = Eigen::Vector3d::UnitX();
    }
    sticky_dir.normalize();

    Eigen::Vector3d init_seed_dir = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_target_vel = Eigen::Vector3d::Zero();
    if (cost_functional::sampleTrackingReference(reference, sample_times.front(), initial_target, initial_target_vel))
    {
      init_seed_dir = start_pt - initial_target;
    }
    init_seed_dir.z() = 0.0;
    if (init_seed_dir.head<2>().norm() < 0.3)
    {
      init_seed_dir = -initial_target_vel;
      init_seed_dir.z() = 0.0;
    }
    if (init_seed_dir.head<2>().norm() < 0.3)
    {
      init_seed_dir = -start_vel;
      init_seed_dir.z() = 0.0;
    }
    if (init_seed_dir.head<2>().norm() < 1.0e-3)
    {
      init_seed_dir = sticky_dir;
    }
    init_seed_dir.normalize();

    const double yaw_step_rad = tracking_viewpoint_yaw_step_deg_ * M_PI / 180.0;
    const int max_ring_id = std::max(4, static_cast<int>(std::ceil(M_PI / yaw_step_rad)));
    const int angle_sample_num = 2 * max_ring_id + 1;
    const double max_los_dist =
        std::max(tracking_distance_max_ + 0.5,
                 desired_dist + 2.0 * std::max(guide_min_clearance_, tracking_viewpoint_clearance_));
    const double resolution = grid_map_ ? std::max(grid_map_->getResolution(), 1.0e-3) : 0.1;
    const double probe_radius =
        std::max(tracking_viewpoint_clearance_, 2.0 * resolution);

    std::vector<std::vector<ViewpointCandidate>> candidate_layers;
    candidate_layers.reserve(sample_times.size());

    Eigen::Vector3d prev_target = initial_target;
    Eigen::Vector3d prev_seed_viewpoint = start_pt;
    bool have_prev_seed_viewpoint = true;
    for (std::size_t layer_idx = 0; layer_idx < sample_times.size(); ++layer_idx)
    {
      const double t_query = sample_times[layer_idx];
      Eigen::Vector3d ref_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d ref_vel = Eigen::Vector3d::Zero();
      if (!cost_functional::sampleTrackingReference(reference, t_query, ref_pos, ref_vel))
      {
        continue;
      }

      Eigen::Vector3d seed_dir =
          (layer_idx == 0 || !have_prev_seed_viewpoint) ? init_seed_dir : (prev_seed_viewpoint - ref_pos);
      seed_dir.z() = 0.0;
      if (seed_dir.head<2>().norm() < 0.3)
      {
        seed_dir = -ref_vel;
        seed_dir.z() = 0.0;
      }
      if (seed_dir.head<2>().norm() < 0.3)
      {
        seed_dir = start_pt - ref_pos;
        seed_dir.z() = 0.0;
      }
      if (seed_dir.head<2>().norm() < 1.0e-3)
      {
        seed_dir = sticky_dir;
      }
      seed_dir.normalize();

      const double seed_yaw = std::atan2(seed_dir.y(), seed_dir.x());
      std::vector<ViewpointCandidate> layer_candidates;
      layer_candidates.reserve(static_cast<std::size_t>(angle_sample_num));

      for (int sample_id = 0; sample_id < angle_sample_num; ++sample_id)
      {
        const int ring_id = (sample_id == 0) ? 0 : ((sample_id + 1) / 2);
        const double yaw_offset = static_cast<double>(ring_id) * yaw_step_rad;
        const double candidate_yaw =
            seed_yaw + ((sample_id % 2 == 0) ? yaw_offset : -yaw_offset);

        Eigen::Vector3d candidate = ref_pos;
        candidate.x() += desired_dist * std::cos(candidate_yaw);
        candidate.y() += desired_dist * std::sin(candidate_yaw);
        candidate.z() = ref_pos.z();

        Eigen::Vector3d safe_candidate = candidate;
        if (!sanitizeLocalTarget(candidate, safe_candidate))
        {
          continue;
        }

        const Eigen::Vector3d rel = safe_candidate - ref_pos;
        const double radial_dist = rel.head<2>().norm();
        if (radial_dist < 0.7 * tracking_distance_min_ ||
            radial_dist > 1.35 * tracking_distance_max_)
        {
          continue;
        }

        if (!lineOfSightFree(safe_candidate, ref_pos, max_los_dist))
        {
          continue;
        }

        bool duplicate = false;
        for (const auto &existing : layer_candidates)
        {
          if ((existing.viewpoint - safe_candidate).norm() < std::max(0.25, 1.5 * resolution))
          {
            duplicate = true;
            break;
          }
        }
        if (duplicate)
        {
          continue;
        }

        const double clearance = estimateObstacleClearance(safe_candidate, probe_radius, nullptr);
        const double radial_err = std::abs(radial_dist - desired_dist);
        const Eigen::Vector3d rel_dir = rel.normalized();
        const double sticky_align = rel_dir.dot(sticky_dir);
        const double motion_align =
            (layer_idx == 0 || (ref_pos - prev_target).head<2>().norm() < 1.0e-3)
                ? 0.0
                : rel_dir.head<2>().dot((ref_pos - prev_target).head<2>().normalized());

        ViewpointCandidate vp;
        vp.viewpoint = safe_candidate;
        vp.target = ref_pos;
        vp.target_vel = ref_vel;
        vp.t = t_query;
        vp.base_score =
            1.1 * clearance -
            0.8 * radial_err +
            0.55 * sticky_align +
            0.25 * motion_align;
        layer_candidates.push_back(vp);
      }

      if (!layer_candidates.empty())
      {
        std::sort(layer_candidates.begin(),
                  layer_candidates.end(),
                  [](const ViewpointCandidate &lhs, const ViewpointCandidate &rhs)
                  { return lhs.base_score > rhs.base_score; });
        if (layer_candidates.size() > 7)
        {
          layer_candidates.resize(7);
        }
        prev_seed_viewpoint = layer_candidates.front().viewpoint;
        have_prev_seed_viewpoint = true;
        candidate_layers.push_back(layer_candidates);
        prev_target = ref_pos;
      }
    }

    if (candidate_layers.empty())
    {
      return false;
    }

    for (std::size_t layer_idx = 0; layer_idx < candidate_layers.size(); ++layer_idx)
    {
      auto &layer = candidate_layers[layer_idx];
      if (layer_idx == 0)
      {
        for (auto &candidate : layer)
        {
          const double continuity =
              -0.10 * (candidate.viewpoint - start_pt).head<2>().norm();
          const double connect_bonus =
              lineOfSightFree(start_pt, candidate.viewpoint, tracking_viewpoint_connect_dist_) ? 0.35 : -0.25;
          candidate.dp_score = candidate.base_score + continuity + connect_bonus;
          candidate.parent = -1;
        }
        continue;
      }

      const auto &prev_layer = candidate_layers[layer_idx - 1];
      for (auto &candidate : layer)
      {
        for (int prev_idx = 0; prev_idx < static_cast<int>(prev_layer.size()); ++prev_idx)
        {
          const auto &prev_candidate = prev_layer[static_cast<std::size_t>(prev_idx)];
          const Eigen::Vector3d prev_rel = prev_candidate.viewpoint - prev_candidate.target;
          const Eigen::Vector3d curr_rel = candidate.viewpoint - candidate.target;
          const double rel_align =
              (prev_rel.head<2>().norm() < 1.0e-3 || curr_rel.head<2>().norm() < 1.0e-3)
                  ? 0.0
                  : prev_rel.head<2>().normalized().dot(curr_rel.head<2>().normalized());
          const double continuity =
              -0.12 * (candidate.viewpoint - prev_candidate.viewpoint).head<2>().norm();
          const double connect_bonus =
              lineOfSightFree(prev_candidate.viewpoint, candidate.viewpoint, tracking_viewpoint_connect_dist_) ? 0.45 : -0.35;
          const double transition_score =
              prev_candidate.dp_score + candidate.base_score + continuity + connect_bonus + 0.7 * rel_align;
          if (transition_score > candidate.dp_score)
          {
            candidate.dp_score = transition_score;
            candidate.parent = prev_idx;
          }
        }
      }
    }

    int best_layer_idx = static_cast<int>(candidate_layers.size()) - 1;
    int best_candidate_idx = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(candidate_layers.back().size()); ++i)
    {
      const double score = candidate_layers.back()[static_cast<std::size_t>(i)].dp_score;
      if (score > best_score)
      {
        best_score = score;
        best_candidate_idx = i;
      }
    }
    if (best_candidate_idx < 0)
    {
      return false;
    }

    std::vector<ViewpointCandidate> best_sequence;
    while (best_layer_idx >= 0 && best_candidate_idx >= 0)
    {
      const auto &candidate = candidate_layers[static_cast<std::size_t>(best_layer_idx)][static_cast<std::size_t>(best_candidate_idx)];
      best_sequence.push_back(candidate);
      best_candidate_idx = candidate.parent;
      --best_layer_idx;
    }
    std::reverse(best_sequence.begin(), best_sequence.end());

    for (std::size_t i = 0; i < best_sequence.size(); ++i)
    {
      const auto &candidate = best_sequence[i];
      if (!viewpoint_series.empty() &&
          (candidate.viewpoint - viewpoint_series.back()).norm() < std::max(0.3, 2.0 * resolution) &&
          i + 1 < best_sequence.size())
      {
        continue;
      }

      target_samples.push_back(candidate.target);
      viewpoint_series.push_back(candidate.viewpoint);
      if (viewpoint_target_vels != nullptr)
      {
        viewpoint_target_vels->push_back(candidate.target_vel);
      }
      if (viewpoint_times != nullptr)
      {
        viewpoint_times->push_back(candidate.t);
      }
    }

    return !viewpoint_series.empty();
  }

  bool EGOPlannerManager::buildGuidePathFromWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                                                      std::vector<Eigen::Vector3d> &guide_path) const
  {
    guide_path.clear();
    if (waypoints.size() < 2)
    {
      return false;
    }

    guide_path.push_back(waypoints.front());
    for (std::size_t i = 1; i < waypoints.size(); ++i)
    {
      const Eigen::Vector3d &next_wp = waypoints[i];
      if ((next_wp - guide_path.back()).norm() < 1.0e-3)
      {
        continue;
      }

      if (lineOfSightFree(guide_path.back(), next_wp, tracking_viewpoint_connect_dist_))
      {
        guide_path.push_back(next_wp);
        continue;
      }

      std::vector<Eigen::Vector3d> segment_path;
      Eigen::Vector3d safe_goal = next_wp;
      if (!prepareLocalAStarPath(guide_path.back(), next_wp, segment_path, safe_goal))
      {
        return false;
      }

      for (std::size_t j = 1; j < segment_path.size(); ++j)
      {
        if ((segment_path[j] - guide_path.back()).norm() > 1.0e-3)
        {
          guide_path.push_back(segment_path[j]);
        }
      }
    }

    if (guide_path.size() < 2)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> shortcut_path;
    shortcut_path.reserve(guide_path.size());
    std::size_t anchor_idx = 0;
    shortcut_path.push_back(guide_path.front());
    while (anchor_idx + 1 < guide_path.size())
    {
      std::size_t next_idx = anchor_idx + 1;
      for (std::size_t cand = guide_path.size(); cand-- > anchor_idx + 1;)
      {
        if (lineOfSightFree(guide_path[anchor_idx], guide_path[cand]))
        {
          next_idx = cand;
          break;
        }
      }

      if ((guide_path[next_idx] - shortcut_path.back()).norm() > 1.0e-3)
      {
        shortcut_path.push_back(guide_path[next_idx]);
      }
      anchor_idx = next_idx;
    }

    if (shortcut_path.size() >= 2)
    {
      std::vector<Eigen::Vector3d> sparse_path;
      if (sparsifyGuidePath(shortcut_path, sparse_path))
      {
        guide_path = sparse_path;
      }
      else
      {
        guide_path = shortcut_path;
      }
    }

    return guide_path.size() >= 2;
  }

  bool EGOPlannerManager::buildTrackingVisibleRegionGuide(
      const cost_functional::TrackingReference &reference,
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      std::vector<Eigen::Vector3d> &target_samples,
      std::vector<Eigen::Vector3d> &viewpoint_series,
      std::vector<Eigen::Vector3d> &guide_path,
      std::vector<Eigen::Vector3d> *viewpoint_target_vels,
      std::vector<double> *viewpoint_times,
      std::vector<Eigen::Vector3d> *candidate_points) const
  {
    std::vector<cost_functional::TrackingSemanticGuide> hypotheses;
    if (!buildTrackingSemanticGuideHypotheses(reference, start_pt, start_vel, hypotheses))
    {
      return false;
    }

    const auto &best = hypotheses.front();
    target_samples.clear();
    viewpoint_series.clear();
    guide_path.clear();
    if (viewpoint_target_vels != nullptr)
    {
      viewpoint_target_vels->clear();
    }
    if (viewpoint_times != nullptr)
    {
      viewpoint_times->clear();
    }
    if (candidate_points != nullptr)
    {
      candidate_points->clear();
    }
    target_samples = best.target_samples;
    viewpoint_series = best.viewpoint_series;
    guide_path = best.corridor_seed_path;
    if (viewpoint_target_vels != nullptr)
    {
      *viewpoint_target_vels = best.target_vel_samples;
    }
    if (viewpoint_times != nullptr)
    {
      *viewpoint_times = best.viewpoint_times;
    }
    if (candidate_points != nullptr)
    {
      *candidate_points = best.candidate_points;
    }

    return !viewpoint_series.empty() && guide_path.size() >= 2;
  }

  bool EGOPlannerManager::buildTrackingVisibleFanRegions(
      cost_functional::TrackingSemanticGuide &semantic_guide) const
  {
    semantic_guide.visible_regions.clear();
    if (semantic_guide.viewpoint_series.empty() ||
        semantic_guide.viewpoint_times.size() != semantic_guide.viewpoint_series.size() ||
        (!semantic_guide.target_samples.empty() &&
         semantic_guide.target_samples.size() != semantic_guide.viewpoint_series.size()))
    {
      return false;
    }

    if (semantic_guide.target_samples.empty())
    {
      return false;
    }

    if (semantic_guide.target_vel_samples.size() != semantic_guide.target_samples.size())
    {
      semantic_guide.target_vel_samples.assign(semantic_guide.target_samples.size(), Eigen::Vector3d::Zero());
      fillReferenceVelocities(semantic_guide.viewpoint_times,
                              semantic_guide.target_samples,
                              semantic_guide.target_vel_samples);
    }

    std::vector<Eigen::Vector3d> viewpoint_vels;
    fillReferenceVelocities(semantic_guide.viewpoint_times,
                            semantic_guide.viewpoint_series,
                            viewpoint_vels);

    const double yaw_half_span =
        std::max(10.0, tracking_visible_yaw_half_span_deg_) * M_PI / 180.0;
    const double z_half_span = std::max(0.15, tracking_visible_z_half_span_);

    semantic_guide.visible_regions.reserve(semantic_guide.viewpoint_series.size());
    for (std::size_t i = 0; i < semantic_guide.viewpoint_series.size(); ++i)
    {
      cost_functional::VisibleFanRegion region;
      region.t = semantic_guide.viewpoint_times[i];
      region.target_position = semantic_guide.target_samples[i];
      region.target_velocity =
          (i < semantic_guide.target_vel_samples.size()) ? semantic_guide.target_vel_samples[i]
                                                         : Eigen::Vector3d::Zero();
      region.min_tracking_distance = tracking_distance_min_;
      region.max_tracking_distance = tracking_distance_max_;
      const Eigen::Vector3d rel = semantic_guide.viewpoint_series[i] - semantic_guide.target_samples[i];
      region.yaw_center = std::atan2(rel.y(), rel.x());
      region.yaw_half_span = yaw_half_span;
      region.z_center = semantic_guide.viewpoint_series[i].z();
      region.z_half_span = z_half_span;
      region.preferred_viewpoint = semantic_guide.viewpoint_series[i];
      region.preferred_view_velocity =
          (i < viewpoint_vels.size()) ? viewpoint_vels[i] : Eigen::Vector3d::Zero();
      region.visibility_margin =
          std::min(estimateObstacleClearance(region.preferred_viewpoint,
                                             std::max(guide_min_clearance_,
                                                      2.0 * (grid_map_ ? grid_map_->getResolution() : 0.1)),
                                             nullptr),
                   std::max(0.0, grid_map_ && grid_map_->esdfEnabled()
                                     ? grid_map_->getDistance(region.preferred_viewpoint)
                                     : estimateObstacleClearance(region.preferred_viewpoint,
                                                                 std::max(guide_min_clearance_, 0.15),
                                                                 nullptr)));
      region.valid = region.target_position.allFinite() &&
                     region.preferred_viewpoint.allFinite() &&
                     std::isfinite(region.t);
      semantic_guide.visible_regions.push_back(region);
    }

    semantic_guide.valid = semantic_guide.consistent();
    return semantic_guide.valid;
  }

  bool EGOPlannerManager::buildTrackingSemanticGuideHypotheses(
      const cost_functional::TrackingReference &reference,
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      std::vector<cost_functional::TrackingSemanticGuide> &hypotheses) const
  {
    hypotheses.clear();
    if (!reference.valid() || !tracking_vrg_)
    {
      return false;
    }

    std::vector<Eigen::Vector3d> sticky_dirs;
    const auto addStickyDir = [&](const Eigen::Vector3d &dir_in)
    {
      Eigen::Vector3d dir = dir_in;
      dir.z() = 0.0;
      if (dir.head<2>().norm() < 1.0e-3)
      {
        return;
      }
      dir.normalize();
      for (const auto &existing : sticky_dirs)
      {
        if (dir.dot(existing) > 0.98)
        {
          return;
        }
      }
      sticky_dirs.push_back(dir);
    };

    Eigen::Vector3d target_now = reference.p_ref.front();
    Eigen::Vector3d target_vel_now = reference.v_ref.empty() ? Eigen::Vector3d::Zero() : reference.v_ref.front();
    cost_functional::sampleTrackingReference(reference, 0.0, target_now, target_vel_now);

    if (have_tracking_anchor_dir_)
    {
      addStickyDir(last_tracking_anchor_dir_);
      addStickyDir(rotateOnYaw(last_tracking_anchor_dir_, tracking_anchor_side_angle_deg_ * M_PI / 180.0));
      addStickyDir(rotateOnYaw(last_tracking_anchor_dir_, -tracking_anchor_side_angle_deg_ * M_PI / 180.0));
    }

    addStickyDir(start_pt - target_now);
    addStickyDir(-target_vel_now);
    addStickyDir(-start_vel);
    addStickyDir(Eigen::Vector3d::UnitX());

    for (const auto &sticky_dir : sticky_dirs)
    {
      cost_functional::TrackingSemanticGuide hypothesis;
      if (!tracking_vrg_->search(reference.t_ref,
                                 reference.p_ref,
                                 reference.v_ref,
                                 start_pt,
                                 start_vel,
                                 &sticky_dir,
                                 hypothesis))
      {
        continue;
      }

      if (!buildTrackingVisibleFanRegions(hypothesis))
      {
        continue;
      }

      if (!hypothesis.consistent())
      {
        continue;
      }
      hypotheses.push_back(hypothesis);
    }

    if (hypotheses.empty())
    {
      return false;
    }

    std::sort(hypotheses.begin(),
              hypotheses.end(),
              [](const cost_functional::TrackingSemanticGuide &lhs,
                 const cost_functional::TrackingSemanticGuide &rhs)
              {
                const double lhs_score =
                    lhs.hypothesis_score + 0.8 * lhs.visibility_score - 0.03 * lhs.path_length;
                const double rhs_score =
                    rhs.hypothesis_score + 0.8 * rhs.visibility_score - 0.03 * rhs.path_length;
                return lhs_score > rhs_score;
              });

    return true;
  }

  bool EGOPlannerManager::planTrackingTask(
      const cost_functional::TrackingReference &reference,
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc,
      const bool flag_polyInit,
      const bool flag_randomPolyTraj,
      const bool force_plain)
  {
    if (!reference.valid())
    {
      ROS_WARN("planTrackingTask rejected: invalid tracking reference.");
      return false;
    }

    Eigen::Vector3d tracking_target = reference.p_ref.front();
    Eigen::Vector3d tracking_target_vel_now = reference.v_ref.empty() ? Eigen::Vector3d::Zero()
                                                                       : reference.v_ref.front();
    cost_functional::sampleTrackingReference(reference, 0.0, tracking_target, tracking_target_vel_now);
    const bool touch_goal = false;
    const double desired_dist = 0.5 * (tracking_distance_min_ + tracking_distance_max_);
    const auto computeYaw0 = [&]() -> double
    {
      const double t_local_now = std::max(0.0, ros::Time::now().toSec() - traj_.local_traj.start_time);
      if (traj_.local_traj.has_yaw_ref)
      {
        return traj_.local_traj.sampleYaw(t_local_now);
      }
      if (start_vel.head<2>().norm() > 0.15)
      {
        return std::atan2(start_vel.y(), start_vel.x());
      }
      const Eigen::Vector3d rel = tracking_target - start_pt;
      if (rel.head<2>().norm() > 0.15)
      {
        return std::atan2(rel.y(), rel.x());
      }
      return 0.0;
    };
    const double yaw0 = computeYaw0();
    // This tracking pipeline is semantic-guide-based:
    // prediction -> occlusion-aware topology -> visible regions -> corridor -> joint safety/visibility optimization.
    std::vector<cost_functional::TrackingSemanticGuide> semantic_hypotheses;
    if (buildTrackingSemanticGuideHypotheses(reference, start_pt, start_vel, semantic_hypotheses))
    {
      const int top_k = std::max(1, std::min(tracking_hypothesis_topk_, static_cast<int>(semantic_hypotheses.size())));
      for (int hyp_idx = 0; hyp_idx < top_k; ++hyp_idx)
      {
        const auto &semantic_guide = semantic_hypotheses[static_cast<std::size_t>(hyp_idx)];
        if (!semantic_guide.consistent() ||
            semantic_guide.viewpoint_series.empty() ||
            semantic_guide.corridor_seed_path.size() < 2)
        {
          continue;
        }

        cost_functional::TrackingReference planning_reference = reference;
        planning_reference.t_view_ref = semantic_guide.viewpoint_times;
        planning_reference.p_view_ref = semantic_guide.viewpoint_series;
        fillReferenceVelocities(planning_reference.t_view_ref,
                                planning_reference.p_view_ref,
                                planning_reference.v_view_ref);
        planning_reference.use_view_terminal = true;
        planning_reference.has_terminal_ref = true;
        planning_reference.p_term_ref = semantic_guide.viewpoint_series.back();
        planning_reference.v_term_ref =
            planning_reference.v_view_ref.empty() ? Eigen::Vector3d::Zero()
                                                  : planning_reference.v_view_ref.back();

        const Eigen::Vector3d tracking_anchor = semantic_guide.viewpoint_series.back();
        const double anchor_t =
            semantic_guide.viewpoint_times.empty() ? reference.t_ref.back() : semantic_guide.viewpoint_times.back();
        const Eigen::Vector3d anchor_view_vel =
            planning_reference.v_view_ref.empty() ? Eigen::Vector3d::Zero()
                                                  : planning_reference.v_view_ref.back();

        bool semantic_screen_ok = true;
        if (use_sfc_corridor_ && !force_plain)
        {
          spatial_map::PolyhedraH screening_corridor;
          Eigen::VectorXi screening_piece_idx;
          Eigen::MatrixXd screening_inner_pts;
          Eigen::VectorXd screening_durations;
          std::vector<double> screening_clearances;
          semantic_screen_ok =
              generateTrackingSafeFlightCorridor(semantic_guide, screening_corridor, screening_piece_idx) &&
              buildTimeAlignedTrackingInitialGuess(start_pt,
                                                  start_vel,
                                                  tracking_anchor,
                                                  semantic_guide,
                                                  screening_corridor,
                                                  screening_inner_pts,
                                                  screening_durations,
                                                  screening_piece_idx,
                                                  &screening_clearances);

          if (semantic_screen_ok)
          {
            MINCOTraj3D screening_init;
            const auto head_state = makeBoundaryState(start_pt, start_vel, start_acc);
            const auto tail_state = makeBoundaryState(tracking_anchor, anchor_view_vel, Eigen::Vector3d::Zero());
            semantic_screen_ok =
                screening_init.generate(screening_inner_pts, head_state, tail_state, screening_durations) &&
                ploy_traj_opt_->isTrajectoryCollisionFree(screening_init) &&
                ploy_traj_opt_->isTrajectoryInsideCorridor(screening_init, screening_corridor, 0.0);
          }

          ROS_INFO("Tracking semantic hypothesis %d/%d: guide_pts=%zu visible_regions=%zu score=%.3f screen=%s",
                   hyp_idx + 1,
                   top_k,
                   semantic_guide.corridor_seed_path.size(),
                   semantic_guide.visible_regions.size(),
                   semantic_guide.hypothesis_score,
                   semantic_screen_ok ? "pass" : "fail");
          if (!semantic_screen_ok)
          {
            continue;
          }
        }

        if (visualization_)
        {
          visualization_->displayDebugPathList(semantic_guide.target_samples,
                                               0.10,
                                               Eigen::Vector4d(1.0, 0.55, 0.05, 1.0),
                                               8000);
          visualization_->displayDebugPathList(semantic_guide.candidate_points,
                                               0.06,
                                               Eigen::Vector4d(0.65, 0.65, 1.0, 0.9),
                                               8003);
          visualization_->displayDebugPathList(semantic_guide.viewpoint_series,
                                               0.12,
                                               Eigen::Vector4d(0.15, 0.75, 1.0, 1.0),
                                               8001);
          visualization_->displayDebugPathList(semantic_guide.corridor_seed_path,
                                               0.08,
                                               Eigen::Vector4d(0.15, 1.0, 0.35, 1.0),
                                               8002,
                                               true);
        }

        ROS_INFO("planTrackingTask(VRG-semantic): ref_size=%zu hypothesis=%d/%d viewpoint_count=%zu guide_pts=%zu view_ref=%s horizon_end_t=%.3f anchor_t=%.2f target_now=[%.2f %.2f %.2f] anchor=[%.2f %.2f %.2f] d*=%.2f",
                 reference.t_ref.size(),
                 hyp_idx + 1,
                 top_k,
                 semantic_guide.viewpoint_series.size(),
                 semantic_guide.corridor_seed_path.size(),
                 planning_reference.viewValid() ? "yes" : "no",
                 reference.t_ref.back(),
                 anchor_t,
                 tracking_target.x(),
                 tracking_target.y(),
                 tracking_target.z(),
                 tracking_anchor.x(),
                 tracking_anchor.y(),
                 tracking_anchor.z(),
                 desired_dist);

        if (reboundReplan(start_pt,
                          start_vel,
                          start_acc,
                          tracking_anchor,
                          anchor_view_vel,
                          flag_polyInit,
                          flag_randomPolyTraj,
                          touch_goal,
                          force_plain,
                          &planning_reference,
                          &semantic_guide.corridor_seed_path,
                          &semantic_guide))
        {
          Eigen::Vector3d success_dir = tracking_anchor - tracking_target;
          success_dir.z() = 0.0;
          if (success_dir.head<2>().norm() > 1.0e-3)
          {
            last_tracking_anchor_dir_ = success_dir.normalized();
            have_tracking_anchor_dir_ = true;
          }

          const double yaw_dt = 0.05;
          const double max_yaw_rate = 1.2;
          auto yaw_plan = TrackingYawPlanner::planFacingTarget(
              traj_.local_traj.traj, planning_reference, yaw_dt, max_yaw_rate, yaw0);
          planning_reference.t_yaw_ref = yaw_plan.t;
          planning_reference.yaw_ref = yaw_plan.yaw;
          traj_.setLocalYawRef(yaw_plan.t, yaw_plan.yaw);
          active_tracking_semantic_guide_ = semantic_guide;
          have_active_tracking_semantic_guide_ = active_tracking_semantic_guide_.consistent();
          return true;
        }
      }

      ROS_WARN("planTrackingTask: semantic visible-region guide hypotheses failed, fallback to anchor trials.");
    }
    else
    {
      ROS_WARN("planTrackingTask: failed to build semantic visible-region guide hypotheses, fallback to anchor trials.");
    }

    std::vector<Eigen::Vector3d> anchor_candidates;
    std::vector<Eigen::Vector3d> anchor_target_vels;
    std::vector<double> anchor_times;
    if (!buildTrackingAnchorCandidates(reference,
                                       start_pt,
                                       start_vel,
                                       anchor_candidates,
                                       &anchor_target_vels,
                                       &anchor_times))
    {
      ROS_WARN("planTrackingTask rejected: failed to build tracking anchor candidates.");
      return false;
    }

    for (std::size_t i = 0; i < anchor_candidates.size(); ++i)
    {
      const Eigen::Vector3d &tracking_anchor = anchor_candidates[i];
      const Eigen::Vector3d target_vel =
          (i < anchor_target_vels.size()) ? anchor_target_vels[i] : Eigen::Vector3d::Zero();
      const double anchor_t = (i < anchor_times.size()) ? anchor_times[i] : 0.0;
      cost_functional::TrackingReference planning_reference = reference;
      planning_reference.use_view_terminal = false;
      planning_reference.has_terminal_ref = true;
      planning_reference.p_term_ref = tracking_anchor;
      planning_reference.v_term_ref = target_vel;

      ROS_INFO("planTrackingTask: ref_size=%zu anchor_trial=%zu horizon_end_t=%.3f anchor_t=%.2f target_now=[%.2f %.2f %.2f] anchor=[%.2f %.2f %.2f] d*=%.2f",
               reference.t_ref.size(),
               i + 1,
               reference.t_ref.back(),
               anchor_t,
               tracking_target.x(),
               tracking_target.y(),
               tracking_target.z(),
               tracking_anchor.x(),
               tracking_anchor.y(),
               tracking_anchor.z(),
               desired_dist);

      if (reboundReplan(start_pt,
                        start_vel,
                        start_acc,
                        tracking_anchor,
                        target_vel,
                        flag_polyInit,
                        flag_randomPolyTraj,
                        touch_goal,
                        force_plain,
                        &planning_reference,
                        nullptr))
      {
        Eigen::Vector3d success_dir = tracking_anchor - tracking_target;
        success_dir.z() = 0.0;
        if (success_dir.head<2>().norm() > 1.0e-3)
        {
          last_tracking_anchor_dir_ = success_dir.normalized();
          have_tracking_anchor_dir_ = true;
        }
        const double yaw_dt = 0.05;
        const double max_yaw_rate = 1.2;
        auto yaw_plan = TrackingYawPlanner::planFacingTarget(
            traj_.local_traj.traj, planning_reference, yaw_dt, max_yaw_rate, yaw0);
        planning_reference.t_yaw_ref = yaw_plan.t;
        planning_reference.yaw_ref = yaw_plan.yaw;
        traj_.setLocalYawRef(yaw_plan.t, yaw_plan.yaw);
        have_active_tracking_semantic_guide_ = false;
        active_tracking_semantic_guide_.clear();
        active_tracking_corridor_.clear();
        return true;
      }
    }

    ROS_WARN("planTrackingTask failed after %zu anchor trials.", anchor_candidates.size());
    return false;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal,
      const bool force_plain,
      const cost_functional::TrackingReference *tracking_ref,
      const std::vector<Eigen::Vector3d> *preferred_guide_path,
      const cost_functional::TrackingSemanticGuide *tracking_semantic_guide)
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
    Eigen::VectorXi corridor_piece_idx;
    Eigen::Vector3d safe_target_pt = local_target_pt;
    const bool is_tracking_task = (tracking_ref != nullptr && tracking_ref->valid());
    if (tracking_ref != nullptr && !tracking_ref->valid())
    {
      ROS_WARN("Tracking task rejected: invalid tracking reference.");
      return false;
    }

    if (!sanitizeLocalTarget(local_target_pt, safe_target_pt))
    {
      reportCorridorFailure(FAIL_LOCAL_TARGET_INVALID, "unable to sanitize local target before planning");
      return false;
    }

    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(safe_target_pt, local_target_vel, Eigen::Vector3d::Zero());

    const bool use_corridor = use_sfc_corridor_ && !force_plain;
    const bool use_esdf = use_esdf_ && !use_corridor && !force_plain;
    const bool has_preferred_guide =
        preferred_guide_path != nullptr && preferred_guide_path->size() >= 2;
    const bool has_tracking_semantic_guide =
        tracking_semantic_guide != nullptr && tracking_semantic_guide->consistent();

    if (use_corridor)
    {
      std::vector<Eigen::Vector3d> guide_path;
      std::vector<Eigen::Vector3d> transition_points;
      std::vector<double> inner_clearances;
      bool corridor_ready = false;
      if (has_tracking_semantic_guide)
      {
        guide_path = tracking_semantic_guide->corridor_seed_path;
        guide_path.front() = start_pt;
        guide_path.back() = safe_target_pt;
        corridor_ready =
            generateTrackingSafeFlightCorridor(*tracking_semantic_guide,
                                              corridor_hpolys,
                                              corridor_piece_idx);

        if (corridor_ready && visualization_)
        {
          visualization_->displayGlobalPathList(tracking_semantic_guide->corridor_seed_path, 0.08, 2);
          visualization_->displayFrontendList(guide_path, 0.12, 2);

          std::vector<Eigen::Vector3d> tri, edges;
          buildCorridorVisualization(corridor_hpolys, tri, edges);
          visualization_->displayCorridor(tri, edges, 2);
        }
      }
      else if (has_preferred_guide)
      {
        guide_path = *preferred_guide_path;
        guide_path.front() = start_pt;
        guide_path.back() = safe_target_pt;

        std::vector<Eigen::Vector3d> sparse_guide_path;
        if (sparsifyGuidePath(guide_path, sparse_guide_path) &&
            generateSafeFlightCorridor(sparse_guide_path, corridor_hpolys))
        {
          guide_path = sparse_guide_path;
          corridor_ready = true;
        }
        else if (generateSafeFlightCorridor(guide_path, corridor_hpolys))
        {
          corridor_ready = true;
        }
        else
        {
          ROS_WARN("Tracking preferred guide path failed to generate corridor, fallback to local guide search.");
        }

        if (corridor_ready && visualization_)
        {
          visualization_->displayGlobalPathList(*preferred_guide_path, 0.08, 2);
          visualization_->displayFrontendList(guide_path, 0.12, 2);

          std::vector<Eigen::Vector3d> tri, edges;
          buildCorridorVisualization(corridor_hpolys, tri, edges);
          visualization_->displayCorridor(tri, edges, 2);
        }
      }

      if (!corridor_ready &&
          !prepareLocalGuideAndCorridor(start_pt, start_vel, safe_target_pt, guide_path, corridor_hpolys))
      {
        return false;
      }
      const bool tracking_semantic_init_ok =
          has_tracking_semantic_guide &&
          buildTimeAlignedTrackingInitialGuess(start_pt,
                                              start_vel,
                                              safe_target_pt,
                                              *tracking_semantic_guide,
                                              corridor_hpolys,
                                              innerPts,
                                              durations,
                                              corridor_piece_idx,
                                              &inner_clearances);
      if (!tracking_semantic_init_ok &&
          !buildCorridorAwareInitialGuess(start_pt,
                                          start_vel,
                                          safe_target_pt,
                                          corridor_hpolys,
                                          innerPts,
                                          durations,
                                          corridor_piece_idx,
                                          transition_points,
                                          inner_clearances))
      {
        reportCorridorFailure(FAIL_CORRIDOR_INIT, "failed to build GCOPTER-style corridor initial guess");
        return false;
      }

      bool corridor_warm_timing_used = false;
      if (!flag_polyInit)
      {
        MINCOTraj3D warm_traj;
        Eigen::MatrixXd warm_inner_pts;
        Eigen::VectorXd warm_durations;
        MINCOBoundaryState3D warm_head, warm_tail;

        const bool warm_ok = computeInitState(start_pt,
                                              start_vel,
                                              start_acc,
                                              safe_target_pt,
                                              local_target_vel,
                                              false,
                                              false,
                                              ts,
                                              warm_traj,
                                              warm_inner_pts,
                                              warm_durations,
                                              warm_head,
                                              warm_tail);
        if (warm_ok && applyWarmStartTimingProfile(warm_durations, durations))
        {
          corridor_warm_timing_used = true;
        }
      }

      if (!initTraj.generate(innerPts, headState, tailState, durations))
      {
        reportCorridorFailure(FAIL_CORRIDOR_INIT, "failed to generate corridor-seeded MINCO trajectory");
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

      const bool seed_collision_free = ploy_traj_opt_->isTrajectoryCollisionFree(initTraj);
      const bool seed_inside_corridor =
          ploy_traj_opt_->isTrajectoryInsideCorridor(initTraj, corridor_hpolys, 0.0);
      ROS_INFO("INIT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f inside_corridor=%s",
               seed_collision_free ? "yes" : "no",
               computeTrajectoryMinSdf(initTraj),
               seed_inside_corridor ? "yes" : "no");
      ROS_INFO("Corridor seed warm_timing=%s time_scaling_feasible=%s",
               corridor_warm_timing_used ? "yes" : "no",
               init_seed_feasible ? "yes" : "no");

      if (!seed_collision_free || !seed_inside_corridor)
      {
        std::stringstream ss;
        ss << "corridor seed remains infeasible after conservative timing/scaling: "
           << "collision_free=" << (seed_collision_free ? "yes" : "no")
           << " inside_corridor=" << (seed_inside_corridor ? "yes" : "no");
        reportCorridorFailure(FAIL_CORRIDOR_INIT, ss.str());
        return false;
      }
    }
    else if (use_esdf)
    {
      std::vector<Eigen::Vector3d> dense_path, guide_path;
      Eigen::Vector3d safe_goal = safe_target_pt;
      if (has_preferred_guide)
      {
        dense_path = *preferred_guide_path;
        dense_path.front() = start_pt;
        dense_path.back() = safe_goal;
        if (!sparsifyGuidePath(dense_path, guide_path))
        {
          guide_path = dense_path;
        }
      }
      else if (!prepareLocalAStarPath(start_pt, safe_target_pt, dense_path, safe_goal))
      {
        ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: cannot prepare local A* path for ESDF");
        return false;
      }

      if (guide_path.empty() && !sparsifyGuidePath(dense_path, guide_path))
      {
        ROS_WARN("FAIL_LOCAL_ASTAR_SEARCH: cannot sparsify ESDF guide path");
        return false;
      }

      if (visualization_)
      {
        visualization_->displayGlobalPathList(dense_path, 0.08, 1);
        visualization_->displayFrontendList(guide_path, 0.10, 1);
      }

      Eigen::MatrixXd guide_inner_pts;
      Eigen::VectorXd guide_durations;
      MINCOTraj3D guide_init_traj;
      if (!buildInitStateFromGuidePath(start_pt,
                                       start_vel,
                                       start_acc,
                                       safe_goal,
                                       local_target_vel,
                                       guide_path,
                                       guide_init_traj,
                                       guide_inner_pts,
                                       guide_durations,
                                       headState,
                                       tailState))
      {
        ROS_WARN("FAIL_ESDF_INIT: failed to build guide-based initial trajectory.");
        return false;
      }

      innerPts = guide_inner_pts;
      durations = guide_durations;
      initTraj = guide_init_traj;

      bool warm_start_used = false;
      if (!flag_polyInit)
      {
        MINCOTraj3D warm_traj;
        Eigen::MatrixXd warm_inner_pts;
        Eigen::VectorXd warm_durations;
        MINCOBoundaryState3D warm_head, warm_tail;
        const bool warm_ok = computeInitState(start_pt,
                                              start_vel,
                                              start_acc,
                                              safe_goal,
                                              local_target_vel,
                                              false,
                                              false,
                                              ts,
                                              warm_traj,
                                              warm_inner_pts,
                                              warm_durations,
                                              warm_head,
                                              warm_tail);

        if (warm_ok && warm_durations.size() > 0)
        {
          std::vector<Eigen::Vector3d> warm_anchors;
          if (resamplePolylineByCount(guide_path, warm_durations.size() + 1, warm_anchors))
          {
            Eigen::MatrixXd warm_inner_from_guide(3, std::max(0, static_cast<int>(warm_anchors.size()) - 2));
            for (int i = 1; i + 1 < static_cast<int>(warm_anchors.size()); ++i)
            {
              warm_inner_from_guide.col(i - 1) = warm_anchors[static_cast<std::size_t>(i)];
            }

            MINCOTraj3D mixed_init_traj;
            if (mixed_init_traj.generate(warm_inner_from_guide, headState, tailState, warm_durations))
            {
              innerPts = warm_inner_from_guide;
              durations = warm_durations;
              initTraj = mixed_init_traj;
              warm_start_used = true;
            }
          }
        }
      }

      ROS_INFO("ESDF init strategy: %s (guide_points=%zu pieces=%ld)",
               warm_start_used ? "warm_duration+guide_inner" : "guide_only",
               guide_path.size(),
               static_cast<long>(durations.size()));
      const double init_min_sdf = computeTrajectoryMinSdf(initTraj);
      const double esdf_tol = grid_map_ ? -std::max(0.02, 0.5 * grid_map_->getResolution()) : 0.0;
      const bool init_esdf_free = init_min_sdf >= esdf_tol;
      ROS_INFO("INIT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f",
               init_esdf_free ? "yes" : "no",
               init_min_sdf);
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
    if (visualization_)
    {
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    std::vector<std::vector<Eigen::Vector3d>> vis_trajs;

    if (use_corridor)
    {
      double final_cost;
      if (is_tracking_task)
      {
        if (has_tracking_semantic_guide)
        {
          flag_success = ploy_traj_opt_->optimizeTrackingTrajectoryWithVisibleRegions(headState,
                                                                                      tailState,
                                                                                      innerPts,
                                                                                      durations,
                                                                                      corridor_hpolys,
                                                                                      &corridor_piece_idx,
                                                                                      *tracking_ref,
                                                                                      *tracking_semantic_guide,
                                                                                      final_cost);
        }
        else
        {
          flag_success = ploy_traj_opt_->optimizeTrackingTrajectory(headState, tailState,
                                                                    innerPts, durations,
                                                                    corridor_hpolys,
                                                                    &corridor_piece_idx,
                                                                    *tracking_ref,
                                                                    final_cost);
        }
      }
      else
      {
        flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                          innerPts, durations,
                                                          corridor_hpolys,
                                                          &corridor_piece_idx,
                                                          final_cost);
      }

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f inside_corridor=%s",
                 computeTrajectoryMinSdf(opt_traj),
                 ploy_traj_opt_->isTrajectoryInsideCorridor(opt_traj, corridor_hpolys, 0.0) ? "yes" : "no");
        setLocalTrajFromOpt(opt_traj, touch_goal);
        if (is_tracking_task)
        {
          active_tracking_corridor_ = corridor_hpolys;
          if (has_tracking_semantic_guide)
          {
            active_tracking_semantic_guide_ = *tracking_semantic_guide;
            have_active_tracking_semantic_guide_ = active_tracking_semantic_guide_.consistent();
          }
        }
        else
        {
          have_active_tracking_semantic_guide_ = false;
          active_tracking_semantic_guide_.clear();
          active_tracking_corridor_.clear();
        }
        cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
        if (visualization_)
        {
          visualization_->displayOptimalList(cstr_pts, 0);
        }
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
      if (is_tracking_task)
      {
        flag_success = ploy_traj_opt_->optimizeTrackingTrajectoryWithDistanceField(headState, tailState,
                                                                                   innerPts, durations,
                                                                                   *tracking_ref,
                                                                                   has_tracking_semantic_guide ? tracking_semantic_guide : nullptr,
                                                                                   final_cost);
      }
      else
      {
        flag_success = ploy_traj_opt_->optimizeTrajectoryWithDistanceField(headState, tailState,
                                                                           innerPts, durations, final_cost);
      }

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        const double min_sdf = computeTrajectoryMinSdf(opt_traj);
        const double esdf_tol = grid_map_ ? -std::max(0.02, 0.5 * grid_map_->getResolution()) : 0.0;
        const bool esdf_free = min_sdf >= esdf_tol;
        ROS_INFO("OPT_TRAJ_CHECK: collision_free=yes min_sdf=%.3f",
                 min_sdf);
        ROS_DEBUG("OPT_TRAJ_CHECK_DETAIL: esdf_free=%s tol=%.3f",
                  esdf_free ? "yes" : "no", esdf_tol);
        setLocalTrajFromOpt(opt_traj, touch_goal);
        if (is_tracking_task && has_tracking_semantic_guide)
        {
          active_tracking_semantic_guide_ = *tracking_semantic_guide;
          active_tracking_corridor_.clear();
          have_active_tracking_semantic_guide_ = active_tracking_semantic_guide_.consistent();
        }
        else if (!is_tracking_task)
        {
          have_active_tracking_semantic_guide_ = false;
          active_tracking_semantic_guide_.clear();
          active_tracking_corridor_.clear();
        }
        cstr_pts = sampleTrajectoryForDisplay(opt_traj, 0.02);
        if (visualization_)
        {
          visualization_->displayOptimalList(cstr_pts, 0);
        }
      }
      else
      {
        const MINCOTraj3D &opt_traj = ploy_traj_opt_->getTrajectory();
        const double min_sdf = computeTrajectoryMinSdf(opt_traj);
        const double esdf_tol = grid_map_ ? -std::max(0.02, 0.5 * grid_map_->getResolution()) : 0.0;
        ROS_WARN("OPT_TRAJ_CHECK: collision_free=%s min_sdf=%.3f",
                 (min_sdf >= esdf_tol) ? "yes" : "no",
                 min_sdf);
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

      if (visualization_)
      {
        visualization_->displayMultiOptimalPathList(vis_trajs, 0.1);
      }

      if (flag_success)
      {
        setLocalTrajFromOpt(best_traj, touch_goal);
        have_active_tracking_semantic_guide_ = false;
        active_tracking_semantic_guide_.clear();
        active_tracking_corridor_.clear();
        cstr_pts = best_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
        if (visualization_)
        {
          visualization_->displayOptimalList(cstr_pts, 0);
        }
      }
    }
    else
    {
      double final_cost;
      if (is_tracking_task)
      {
        flag_success = ploy_traj_opt_->optimizeTrackingTrajectory(headState, tailState,
                                                                  innerPts, durations,
                                                                  *tracking_ref,
                                                                  has_tracking_semantic_guide ? tracking_semantic_guide : nullptr,
                                                                  final_cost);
      }
      else
      {
        flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                          innerPts, durations, final_cost);
      }

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        setLocalTrajFromOpt(opt_traj, touch_goal);
        if (is_tracking_task && has_tracking_semantic_guide)
        {
          active_tracking_semantic_guide_ = *tracking_semantic_guide;
          active_tracking_corridor_.clear();
          have_active_tracking_semantic_guide_ = active_tracking_semantic_guide_.consistent();
        }
        else if (!is_tracking_task)
        {
          have_active_tracking_semantic_guide_ = false;
          active_tracking_semantic_guide_.clear();
          active_tracking_corridor_.clear();
        }
        
        cstr_pts = opt_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
        if (visualization_)
        {
          visualization_->displayOptimalList(cstr_pts, 0);
        }
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
      if (visualization_)
      {
        visualization_->displayFailedList(cstr_pts, 0);
      }

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

  bool EGOPlannerManager::trackingSemanticHorizonValid(double t_cur, double horizon) const
  {
    if (!have_active_tracking_semantic_guide_ ||
        !active_tracking_semantic_guide_.consistent() ||
        traj_.local_traj.duration <= 1.0e-3)
    {
      return false;
    }

    const double end_t = std::min(traj_.local_traj.duration, t_cur + std::max(0.2, horizon));
    const double dt = std::max(0.08, std::min(0.20, 0.25 * std::max(0.2, horizon)));
    for (double t = t_cur; t <= end_t + 1.0e-6; t += dt)
    {
      const double sample_t = std::min(t, traj_.local_traj.duration);
      const Eigen::Vector3d pt = traj_.local_traj.traj.evaluate(sample_t, 0);
      if (grid_map_ && grid_map_->getInflateOccupancy(pt) != 0)
      {
        return false;
      }

      if (!active_tracking_corridor_.empty() &&
          !pointInsideCorridor(pt, active_tracking_corridor_, 0.0))
      {
        return false;
      }

      cost_functional::VisibleFanRegion region;
      if (!cost_functional::semantic_guide::sampleVisibleFanRegion(active_tracking_semantic_guide_,
                                                                   sample_t,
                                                                   region))
      {
        return false;
      }

      if (!cost_functional::semantic_guide::pointInsideVisibleFan(
              pt, region, 0.05, 8.0 * M_PI / 180.0, 0.08))
      {
        return false;
      }

      if (!lineOfSightFree(pt, region.target_position))
      {
        return false;
      }
    }
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
