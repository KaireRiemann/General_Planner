#include "optimizer/poly_traj_optimizer.h"
#include "SFCGenerator/geo_utils.hpp"

#include <limits>

using namespace std;

#define VERBOSE_OUTPUT false
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)

namespace
{
  spatial_map::PolyhedronV makeSpatialVertexPoly(const Eigen::Matrix3Xd &vertices)
  {
    spatial_map::PolyhedronV mapped;
    if (vertices.cols() <= 0)
    {
      return mapped;
    }

    mapped.resize(3, vertices.cols());
    mapped.col(0) = vertices.col(0);
    for (int i = 1; i < vertices.cols(); ++i)
    {
      mapped.col(i) = vertices.col(i) - vertices.col(0);
    }
    return mapped;
  }

  bool processCorridor(const spatial_map::PolyhedraH &hpolys,
                       spatial_map::PolyhedraV &vpolys)
  {
    vpolys.clear();
    if (hpolys.empty())
    {
      return false;
    }
    const int size_corridor = static_cast<int>(hpolys.size()) - 1;
    vpolys.reserve(static_cast<std::size_t>(2 * std::max(size_corridor, 0) + 1));

    Eigen::Matrix3Xd vertices;
    for (int i = 0; i < size_corridor; ++i)
    {
      if (!geo_utils::enumerateVs(hpolys[static_cast<std::size_t>(i)], vertices) || vertices.cols() <= 0)
      {
        return false;
      }
      vpolys.push_back(makeSpatialVertexPoly(vertices));

      Eigen::MatrixX4d overlap_h(hpolys[static_cast<std::size_t>(i)].rows() +
                                     hpolys[static_cast<std::size_t>(i + 1)].rows(),
                                 4);
      overlap_h.topRows(hpolys[static_cast<std::size_t>(i)].rows()) = hpolys[static_cast<std::size_t>(i)];
      overlap_h.bottomRows(hpolys[static_cast<std::size_t>(i + 1)].rows()) = hpolys[static_cast<std::size_t>(i + 1)];
      if (!geo_utils::enumerateVs(overlap_h, vertices) || vertices.cols() <= 0)
      {
        return false;
      }
      vpolys.push_back(makeSpatialVertexPoly(vertices));
    }

    if (!geo_utils::enumerateVs(hpolys.back(), vertices) || vertices.cols() <= 0)
    {
      return false;
    }
    vpolys.push_back(makeSpatialVertexPoly(vertices));
    return true;
  }

  struct TrajectoryCheckDebug
  {
    bool collision_detected{false};
    double first_collision_t{-1.0};
    Eigen::Vector3d first_collision_pt{Eigen::Vector3d::Zero()};
    double max_corridor_violation{0.0};
    double worst_corridor_t{-1.0};
    Eigen::Vector3d worst_corridor_pt{Eigen::Vector3d::Zero()};
  };

  double computePolyViolation(const spatial_map::PolyhedronH &poly,
                              const Eigen::Vector3d &pt,
                              const double margin)
  {
    double worst_plane = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < poly.rows(); ++i)
    {
      worst_plane = std::max(worst_plane,
                             poly.row(i).head<3>().dot(pt) + poly(i, 3) + margin);
    }
    return worst_plane;
  }

  double computeCorridorViolation(const spatial_map::PolyhedraH &corridor_hpolys,
                                  const Eigen::Vector3d &pt,
                                  const double margin)
  {
    if (corridor_hpolys.empty())
    {
      return 0.0;
    }

    double best_poly_violation = std::numeric_limits<double>::infinity();
    for (const auto &poly : corridor_hpolys)
    {
      best_poly_violation = std::min(best_poly_violation,
                                     computePolyViolation(poly, pt, margin));
    }
    return best_poly_violation;
  }

  TrajectoryCheckDebug analyzeTrajectoryCheck(const ego_planner::MINCOTraj &traj,
                                             const GridMap::Ptr &grid_map,
                                             const spatial_map::PolyhedraH &corridor_hpolys,
                                             const double corridor_margin,
                                             const double max_vel)
  {
    TrajectoryCheckDebug debug;
    const double total_duration = traj.getTotalDuration();
    const double map_dt =
        grid_map ? std::min(0.02, grid_map->getResolution() / std::max(max_vel, 0.1) * 0.5) : 0.02;
    const double dt = std::max(0.005, map_dt);

    for (double t = 0.0; t <= total_duration + 1.0e-6; t += dt)
    {
      const double sample_t = std::min(t, total_duration);
      const Eigen::Vector3d pt = traj.evaluate(sample_t, 0);

      if (grid_map && !debug.collision_detected &&
          grid_map->getInflateOccupancy(pt) != 0)
      {
        debug.collision_detected = true;
        debug.first_collision_t = sample_t;
        debug.first_collision_pt = pt;
      }

      const double violation =
          computeCorridorViolation(corridor_hpolys, pt, corridor_margin);
      if (violation > debug.max_corridor_violation)
      {
        debug.max_corridor_violation = violation;
        debug.worst_corridor_t = sample_t;
        debug.worst_corridor_pt = pt;
      }
    }

    return debug;
  }
} // namespace

namespace ego_planner
{
  void PolyTrajOptimizer::resetSpatialOptimizationContext()
  {
    corridor_vpolys_.clear();
    corridor_vpoly_idx_.resize(0);
    corridor_hpoly_idx_.resize(0);
    corridorSpatialMap_.reset(nullptr, nullptr, 0);
    corridor_cost_manager_.setCorridor(nullptr, nullptr);
  }

  // =====================================================
  //  Main optimization loop (decision logic)
  // =====================================================
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      double &final_cost)
  {
    optimize_mode_ = MODE_PLAIN;
    resetSpatialOptimizationContext();

    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_unsafe, flag_success, flag_swarm_too_close;
    multitopology_data_.initial_obstacles_avoided = false;
    wei_swarm_mod_ = wei_swarm_;

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    mincoOpt_.setEnergyWeight(rho_energy_);
    mincoOpt_.setSamplesPerPiece(cps_num_prePiece_);

    Eigen::MatrixXd waypoints(piece_num_ + 1, 3);
    waypoints.row(0) = iniState.col(0).transpose(); 
    
    for (int i = 0; i < initInnerPts.cols(); ++i) {
        waypoints.row(i + 1) = initInnerPts.col(i).transpose(); 
    }
    
    waypoints.row(piece_num_) = finState.col(0).transpose(); 


    std::vector<double> time_segs(piece_num_);
    for (int i = 0; i < piece_num_; ++i)
      time_segs[i] = initT(i);

    mincoOpt_.setInitState(time_segs, waypoints, iniState, finState);

    cost_manager_.grid_map = grid_map_;
    cost_manager_.cps = &cps_;
    cost_manager_.swarm_traj = swarm_trajs_;
    cost_manager_.wei_obs = wei_obs_;
    cost_manager_.wei_obs_soft = wei_obs_soft_;
    cost_manager_.wei_swarm = wei_swarm_mod_;
    cost_manager_.wei_feas = wei_feas_;
    cost_manager_.wei_sqrvar = wei_sqrvar_;
    cost_manager_.obs_clearance = obs_clearance_;
    cost_manager_.obs_clearance_soft = obs_clearance_soft_;
    cost_manager_.swarm_clearance = swarm_clearance_;
    cost_manager_.max_vel = max_vel_;
    cost_manager_.max_acc = max_acc_;
    cost_manager_.max_jer = max_jer_;
    cost_manager_.drone_id = drone_id_;
    cost_manager_.t_now = t_now_;
    cost_manager_.touch_goal = touch_goal_;
    cost_manager_.cps_per_piece = cps_num_prePiece_;
    cost_manager_.min_ellip_dist2_ptr = &min_ellip_dist2_;

    Eigen::VectorXd x0 = mincoOpt_.generateInitialGuess();
    variable_num_ = x0.size();

    double x_init[variable_num_];
    memcpy(x_init, x0.data(), variable_num_ * sizeof(double));

    min_ellip_dist2_.resize(swarm_trajs_ ? swarm_trajs_->size() : 0);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;

    do
    {
      iter_num_ = 0;
      flag_force_return = false;
      force_stop_type_ = DONT_STOP;
      flag_still_unsafe = false;
      flag_success = false;
      flag_swarm_too_close = false;
      cost_manager_.wei_swarm = wei_swarm_mod_;

      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);
      t2 = ros::Time::now();

      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      PRINTF_COND("LBFGS result=%d (%s), iter=%d, cost=%.6f, time=%.3fms\n",
                  result, lbfgs::lbfgs_strerror(result), iter_num_, final_cost, time_ms);

      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP ||
          result == lbfgs::LBFGSERR_ROUNDING_ERROR)
      {
        flag_force_return = false;

        if (swarm_trajs_ != nullptr)
        {
            for (size_t i = 0; i < swarm_trajs_->size(); ++i)
            {
              flag_swarm_too_close |= min_ellip_dist2_[i] < pow((swarm_clearance_ + swarm_trajs_->at(i).des_clearance) * 1.25, 2);
            }
        }

        if (!flag_swarm_too_close)
        {
          const MINCOTraj &traj = mincoOpt_.getTrajectory();
          Eigen::MatrixXd init_points = getInitConstraintPoints(); 

          std::vector<std::pair<int, int>> segments_nouse;
          if (finelyCheckAndSetConstraintPoints(segments_nouse, traj, init_points, false) == CHK_RET::OBS_FREE)
          {
            flag_success = true;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          }
          else
          {
            flag_still_unsafe = true;
            restart_nums++;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
          }
        }
        else
        {
          PRINTF_COND("Swarm clearance not satisfied, keep optimizing. iter=%d,time(ms)=%5.3f, wei_swarm_mod_=%f\n", iter_num_, time_ms, wei_swarm_mod_);
          flag_still_unsafe = true;
          restart_nums++;
          wei_swarm_mod_ *= 2;
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iter_num_, time_ms);
      }
      else
      {
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iter_num_, time_ms);
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }

    } while ((flag_still_unsafe && restart_nums < 3) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return flag_success;
  }

  bool PolyTrajOptimizer::optimizeTrajectoryWithDistanceField(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      double &final_cost)
  {
    optimize_mode_ = MODE_ESDF;
    resetSpatialOptimizationContext();

    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }
    if (!grid_map_ || !grid_map_->esdfEnabled())
    {
      ROS_ERROR("ESDF optimization requested but ESDF map is disabled.");
      return false;
    }

    int restart_nums = 0;
    bool flag_success = false, flag_swarm_too_close = false;
    wei_swarm_mod_ = wei_swarm_;

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    distanceFieldMincoOpt_.setEnergyWeight(rho_energy_);
    distanceFieldMincoOpt_.setSamplesPerPiece(cps_num_prePiece_);

    Eigen::MatrixXd waypoints(piece_num_ + 1, 3);
    waypoints.row(0) = iniState.col(0).transpose();
    for (int i = 0; i < initInnerPts.cols(); ++i)
    {
      waypoints.row(i + 1) = initInnerPts.col(i).transpose();
    }
    waypoints.row(piece_num_) = finState.col(0).transpose();

    std::vector<double> time_segs(piece_num_);
    for (int i = 0; i < piece_num_; ++i)
    {
      time_segs[i] = initT(i);
    }

    const MINCOTraj init_traj = generateTrajectory(iniState, finState, initInnerPts, initT);
    const Eigen::MatrixXd init_cps = init_traj.getInitConstraintPoints(cps_num_prePiece_);
    cps_.resize_cp(init_cps.cols());
    cps_.points = init_cps;

    distanceFieldMincoOpt_.setInitState(time_segs, waypoints, iniState, finState);

    distance_field_cost_manager_.grid_map = grid_map_;
    distance_field_cost_manager_.cps = &cps_;
    distance_field_cost_manager_.swarm_traj = swarm_trajs_;
    distance_field_cost_manager_.wei_dist = wei_dist_;
    distance_field_cost_manager_.wei_swarm = wei_swarm_mod_;
    distance_field_cost_manager_.wei_feas = wei_feas_;
    distance_field_cost_manager_.wei_sqrvar = wei_sqrvar_;
    distance_field_cost_manager_.safe_margin = safety_margin_;
    distance_field_cost_manager_.swarm_clearance = swarm_clearance_;
    distance_field_cost_manager_.max_vel = max_vel_;
    distance_field_cost_manager_.max_acc = max_acc_;
    distance_field_cost_manager_.max_jer = max_jer_;
    distance_field_cost_manager_.drone_id = drone_id_;
    distance_field_cost_manager_.t_now = t_now_;
    distance_field_cost_manager_.touch_goal = touch_goal_;
    distance_field_cost_manager_.min_ellip_dist2_ptr = &min_ellip_dist2_;

    Eigen::VectorXd x0 = distanceFieldMincoOpt_.generateInitialGuess();
    variable_num_ = x0.size();

    double x_init[variable_num_];
    memcpy(x_init, x0.data(), variable_num_ * sizeof(double));

    min_ellip_dist2_.resize(swarm_trajs_ ? swarm_trajs_->size() : 0);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;

    const auto computeMinSdf = [&](const MINCOTraj &traj) -> double
    {
      const double total_duration = traj.getTotalDuration();
      const double dt = std::max(0.01, std::min(0.05, grid_map_->getResolution() / std::max(max_vel_, 0.1)));
      double min_sdf = std::numeric_limits<double>::infinity();
      for (double t = 0.0; t <= total_duration + 1.0e-6; t += dt)
      {
        const double sample_t = std::min(t, total_duration);
        min_sdf = std::min(min_sdf, grid_map_->getDistance(traj.evaluate(sample_t, 0)));
      }
      return std::isfinite(min_sdf) ? min_sdf : 0.0;
    };

    const double sdf_collision_tol = -std::max(0.10, 0.5 * grid_map_->getResolution());
    const double sdf_soft_margin = std::max(0.0, 0.5 * obs_clearance_);
    const double init_min_sdf = computeMinSdf(init_traj);
    const bool init_esdf_free = init_min_sdf >= sdf_collision_tol;

    do
    {
      iter_num_ = 0;
      force_stop_type_ = DONT_STOP;
      flag_success = false;
      flag_swarm_too_close = false;
      distance_field_cost_manager_.wei_swarm = wei_swarm_mod_;

      const int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP ||
          result == lbfgs::LBFGSERR_ROUNDING_ERROR)
      {
        if (swarm_trajs_ != nullptr)
        {
          for (size_t i = 0; i < swarm_trajs_->size(); ++i)
          {
            flag_swarm_too_close |= min_ellip_dist2_[i] <
                                    pow((swarm_clearance_ + swarm_trajs_->at(i).des_clearance) * 1.25, 2);
          }
        }

        const MINCOTraj &traj = distanceFieldMincoOpt_.getTrajectory();
        const double min_sdf = computeMinSdf(traj);
        const bool flag_collision_free = min_sdf >= sdf_collision_tol;
        const bool flag_margin_safe = min_sdf >= sdf_soft_margin;

        if (!flag_swarm_too_close && flag_collision_free)
        {
          flag_success = true;
          if (!flag_margin_safe)
          {
            ROS_WARN("ESDF optimize accepted with small clearance: min_sdf=%.3f soft_margin=%.3f tol=%.3f",
                     min_sdf, sdf_soft_margin, sdf_collision_tol);
          }
        }
        else
        {
          ROS_WARN("ESDF optimize rejected: collision_free=%s swarm_safe=%s min_sdf=%.3f safe_margin=%.3f collision_tol=%.3f cost=%.3f",
                   flag_collision_free ? "yes" : "no",
                   flag_swarm_too_close ? "no" : "yes",
                   min_sdf,
                   sdf_soft_margin,
                   sdf_collision_tol,
                   final_cost);
          restart_nums++;
          if (!flag_margin_safe || !flag_collision_free)
          {
            distance_field_cost_manager_.wei_dist *= 2.0;
          }
          if (flag_swarm_too_close)
          {
            wei_swarm_mod_ *= 2.0;
          }
        }
      }
      else
      {
        ROS_WARN("ESDF solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
        restart_nums++;
      }
    } while (!flag_success && restart_nums < 3);

    if (!flag_success && init_esdf_free)
    {
      Eigen::VectorXd grad_dummy = Eigen::VectorXd::Zero(variable_num_);
      final_cost = distanceFieldMincoOpt_.evaluate(x0, grad_dummy, time_cost_, distance_field_cost_manager_);
      ROS_WARN("ESDF optimize fallback: use feasible init trajectory (init_min_sdf=%.3f tol=%.3f).",
               init_min_sdf,
               sdf_collision_tol);
      return true;
    }

    return flag_success;
  }

  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      const spatial_map::PolyhedraH &corridor_hpolys,
      const Eigen::VectorXi *corridor_piece_idx,
      double &final_cost)
  {
    optimize_mode_ = MODE_CORRIDOR;
    resetSpatialOptimizationContext();

    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }
    if (corridor_hpolys.empty())
    {
      ROS_ERROR("corridor_hpolys is empty");
      return false;
    }

    int restart_nums = 0;
    bool flag_success = false, flag_swarm_too_close = false;
    wei_swarm_mod_ = wei_swarm_;

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    corridorMincoOpt_.setEnergyWeight(rho_energy_);
    corridorMincoOpt_.setSamplesPerPiece(std::max(cps_num_prePiece_ * 4, 16));

    Eigen::MatrixXd waypoints(piece_num_ + 1, 3);
    waypoints.row(0) = iniState.col(0).transpose();
    for (int i = 0; i < initInnerPts.cols(); ++i)
    {
      waypoints.row(i + 1) = initInnerPts.col(i).transpose();
    }
    waypoints.row(piece_num_) = finState.col(0).transpose();

    std::vector<double> time_segs(piece_num_);
    for (int i = 0; i < piece_num_; ++i)
    {
      time_segs[i] = initT(i);
    }

    spatial_map::PolyhedraH normalized_corridor = corridor_hpolys;
    for (auto &poly : normalized_corridor)
    {
      for (int r = 0; r < poly.rows(); ++r)
      {
        const double norm = poly.row(r).head<3>().norm();
        if (norm > 1.0e-9)
        {
          poly.row(r) /= norm;
        }
      }
    }

    Eigen::VectorXi segment_poly_idx(piece_num_);
    segment_poly_idx.setZero();

    if (!processCorridor(normalized_corridor, corridor_vpolys_))
    {
      ROS_ERROR("Failed to preprocess corridor into vertex polytopes.");
      return false;
    }

    bool use_gcopter_piece_idx = false;
    if (corridor_piece_idx != nullptr &&
        corridor_piece_idx->size() == static_cast<int>(normalized_corridor.size()) &&
        corridor_piece_idx->sum() == piece_num_ &&
        static_cast<int>(corridor_vpolys_.size()) == 2 * static_cast<int>(normalized_corridor.size()) - 1)
    {
      corridor_vpoly_idx_.resize(std::max(0, piece_num_ - 1));
      corridor_hpoly_idx_.resize(piece_num_);

      int j = 0;
      for (int i = 0; i < corridor_piece_idx->size(); ++i)
      {
        const int k = std::max(1, (*corridor_piece_idx)(i));
        for (int l = 0; l < k && j < piece_num_; ++l, ++j)
        {
          if (j < piece_num_ - 1)
          {
            if (l < k - 1)
            {
              corridor_vpoly_idx_(j) = 2 * i;
            }
            else if (i < corridor_piece_idx->size() - 1)
            {
              corridor_vpoly_idx_(j) = 2 * i + 1;
            }
            else
            {
              corridor_vpoly_idx_(j) = 2 * i;
            }
          }
          corridor_hpoly_idx_(j) = i;
        }
      }
      use_gcopter_piece_idx = (j == piece_num_);
    }

    if (!use_gcopter_piece_idx)
    {
      const double mapping_margin = 0.0;
      int poly_cursor = 0;
      for (int seg_idx = 0; seg_idx < piece_num_; ++seg_idx)
      {
        const Eigen::Vector3d p0 = waypoints.row(seg_idx).transpose();
        const Eigen::Vector3d p1 = waypoints.row(seg_idx + 1).transpose();
        const Eigen::Vector3d pm = 0.5 * (p0 + p1);

        int best_poly = std::min(poly_cursor, static_cast<int>(normalized_corridor.size()) - 1);
        int best_score = -1;
        for (int poly_id = poly_cursor; poly_id < static_cast<int>(normalized_corridor.size()); ++poly_id)
        {
          int score = 0;
          score += pointInsidePolytope(p0, normalized_corridor[poly_id], mapping_margin) ? 1 : 0;
          score += pointInsidePolytope(pm, normalized_corridor[poly_id], mapping_margin) ? 1 : 0;
          score += pointInsidePolytope(p1, normalized_corridor[poly_id], mapping_margin) ? 1 : 0;
          if (score > best_score)
          {
            best_score = score;
            best_poly = poly_id;
          }
          if (score == 3)
          {
            break;
          }
        }
        segment_poly_idx(seg_idx) = best_poly;
        poly_cursor = best_poly;
      }

      corridor_hpoly_idx_ = segment_poly_idx;
      corridor_vpoly_idx_.resize(std::max(0, piece_num_ - 1));
      for (int i = 0; i < piece_num_ - 1; ++i)
      {
        const int left_poly = corridor_hpoly_idx_(i);
        const int right_poly = corridor_hpoly_idx_(i + 1);
        int v_poly_id = std::min(2 * std::max(left_poly, 0),
                                 static_cast<int>(corridor_vpolys_.size()) - 1);
        if (right_poly == left_poly + 1 &&
            2 * left_poly + 1 < static_cast<int>(corridor_vpolys_.size()))
        {
          v_poly_id = 2 * left_poly + 1;
        }
        corridor_vpoly_idx_(i) = v_poly_id;
      }
      ROS_WARN("Corridor mapping fallback: use geometric midpoint assignment instead of GCOPTER piece indexing.");
    }

    corridorSpatialMap_.reset(&corridor_vpolys_, &corridor_vpoly_idx_, piece_num_);
    corridorMincoOpt_.setSpatialMap(&corridorSpatialMap_);
    corridorMincoOpt_.setInitState(time_segs, waypoints, iniState, finState);

    corridor_cost_manager_.setCorridor(&normalized_corridor, &corridor_hpoly_idx_);
    corridor_cost_manager_.cps = &cps_;
    corridor_cost_manager_.swarm_traj = swarm_trajs_;
    corridor_cost_manager_.wei_corridor = wei_corridor_;
    corridor_cost_manager_.wei_swarm = wei_swarm_mod_;
    corridor_cost_manager_.wei_feas = wei_feas_;
    corridor_cost_manager_.wei_sqrvar = wei_sqrvar_;
    corridor_cost_manager_.corridor_clearance = corridor_clearance_;
    corridor_cost_manager_.corridor_smoothing = corridor_smoothing_;
    corridor_cost_manager_.swarm_clearance = swarm_clearance_;
    corridor_cost_manager_.max_vel = max_vel_;
    corridor_cost_manager_.max_acc = max_acc_;
    corridor_cost_manager_.max_jer = max_jer_;
    corridor_cost_manager_.drone_id = drone_id_;
    corridor_cost_manager_.t_now = t_now_;
    corridor_cost_manager_.touch_goal = touch_goal_;
    corridor_cost_manager_.min_ellip_dist2_ptr = &min_ellip_dist2_;

    Eigen::VectorXd x0 = corridorMincoOpt_.generateInitialGuess();
    variable_num_ = x0.size();

    double x_init[variable_num_];
    memcpy(x_init, x0.data(), variable_num_ * sizeof(double));

    min_ellip_dist2_.resize(swarm_trajs_ ? swarm_trajs_->size() : 0);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;

    do
    {
      iter_num_ = 0;
      force_stop_type_ = DONT_STOP;
      flag_success = false;
      flag_swarm_too_close = false;
      corridor_cost_manager_.wei_swarm = wei_swarm_mod_;

      const int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP ||
          result == lbfgs::LBFGSERR_ROUNDING_ERROR)
      {
        if (swarm_trajs_ != nullptr)
        {
          for (size_t i = 0; i < swarm_trajs_->size(); ++i)
          {
            flag_swarm_too_close |= min_ellip_dist2_[i] <
                                    pow((swarm_clearance_ + swarm_trajs_->at(i).des_clearance) * 1.25, 2);
          }
        }
        const bool flag_collision_free =
            isTrajectoryCollisionFree(corridorMincoOpt_.getTrajectory());

        const double hard_corridor_margin =
            grid_map_ ? -0.5 * grid_map_->getResolution() : -1.0e-3;
        const bool flag_inside_corridor =
            isTrajectoryInsideCorridor(corridorMincoOpt_.getTrajectory(),
                                      normalized_corridor,
                                      hard_corridor_margin);

        if (!flag_swarm_too_close && flag_collision_free && flag_inside_corridor)
        {
          flag_success = true;
        }
        else
        {
          const TrajectoryCheckDebug debug =
              analyzeTrajectoryCheck(corridorMincoOpt_.getTrajectory(),
                                     grid_map_,
                                     normalized_corridor,
                                     corridor_clearance_,
                                     max_vel_);

          ROS_WARN("Corridor optimize rejected: inside_corridor=%s collision_free=%s swarm_safe=%s cost=%.3f "
                   "first_collision_t=%.3f first_collision_pt=[%.2f %.2f %.2f] "
                   "max_corridor_violation=%.3f worst_corridor_t=%.3f worst_corridor_pt=[%.2f %.2f %.2f]",
                   flag_inside_corridor ? "yes" : "no",
                   flag_collision_free ? "yes" : "no",
                   flag_swarm_too_close ? "no" : "yes",
                   final_cost,
                   debug.first_collision_t,
                   debug.first_collision_pt.x(),
                   debug.first_collision_pt.y(),
                   debug.first_collision_pt.z(),
                   debug.max_corridor_violation,
                   debug.worst_corridor_t,
                   debug.worst_corridor_pt.x(),
                   debug.worst_corridor_pt.y(),
                   debug.worst_corridor_pt.z());
          restart_nums++;
          if (!flag_inside_corridor)
          {
            corridor_cost_manager_.wei_corridor *= 2.0;
          }
          if (flag_swarm_too_close)
          {
            wei_swarm_mod_ *= 2.0;
          }
        }
      }
      else
      {
        ROS_WARN("Corridor solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
        ROS_WARN_COND(VERBOSE_OUTPUT, "Corridor solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
        restart_nums++;
      }
    } while (!flag_success && restart_nums < 3);

    if (flag_success)
    {
      Eigen::Map<const Eigen::VectorXd> x_final(x_init, variable_num_);
      corridorMincoOpt_.setWarmStartGuess(x_final);
    }
    return flag_success;
  }

  // =====================================================
  //  Collision detection and constraint point management
  // =====================================================
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const MINCOTraj &traj,
      const Eigen::MatrixXd &init_points,
      const bool flag_first_init)
  {
    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    int in_id = -1, out_id = -1;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    
    int i_end = ConstraintPoints::two_thirds_id(const_cast<Eigen::MatrixXd &>(init_points), touch_goal_);

    PtsChk_t pts_check;
    if (!computePointsToCheck(traj, i_end, pts_check))
      return CHK_RET::ERR;

    for (int i = 0; i < i_end; ++i)
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j)
      {
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second);

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)
          {
            in_id = i;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false;
        }
        else if (!occ && last_occ)
        {
          out_id = i + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    if (segment_ids.size() == 0)
      return CHK_RET::OBS_FREE;

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())
      {
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A corner case 2, I have never exeam it.");
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");
        return CHK_RET::ERR;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      if (i == 0)
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2);
        else
          id_up_bound = init_points.cols() - 2;
      }
      else if (i == segment_ids.size() - 1)
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2);
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2);
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);
      }
      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0;
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);
        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++)
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {
          last_Astar_id = Astar_id;
          if (val >= 0)
          {
            ++Astar_id;
            if (Astar_id >= (int)a_star_pathes[i].size())
              break;
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
              break;
          }
          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);
          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])));
            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ_test = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
              if (occ_test || a < grid_map_->getResolution())
              {
                if (occ_test)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
            got_intersection_id = -1;
        }
      }

      /* Corner case */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {
          last_Astar_id = Astar_id;
          if (val >= 0)
          {
            ++Astar_id;
            if (Astar_id >= (int)a_star_pathes[i].size())
              break;
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
              break;
          }
          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);
          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])));
            if ((intersection_point - middle_point).norm() > 0.01)
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());
              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }
        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
    }

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

  // =====================================================
  //  Rebound detection (during optimization)
  // =====================================================
  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_);
    for (int i = 1; i <= i_end; ++i)
    {
      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution())
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j_inner;
        for (j_inner = i - 1; j_inner >= 0; --j_inner)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j_inner));
          if (!occ)
          {
            in_id = j_inner;
            break;
          }
        }
        if (j_inner < 0)
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }

        for (j_inner = i + 1; j_inner < cps_.cp_size; ++j_inner)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j_inner));
          if (!occ)
          {
            out_id = j_inner;
            break;
          }
        }
        if (j_inner >= cps_.cp_size)
        {
          ROS_WARN("Local target in collision, skip this planning.");
          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j_inner + 1;
        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
          a_star_pathes.push_back(a_star_->getPath());
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A corner case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++)
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {
            last_Astar_id = Astar_id;
            if (val >= 0)
            {
              ++Astar_id;
              if (Astar_id >= (int)a_star_pathes[i].size())
                break;
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
                break;
            }
            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);
            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0))
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])));
              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ_test = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                if (occ_test || a < grid_map_->getResolution())
                {
                  if (occ_test)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
              got_intersection_id = -1;
          }
        }

        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }
          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  // =====================================================
  //  Rebound gate
  // =====================================================
  bool PolyTrajOptimizer::allowRebound(void)
  {
    if (iter_num_ < 3)
      return false;

    double min_product = 1;
    for (int i = 3; i <= cps_.points.cols() - 4; ++i)
    {
      double product = ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized()).dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
      if (product < min_product)
        min_product = product;
    }
    if (min_product < 0.87)
      return false;

    if (multitopology_data_.use_multitopology_trajs)
    {
      if (!multitopology_data_.initial_obstacles_avoided)
      {
        bool avoided = true;
        for (int i = 1; i < cps_.points.cols() - 1; ++i)
        {
          if (cps_.base_point[i].size() > 0)
          {
            if ((cps_.points.col(i) - cps_.base_point[i][0]).dot(cps_.direction[i][0]) < 0)
            {
              avoided = false;
              break;
            }
          }
        }
        multitopology_data_.initial_obstacles_avoided = avoided;
      }
      if (!multitopology_data_.initial_obstacles_avoided)
        return false;
    }

    return true;
  }

  // =====================================================
  //  Multi-topology generation
  // =====================================================
  std::vector<ConstraintPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0)
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ConstraintPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);

    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
      ConstraintPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);
    }

    for (int i = 0; i < seg_upbound; i++)
    {
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        if (occ_start_id == -1 || occ_end_id == -1)
        {
          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
          continue;
        }

        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");
            std::vector<ConstraintPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          if (j == occ_start_id)
            base_pt_reverse = occ_start_pt;
          else if (j == occ_end_id)
            base_pt_reverse = occ_end_pt;
          else
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();

          if (grid_map_->getInflateOccupancy(base_pt_reverse))
          {
            double l_upbound = 5 * CTRL_PT_DIST;
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);
              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;
              goto exit_multi_loop3;
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION)
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!");
            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
            goto exit_multi_loop3;
          }
        }

        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse))
        {
          double l_upbound = 5 * CTRL_PT_DIST;
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION)
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    if (seg_upbound == 0)
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1;
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      ConstraintPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (cp_id < cps_.cp_size)
      {
        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id])
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
              goto abandon_this_trajectory;
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Should not happen!!!!");
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  }

  bool PolyTrajOptimizer::isTrajectoryCollisionFree(const MINCOTraj &traj) const
  {
    if (!grid_map_)
    {
      return true;
    }

    const double t_step = std::min(0.05, grid_map_->getResolution() / std::max(max_vel_, 0.1));
    const double total_duration = traj.getTotalDuration();
    for (double t = 0.0; t <= total_duration + 1.0e-6; t += t_step)
    {
      const double sample_t = std::min(t, total_duration);
      if (grid_map_->getInflateOccupancy(traj.evaluate(sample_t, 0)) != 0)
      {
        return false;
      }
    }

    return true;
  }

  bool PolyTrajOptimizer::pointInsidePolytope(const Eigen::Vector3d &pt,
                                            const spatial_map::PolyhedronH &hpoly,
                                            double margin) const
  {
    for (int i = 0; i < hpoly.rows(); ++i)
    {
      if (hpoly.row(i).head<3>().dot(pt) + hpoly(i, 3) > -margin)
        return false;
    }
    return true;
  }

  bool PolyTrajOptimizer::pointInsideCorridor(const Eigen::Vector3d &pt,
                                              const spatial_map::PolyhedraH &corridor_hpolys,
                                              double margin) const
  {
    for (const auto &poly : corridor_hpolys)
    {
      if (pointInsidePolytope(pt, poly, margin))
        return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::isTrajectoryInsideCorridor(const MINCOTraj &traj,
                                                    const spatial_map::PolyhedraH &corridor_hpolys,
                                                    double margin) const
  {
    const double total_duration = traj.getTotalDuration();
    const double dt = std::min(0.05, 0.5 * corridor_smoothing_);
    for (double t = 0.0; t <= total_duration + 1.0e-6; t += dt)
    {
      const double sample_t = std::min(t, total_duration);
      const Eigen::Vector3d pt = traj.evaluate(sample_t, 0);
      if (!pointInsideCorridor(pt, corridor_hpolys, margin))
        return false;
    }
    return true;
  }

  // =====================================================
  //  Setters
  // =====================================================
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    nh.param("optimization/weight_corridor", wei_corridor_, 1000.0);
    nh.param("optimization/weight_distance_field", wei_dist_, -1.0);
    nh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/weight_energy", rho_energy_, 1.0);
    nh.param("optimization/safety_margin", safety_margin_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    nh.param("optimization/corridor_clearance", corridor_clearance_, 0.0);
    nh.param("optimization/corridor_smoothing", corridor_smoothing_, 0.05);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);

    if (wei_dist_ < 0.0)
    {
      wei_dist_ = wei_obs_;
    }
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;
    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.cp_size = points.cols();
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(Types::SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }
  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }
  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }
  void PolyTrajOptimizer::setConstraintPoints(Types::ConstraintPoints cps) { cps_ = cps; }
  void PolyTrajOptimizer::setUseMultitopologyTrajs(bool use_multitopology_trajs) { multitopology_data_.use_multitopology_trajs = use_multitopology_trajs; }
}//namespace ego_planner
