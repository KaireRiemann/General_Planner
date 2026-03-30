#include <plan_manage/planner_manager.h>
#include <SFCGenerator/sfc_gen.hpp>
#include <SFCGenerator/geo_utils.hpp>
#include <SFCGenerator/quickhull.hpp>
#include "visualization_msgs/Marker.h"

#include <cmath>
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
    nh.param("manager/sfc_path_timeout", sfc_path_timeout_, 0.2);
    nh.param("manager/sfc_progress", sfc_progress_, 0.75);
    nh.param("manager/sfc_range", sfc_range_, 0.8);
    ROS_INFO("Local planner obstacle mode: %s", use_sfc_corridor_ ? "sfc_corridor" : "guide_points");

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

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
    }

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
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

  bool EGOPlannerManager::prepareLocalGuideAndCorridor(
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &goal_pt,
      std::vector<Eigen::Vector3d> &guide_path,
      spatial_map::PolyhedraH &corridor_hpolys)
  {
    std::vector<Eigen::Vector3d> raw_guide_path;
    if (!searchLocalGuidePath(start_pt, goal_pt, raw_guide_path))
    {
      ROS_WARN("Local guide path search failed.");
      return false;
    }

    sfc_gen::refineSeedPath(raw_guide_path, grid_map_.get(), sfc_progress_, sfc_range_, guide_path);
    if (guide_path.size() < 2)
    {
      guide_path = raw_guide_path;
    }

    if (!generateSafeFlightCorridor(guide_path, corridor_hpolys))
    {
      return false;
    }

    visualization_->displayGlobalPathList(raw_guide_path, 0.08, 0);
    visualization_->displayInitPathList(guide_path, 0.12, 0);
    std::vector<Eigen::Vector3d> corridor_triangles;
    std::vector<Eigen::Vector3d> corridor_edges;
    buildCorridorVisualization(corridor_hpolys, corridor_triangles, corridor_edges);
    visualization_->displayCorridor(corridor_triangles, corridor_edges, 0);
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

    std::vector<double> accum_len(guide_path.size(), 0.0);
    for (std::size_t i = 1; i < guide_path.size(); ++i)
    {
      accum_len[i] = accum_len[i - 1] + (guide_path[i] - guide_path[i - 1]).norm();
    }

    const double total_len = accum_len.back();
    const int piece_num = std::max(2, static_cast<int>(std::ceil(total_len / std::max(pp_.polyTraj_piece_length, 0.1))));
    inner_pts.resize(3, std::max(0, piece_num - 1));
    durations.resize(piece_num);

    auto samplePolyline = [&](double s) -> Eigen::Vector3d
    {
      if (s <= 0.0)
      {
        return guide_path.front();
      }
      if (s >= total_len)
      {
        return guide_path.back();
      }

      for (std::size_t i = 1; i < accum_len.size(); ++i)
      {
        if (s <= accum_len[i])
        {
          const double seg_len = std::max(accum_len[i] - accum_len[i - 1], 1.0e-6);
          const double ratio = (s - accum_len[i - 1]) / seg_len;
          return guide_path[i - 1] * (1.0 - ratio) + guide_path[i] * ratio;
        }
      }
      return guide_path.back();
    };

    for (int i = 0; i < piece_num; ++i)
    {
      const double s0 = total_len * static_cast<double>(i) / static_cast<double>(piece_num);
      const double s1 = total_len * static_cast<double>(i + 1) / static_cast<double>(piece_num);
      const Eigen::Vector3d p0 = samplePolyline(s0);
      const Eigen::Vector3d p1 = samplePolyline(s1);
      const double seg_len = (p1 - p0).norm();
      durations(i) = std::max(seg_len / std::max(pp_.max_vel_, 0.1), 0.2);
      if (i < piece_num - 1)
      {
        inner_pts.col(i) = p1;
      }
    }

    return true;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const std::vector<Eigen::Vector3d> &guide_path,
      const spatial_map::PolyhedraH &corridor_hpolys, const bool touch_goal)
  {
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    static int count = 0;
    std::cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << std::endl;

    ploy_traj_opt_->setIfTouchGoal(touch_goal);

    MINCOTraj3D initTraj;
    Eigen::MatrixXd innerPts;
    Eigen::VectorXd durations;
    MINCOBoundaryState3D headState, tailState;

    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(local_target_pt, local_target_vel, Eigen::Vector3d::Zero());

    if (!buildGuideInitialGuess(guide_path, innerPts, durations))
    {
      return false;
    }
    if (!initTraj.generate(innerPts, headState, tailState, durations))
    {
      ROS_ERROR("Failed to generate corridor-seeded MINCO trajectory.");
      return false;
    }

    Eigen::MatrixXd cstr_pts = initTraj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
    {
      point_set.push_back(cstr_pts.col(i));
    }
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    double final_cost = 0.0;
    const bool flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                                 innerPts, durations,
                                                                 corridor_hpolys, final_cost);

    t_opt = ros::Time::now() - t_start;

    std::cout << "Success=" << (flag_success ? "yes" : "no") << std::endl;
    if (flag_success)
    {
      static double sum_time = 0.0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).toSec();
      count_success++;

      MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
      setLocalTrajFromOpt(opt_traj, touch_goal);
      cstr_pts = opt_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayOptimalList(cstr_pts, 0);

      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);
      continous_failures_count_ = 0;
    }
    else
    {
      MINCOTraj3D fail_traj = ploy_traj_opt_->getTrajectory();
      cstr_pts = fail_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
    }

    return flag_success;
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

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    static int count = 0;
    std::cout << "\033[47;30m\n[" << t_start << "] Drone " << pp_.drone_id << " Replan " << count++ << "\033[0m" << std::endl;

    /*** STEP 1: INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    MINCOTraj3D initTraj;
    Eigen::MatrixXd innerPts;
    Eigen::VectorXd durations;
    MINCOBoundaryState3D headState, tailState;
    spatial_map::PolyhedraH corridor_hpolys;

    headState = makeBoundaryState(start_pt, start_vel, start_acc);
    tailState = makeBoundaryState(local_target_pt, local_target_vel, Eigen::Vector3d::Zero());

    if (use_sfc_corridor_)
    {
      std::vector<Eigen::Vector3d> guide_path;
      if (!prepareLocalGuideAndCorridor(start_pt, local_target_pt, guide_path, corridor_hpolys) ||
          !buildGuideInitialGuess(guide_path, innerPts, durations))
      {
        return false;
      }
      initTraj = generateMINCOTraj(headState, tailState, innerPts, durations);
    }
    else
    {
      if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                            flag_polyInit, flag_randomPolyTraj, ts,
                            initTraj, innerPts, durations, headState, tailState))
      {
        return false;
      }
    }

    Eigen::MatrixXd cstr_pts = initTraj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    std::vector<std::pair<int, int>> segments;
    if (!use_sfc_corridor_)
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

    if (use_sfc_corridor_)
    {
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, durations,
                                                        corridor_hpolys, final_cost);

      t_opt = ros::Time::now() - t_start;

      if (flag_success)
      {
        MINCOTraj3D opt_traj = ploy_traj_opt_->getTrajectory();
        setLocalTrajFromOpt(opt_traj, touch_goal);
        cstr_pts = opt_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
        visualization_->displayOptimalList(cstr_pts, 0);
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
      cstr_pts = fail_traj.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
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
