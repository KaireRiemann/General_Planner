#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h"

namespace ego_planner
{

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

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  // Helper: generate trajectory from headState, tailState, innerPts, durations
  static PPoly3D generateSplineTraj(
      const Eigen::Matrix<double, 3, 3> &headState,
      const Eigen::Matrix<double, 3, 3> &tailState,
      const Eigen::MatrixXd &innerPts,
      const Eigen::VectorXd &durations)
  {
    int piece_num = durations.size();
    WaypointsMat waypoints(innerPts.cols() + 2, 3);
    waypoints.row(0) = headState.col(0).transpose();
    for (int i = 0; i < innerPts.cols(); ++i)
      waypoints.row(i + 1) = innerPts.col(i).transpose();
    waypoints.row(innerPts.cols() + 1) = tailState.col(0).transpose();

    BCs bc;
    bc.start_velocity = headState.col(1);
    bc.start_acceleration = headState.col(2);
    bc.end_velocity = tailState.col(1);
    bc.end_acceleration = tailState.col(2);

    std::vector<double> time_segs(piece_num);
    for (int i = 0; i < piece_num; ++i)
      time_segs[i] = durations(i);

    SplineTrajectory::QuinticSplineND<3> spline;
    spline.update(time_segs, waypoints, 0.0, bc);
    return spline.getTrajectoryCopy();
  }

  // Helper: get durations from PPoly3D
  static Eigen::VectorXd getDurationsFromTraj(const PPoly3D &traj)
  {
    int num_segs = traj.getNumSegments();
    Eigen::VectorXd durs(num_segs);
    for (int i = 0; i < num_segs; ++i)
      durs(i) = (*(traj.begin() + i)).duration();
    return durs;
  }

  // Helper: get max velocity rate from PPoly3D
  static double getMaxVelRate(const PPoly3D &traj)
  {
    double maxVel = 0.0;
    double dt = 0.01;
    for (double t = traj.getStartTime(); t <= traj.getEndTime(); t += dt)
    {
      double vel = traj.evaluate(t, SplineTrajectory::Deriv::Vel).norm();
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

    PPoly3D initTraj;
    Eigen::MatrixXd innerPts;
    Eigen::VectorXd durations;
    Eigen::Matrix<double, 3, 3> headState, tailState;

    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts,
                          initTraj, innerPts, durations, headState, tailState))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = ploy_traj_opt_->getInitConstraintPoints(initTraj, durations, ploy_traj_opt_->get_cps_num_prePiece_());
    std::vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initTraj, cstr_pts, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
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

    if (pp_.use_multitopology_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      double final_cost, min_cost = 999999.0;
      PPoly3D best_traj;
      Eigen::VectorXd best_durations;

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
            const SplineTraj &opt_spline = ploy_traj_opt_->getWorkingSpline();
            best_traj = opt_spline.getTrajectoryCopy();
            best_durations = getDurationsFromTraj(best_traj);
            flag_success = true;
          }

          // visualization
          const SplineTraj &vis_spline = ploy_traj_opt_->getWorkingSpline();
          PPoly3D vis_traj = vis_spline.getTrajectoryCopy();
          Eigen::VectorXd vis_durs = getDurationsFromTraj(vis_traj);
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getInitConstraintPoints(vis_traj, vis_durs, ploy_traj_opt_->get_cps_num_prePiece_());
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
        setLocalTrajFromOpt(best_traj, best_durations, touch_goal);
        cstr_pts = ploy_traj_opt_->getInitConstraintPoints(best_traj, best_durations, ploy_traj_opt_->get_cps_num_prePiece_());
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
        const SplineTraj &opt_spline = ploy_traj_opt_->getWorkingSpline();
        PPoly3D opt_traj = opt_spline.getTrajectoryCopy();
        Eigen::VectorXd opt_durs = getDurationsFromTraj(opt_traj);
        setLocalTrajFromOpt(opt_traj, opt_durs, touch_goal);
        cstr_pts = ploy_traj_opt_->getInitConstraintPoints(opt_traj, opt_durs, ploy_traj_opt_->get_cps_num_prePiece_());
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
      const SplineTraj &fail_spline = ploy_traj_opt_->getWorkingSpline();
      PPoly3D fail_traj = fail_spline.getTrajectoryCopy();
      Eigen::VectorXd fail_durs = getDurationsFromTraj(fail_traj);
      cstr_pts = ploy_traj_opt_->getInitConstraintPoints(fail_traj, fail_durs, ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayFailedList(cstr_pts, 0);

      continous_failures_count_++;
    }

    return flag_success;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      PPoly3D &initTraj, Eigen::MatrixXd &outInnerPts, Eigen::VectorXd &outDurations,
      Eigen::Matrix<double, 3, 3> &headState, Eigen::Matrix<double, 3, 3> &tailState)
  {
    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit)
    {
      flag_first_call = false;

      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

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
      PPoly3D initOfInitTraj = generateSplineTraj(headState, tailState, innerPs, piece_dur_vec);

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
      double start_time = initOfInitTraj.getStartTime();
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initOfInitTraj.evaluate(start_time + t, SplineTrajectory::Deriv::Pos);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }

      initTraj = generateSplineTraj(headState, tailState, innerPs, piece_dur_vec);

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

      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);

      double t = piece_dur_vec(0);
      double lc_start = traj_.local_traj.traj.getStartTime();
      double glb_start = traj_.global_traj.traj.getStartTime();
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.evaluate(lc_start + t + passed_t_on_lctraj, SplineTrajectory::Deriv::Pos);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.evaluate(glb_start + glb_t, SplineTrajectory::Deriv::Pos);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initTraj = generateSplineTraj(headState, tailState, innerPs, piece_dur_vec);
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
    double glb_start = traj_.global_traj.traj.getStartTime();

    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      double local_t = t - traj_.global_traj.global_start_time;
      Eigen::Vector3d pos_t = traj_.global_traj.traj.evaluate(glb_start + local_t, SplineTrajectory::Deriv::Pos);
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
      local_target_vel = traj_.global_traj.traj.evaluate(glb_start + local_t, SplineTrajectory::Deriv::Vel);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const PPoly3D &traj, const Eigen::VectorXd &durations, const bool touch_goal)
  {
    Eigen::MatrixXd cps = ploy_traj_opt_->getInitConstraintPoints(traj, durations, getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    {
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());
    }

    return ret;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    Eigen::MatrixXd innerPs = stop_pos; // 3x1
    Eigen::VectorXd durs = Eigen::Vector2d(1.0, 1.0);

    PPoly3D stopTraj = generateSplineTraj(headState, tailState, innerPs, durs);
    setLocalTrajFromOpt(stopTraj, durs, false);

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

    double my_base = traj_.local_traj.traj.getStartTime();
    double other_base = traj_.swarm_traj[drone_id].traj.getStartTime();

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.evaluate(my_base + (t - my_traj_start_time), SplineTrajectory::Deriv::Pos) -
           traj_.swarm_traj[drone_id].traj.evaluate(other_base + (t - other_traj_start_time), SplineTrajectory::Deriv::Pos))
              .norm() < (getSwarmClearance() + traj_.swarm_traj[drone_id].des_clearance))
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
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
        innerPts.col(i) = waypoints[i];
    }

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    PPoly3D globalTraj;
    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                                : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalTraj = generateSplineTraj(headState, tailState, innerPts, time_vec);

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
