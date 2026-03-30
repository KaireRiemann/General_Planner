#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    /* fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);
    nh.param("manager/use_sfc_corridor", use_sfc_corridor_, false);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    have_trigger_ = !flag_realworld_experiment_;
    no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::PolyTraj>("planning/broadcast_traj_recv", 100,
                                                                 &EGOReplanFSM::RecvBroadcastPolyTrajCallback,
                                                                 this,
                                                                 ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else
      std::cout << "Wrong target_type_ value! target_type_=" << target_type_ << std::endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); 
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_) goto force_return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_) goto force_return;
      else changeFSMExecState(SEQUENTIAL_START, "FSM");
      break;
    }

    case SEQUENTIAL_START: 
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); 
        if (success) changeFSMExecState(EXEC_TRAJ, "FSM");
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); 
        }
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      bool success = planFromGlobalTraj(10); 
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else changeFSMExecState(GEN_NEW_TRAJ, "FSM"); 
      break;
    }

    case REPLAN_TRAJ:
    {
      if (planFromLocalTraj(1)) changeFSMExecState(EXEC_TRAJ, "FSM");
      else changeFSMExecState(REPLAN_TRAJ, "FSM");
      break;
    }

    case EXEC_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = std::min(info->duration, t_cur);
      
      // NUBS 接口求值：0代表Position
      Eigen::Vector3d pos = info->traj.evaluate(t_cur, 0); 
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

      // NUBS 保凸性预警：通过判断离最后几个控制点的时间是否逼近，判断轨迹是否即将执行完毕
      bool close_to_current_traj_end = (info->duration - t_cur) < emergency_time_; 

      if (mondifyInCollisionFinalGoal()) 
      {
        // pass
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) 
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) 
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) 
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) callEmergencyStop(odom_pos_);
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == exec_state_) continously_called_times_++;
    else continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    std::cout << "[" + pos_call + "]" << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    std::cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
      std::cout << ". Waiting for ";
    if (!have_odom_) std::cout << "odom,";
    if (!have_target_) std::cout << "target,";
    if (!have_trigger_) std::cout << "trigger,";
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_) std::cout << "prev traj,";
    std::cout << std::endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  // =================================================================================
  // 核心重构：利用控制点 + Greville 时间映射进行极速碰撞检测
  // =================================================================================
 void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    
    const double t_cur = ros::Time::now().toSec() - info->start_time;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0) return;

    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return;
    }

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    double t_end = info->duration;
    if (!touch_the_end) t_end *= 0.75;

    if (t_cur >= t_end) return;

    constexpr double t_step = 0.05;
    for (double t = t_cur; t < t_end; t += t_step)
    {
      Eigen::Vector3d pt =info->traj.getPos(t);
      bool dangerous = false;

      if (map->getInflateOccupancy(pt)) 
      {
          dangerous = true;
      }

  
      if (!dangerous) 
      {
        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_predicted = planner_manager_->traj_.swarm_traj.at(id).traj.evaluate(t_X, 0);
            double dist = (pt - swarm_predicted).norm();
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;
            
            if (dist < allowed_dist)
            {
              ROS_WARN("Swarm warning: drone %d and %d too close (%f m) at future t=%f",
                       planner_manager_->pp_.drone_id, (int)id, dist, t);
              dangerous = true;
              break;
            }
          }
        }
      }

      // 4. 危机处理逻辑
      if (dangerous)
      {
        if (planFromLocalTraj(1)) // 尝试利用优化器重新拉回安全区
        {
          ROS_INFO("Plan success when detect collision at future t=%f", t);
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
        }
        else
        {
          if (t - t_cur < emergency_time_) // 距离撞击时间不足，直接急刹
          {
            ROS_WARN("Emergency stop! Crash in %f seconds", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            ROS_WARN("Future collision detected, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
        }
        return; 
      }
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    polyTraj2ROSMsg(poly_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(poly_msg);

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (use_sfc_corridor_)
    {
      if (!prepareLocalGuideAndCorridor(start_pt_, start_vel_, start_acc_))
      {
        return false;
      }

      bool plan_success = planner_manager_->reboundReplan(
          start_pt_, start_vel_, start_acc_,
          local_target_pt_, local_target_vel_,
          local_guide_path_, local_corridor_hpolys_, touch_goal_);

      have_new_target_ = false;

      if (plan_success)
      {
        traj_utils::PolyTraj poly_msg;
        polyTraj2ROSMsg(poly_msg);
        poly_traj_pub_.publish(poly_msg);
        broadcast_ploytraj_pub_.publish(poly_msg);
      }

      return plan_success;
    }

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::PolyTraj poly_msg;
      polyTraj2ROSMsg(poly_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(poly_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::prepareLocalGuideAndCorridor(const Eigen::Vector3d &start_pt,
                                                  const Eigen::Vector3d &start_vel,
                                                  const Eigen::Vector3d &start_acc)
  {
    (void)start_vel;
    (void)start_acc;

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    const double pos_tol = std::max(planner_manager_->grid_map_->getResolution() * 2.0, 0.2);
    if (have_local_corridor_seed_ &&
        (corridor_seed_start_ - start_pt).norm() < pos_tol &&
        (corridor_seed_goal_ - local_target_pt_).norm() < pos_tol)
    {
      return true;
    }

    if (!planner_manager_->prepareLocalGuideAndCorridor(start_pt, local_target_pt_,
                                                        local_guide_path_, local_corridor_hpolys_))
    {
      have_local_corridor_seed_ = false;
      return false;
    }

    corridor_seed_start_ = start_pt;
    corridor_seed_goal_ = local_target_pt_;
    have_local_corridor_seed_ = true;
    return true;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times) 
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init = (!use_sfc_corridor_) && (timesOfConsecutiveStateCalls().first != 1);

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_abs = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.evaluate(t_abs, 0);
    start_vel_ = info->traj.evaluate(t_abs, 1);
    start_acc_ = info->traj.evaluate(t_abs, 2);

    if (use_sfc_corridor_)
    {
      for (int i = 0; i < trial_times; ++i)
      {
        if (callReboundReplan(true, false))
        {
          return true;
        }
      }
      return false;
    }

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success) break;
        }
        if (!success) return false;
      }
    }
    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      final_goal_ = next_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.evaluate(i * step_size_t, 0);
      }

      have_target_ = true;
      have_new_target_ = true;
      have_local_corridor_seed_ = false;

      if (use_sfc_corridor_ && have_odom_)
      {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        prepareLocalGuideAndCorridor(start_pt_, start_vel_, start_acc_);
      }

      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  bool EGOReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      Eigen::Vector3d orig_goal = final_goal_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;
      
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step)
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.evaluate(t, 0);
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          if (planNextWaypoint(pt)) 
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }
      }
    }
    return false;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1) return;
    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp)) have_trigger_ = true;
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0) return;
    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    std::cout << "Triggered!" << std::endl;
  }

  // ==================================================================
  // ROS 集群通讯模块：基于紧凑 MINCO 参数编解码
  // ==================================================================
  void EGOReplanFSM::RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg)
  {
    constexpr int kBoundaryNum = MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM;
    constexpr int kOrder = MINCOTraj3D::ORDER;
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) return;
    if (msg->drone_id < 0 || msg->order != kOrder) return;

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {
      if (abs((t_now - msg->start_time).toSec()) > 10.0) 
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs", msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    if (msg->start_time.toSec() <= planner_manager_->traj_.swarm_traj[recv_id].start_time) return;

    const int M = static_cast<int>(msg->duration.size());
    if(M == 0) return;

    Eigen::VectorXd T(M);
    for (int i = 0; i < M; ++i) T(i) = msg->duration[i];

    const int Nc = msg->coef_x.size();
    const int expected = M + 2 * kBoundaryNum - 1;
    if (Nc != expected) return;

    Eigen::MatrixXd C(Nc, 3);
    for (int i = 0; i < Nc; ++i)
    {
      C(i, 0) = msg->coef_x[i];
      C(i, 1) = msg->coef_y[i];
      C(i, 2) = msg->coef_z[i];
    }

    MINCOBoundaryState3D headState = MINCOBoundaryState3D::Zero();
    MINCOBoundaryState3D tailState = MINCOBoundaryState3D::Zero();
    for (int d = 0; d < kBoundaryNum; ++d)
    {
      headState.col(d) = C.row(d).transpose();
      tailState.col(d) = C.row(Nc - kBoundaryNum + d).transpose();
    }

    Eigen::MatrixXd P_inner;
    if (M > 1) P_inner = C.block(kBoundaryNum, 0, M - 1, 3).transpose();
    else P_inner.resize(3, 0);

    MINCOTraj3D trajectory;
    trajectory.generate(P_inner, headState, tailState, T);

    Eigen::MatrixXd cps_chk = trajectory.getInitConstraintPoints(planner_manager_->getCpsNumPrePiece());
    bool far_away = true;
    for (int i = 0; i < cps_chk.cols(); ++i)
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) 
      {
        far_away = false;
        break;
      }
    }

    if (!far_away || !have_recv_pre_agent_) 
    {
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.evaluate(0.0, 0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      if (planner_manager_->checkCollision(recv_id))
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");

      if (!have_recv_pre_agent_)
      {
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
        {
          for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].drone_id != i) break;
            have_recv_pre_agent_ = true;
          }
        }
      }
    }
    else planner_manager_->traj_.swarm_traj[recv_id].drone_id = -1; 
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg)
  {
    constexpr int kBoundaryNum = MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM;
    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int M = durs.size();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = MINCOTraj3D::ORDER;
    poly_msg.des_clearance = planner_manager_->getSwarmClearance();
    
    poly_msg.duration.resize(M);
    for (int i = 0; i < M; ++i) poly_msg.duration[i] = durs(i);

    int num_encoded = M + 2 * kBoundaryNum - 1;
    poly_msg.coef_x.resize(num_encoded);
    poly_msg.coef_y.resize(num_encoded);
    poly_msg.coef_z.resize(num_encoded);

    double T_total = data->traj.getTotalDuration();
    for (int d = 0; d < kBoundaryNum; ++d) {
        Eigen::Vector3d head_d = data->traj.evaluate(0.0, d);
        poly_msg.coef_x[d] = head_d(0);
        poly_msg.coef_y[d] = head_d(1);
        poly_msg.coef_z[d] = head_d(2);

        Eigen::Vector3d tail_d = data->traj.evaluate(T_total, d);
        poly_msg.coef_x[num_encoded - kBoundaryNum + d] = tail_d(0);
        poly_msg.coef_y[num_encoded - kBoundaryNum + d] = tail_d(1);
        poly_msg.coef_z[num_encoded - kBoundaryNum + d] = tail_d(2);
    }

    if (M > 1) {
        Eigen::MatrixXd positions = data->traj.getPositions();
        for (int i = 0; i < M - 1; ++i) {
            poly_msg.coef_x[kBoundaryNum + i] = positions(0, i + 1);
            poly_msg.coef_y[kBoundaryNum + i] = positions(1, i + 1);
            poly_msg.coef_z[kBoundaryNum + i] = positions(2, i + 1);
        }
    }
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; 
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.evaluate(traj_t, 0); 
      double reso = map->getResolution();
      
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) return false;
        if (ret == 1) 
        {
          height = forward_p(2);
          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);
          return true;
        }
      }
    }
    return false;
  }
} // namespace ego_planner
