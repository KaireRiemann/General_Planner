#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_tracking_ref_ = false;
    have_tracking_target_odom_ = false;
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
    nh.param("fsm/min_replan_interval", min_replan_interval_, 0.15);
    nh.param("fsm/corridor_fail_cooldown", corridor_fail_cooldown_, 0.25);
    nh.param("fsm/near_goal_replan_radius", near_goal_replan_radius_, 0.8);
    nh.param("fsm/corridor_check_margin", corridor_check_margin_, 0.05);
    nh.param("fsm/corridor_disable_fail_threshold", corridor_disable_fail_threshold_, 3);
    nh.param("fsm/corridor_disable_duration", corridor_disable_duration_, 1.0);
    nh.param("fsm/state2state_keep_current_traj", state2state_keep_current_traj_, true);
    nh.param("fsm/state2state_keep_lookahead", state2state_keep_lookahead_, 0.8);
    nh.param("fsm/state2state_min_rest_time", state2state_min_rest_time_, 0.8);
    nh.param("fsm/state2state_replan_target_shift_thresh", state2state_replan_target_shift_thresh_, 0.6);
    nh.param("fsm/state2state_space_model_preference", state2state_space_model_preference_, std::string("auto"));
    nh.param("fsm/use_tracking_task", use_tracking_task_, false);
    nh.param("fsm/tracking_reference_topic", tracking_reference_topic_, std::string("/tracking/reference"));
    nh.param("fsm/tracking_target_odom_topic", tracking_target_odom_topic_, std::string("/tracking/target_odom"));
    nh.param("fsm/tracking_reference_dt", tracking_reference_dt_, 0.2);
    nh.param("fsm/tracking_reference_timeout", tracking_reference_timeout_, 0.6);
    nh.param("fsm/tracking_prediction_horizon", tracking_prediction_horizon_, 4.0);
    nh.param("fsm/tracking_prediction_dt", tracking_prediction_dt_, 0.2);
    nh.param("fsm/tracking_prediction_max_speed", tracking_prediction_max_speed_, 2.0);
    nh.param("fsm/tracking_relay_goal", tracking_relay_goal_, true);
    nh.param("fsm/tracking_target_goal_topic", tracking_target_goal_topic_, std::string("/tracking/target_goal"));
    nh.param("optimization/tracking_distance_min", tracking_distance_min_, 1.5);
    nh.param("optimization/tracking_distance_max", tracking_distance_max_, 4.0);
    nh.param("optimization/tracking_height_tolerance", tracking_height_tolerance_, 0.4);
    nh.param("fsm/tracking_wait_distance_buffer", tracking_wait_distance_buffer_, 0.35);
    nh.param("fsm/tracking_wait_height_buffer", tracking_wait_height_buffer_, 0.20);
    nh.param("fsm/tracking_wait_target_vel_thresh", tracking_wait_target_vel_thresh_, 0.20);
    nh.param("fsm/tracking_wait_ego_vel_thresh", tracking_wait_ego_vel_thresh_, 0.15);
    nh.param("fsm/tracking_resume_target_vel_thresh", tracking_resume_target_vel_thresh_, 0.25);
    nh.param("fsm/tracking_resume_target_move_thresh", tracking_resume_target_move_thresh_, 0.35);
    nh.param("fsm/tracking_replan_target_shift_thresh", tracking_replan_target_shift_thresh_, 0.8);
    nh.param("fsm/tracking_replan_current_traj_lookahead", tracking_replan_current_traj_lookahead_, 0.8);
    nh.param("fsm/tracking_replan_min_rest_time", tracking_replan_min_rest_time_, 0.8);
    nh.param("fsm/tracking_replan_distance_buffer", tracking_replan_distance_buffer_, 0.55);
    nh.param("fsm/tracking_replan_height_buffer", tracking_replan_height_buffer_, 0.30);
    tracking_distance_min_ = std::max(0.0, tracking_distance_min_);
    tracking_distance_max_ = std::max(tracking_distance_min_ + 0.1, tracking_distance_max_);
    tracking_height_tolerance_ = std::max(0.0, tracking_height_tolerance_);
    state2state_keep_lookahead_ = std::max(0.2, state2state_keep_lookahead_);
    state2state_min_rest_time_ = std::max(0.1, state2state_min_rest_time_);
    state2state_replan_target_shift_thresh_ = std::max(0.05, state2state_replan_target_shift_thresh_);
    std::transform(state2state_space_model_preference_.begin(),
                   state2state_space_model_preference_.end(),
                   state2state_space_model_preference_.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::tolower(c)); });
    if (state2state_space_model_preference_ != "auto" &&
        state2state_space_model_preference_ != "plain" &&
        state2state_space_model_preference_ != "corridor" &&
        state2state_space_model_preference_ != "esdf")
    {
      ROS_WARN("Unknown fsm/state2state_space_model_preference=%s, fallback to auto.",
               state2state_space_model_preference_.c_str());
      state2state_space_model_preference_ = "auto";
    }

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    if (target_type_ == TARGET_TYPE::REFENCE_PATH)
    {
      use_tracking_task_ = true;
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    context_builder_.reset(new runtime::ContextBuilder());
    task_executor_.reset(new runtime::TaskExecutor(planner_manager_.get()));
    plan_monitor_.reset(new runtime::PlanMonitor());
    replan_trigger_.reset(new runtime::ReplanTrigger());

    have_trigger_ = use_tracking_task_ ? true : !flag_realworld_experiment_;
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

    if (use_tracking_task_)
    {
      tracking_ref_sub_ = nh.subscribe<nav_msgs::Path>(
          tracking_reference_topic_,
          1,
          &EGOReplanFSM::trackingReferenceCallback,
          this);
      tracking_target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
          tracking_target_odom_topic_,
          1,
          &EGOReplanFSM::trackingTargetOdomCallback,
          this);
      if (tracking_relay_goal_)
      {
        waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
        tracking_target_goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>(tracking_target_goal_topic_, 1);
      }
      ROS_INFO("Tracking task enabled. Waiting tracking reference on: %s", tracking_reference_topic_.c_str());
      ROS_INFO("Tracking target odom fallback enabled on: %s", tracking_target_odom_topic_.c_str());
      if (tracking_relay_goal_)
      {
        ROS_INFO("Tracking task goal relay enabled: /goal -> %s", tracking_target_goal_topic_.c_str());
      }
    }
    else if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
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

    if (use_tracking_task_)
    {
      refreshTrackingReference();
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
      if (use_tracking_task_)
      {
        if (!have_tracking_ref_ || !have_odom_) goto force_return;
        if (tracking_wait_for_motion_)
        {
          if (!tracking_target_moving_)
          {
            goto force_return;
          }
          tracking_wait_for_motion_ = false;
          ROS_INFO("Tracking target starts moving again, leave WAIT_TARGET.");
        }
        else if (trackingShouldEnterWaitTarget())
        {
          tracking_wait_for_motion_ = true;
          ROS_INFO_THROTTLE(1.0, "Tracking target is stationary and distance is satisfied, keep WAIT_TARGET.");
          goto force_return;
        }
      }
      else
      {
        if (!have_target_ || !have_trigger_) goto force_return;
      }
      changeFSMExecState(SEQUENTIAL_START, "FSM");
      break;
    }

    case SEQUENTIAL_START: 
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(1);
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
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
      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = std::min(info->duration, t_cur);
      const bool tracking_active = use_tracking_task_ && have_tracking_ref_;

      if (tracking_active && trackingShouldEnterWaitTarget())
      {
        tracking_wait_for_motion_ = true;
        changeFSMExecState(WAIT_TARGET, "TRACKING_SETTLED");
        break;
      }
      
      Eigen::Vector3d pos = info->traj.getPos(t_cur); 
      bool touch_the_goal = tracking_active ? false : ((local_target_pt_ - final_goal_).norm() < 1e-2);
      const bool arrived_goal =
          (!tracking_active) &&
          touch_the_goal &&
          (final_goal_ - odom_pos_).norm() < std::max(0.25, 0.5 * near_goal_replan_radius_) &&
          odom_vel_.norm() < 0.3;

      bool close_to_current_traj_end = (info->duration - t_cur) < emergency_time_; 

      if (mondifyInCollisionFinalGoal()) 
      {
        // pass
      }
      else if (!tracking_active &&
               (target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) 
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if (!tracking_active &&
               (arrived_goal || ((t_cur > info->duration - 1e-2) && touch_the_goal))) 
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }
      const double now = ros::Time::now().toSec();
      const bool replan_allowed =
          replan_trigger_ ? replan_trigger_->allowReplan(now, last_replan_time_, min_replan_interval_)
                          : ((last_replan_time_ < 0.0) || (now - last_replan_time_ > min_replan_interval_));

      const bool near_goal = tracking_active ? false :
          ((final_goal_ - pos).norm() < near_goal_replan_radius_ || touch_the_goal);

      if (tracking_active &&
          !near_goal &&
          (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) &&
          trackingCanKeepCurrentTraj(info, t_cur))
      {
        break;
      }

      if (replan_allowed &&
          !near_goal &&
          (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)))
      {
        if (tracking_active)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        else
        {
          std::string keep_reason = state2state_keep_current_traj_ ? "replan_forced" : "keep_disabled";
          const bool keep_current =
              state2state_keep_current_traj_ &&
              stateToStateCanKeepCurrentTraj(info, t_cur, &keep_reason);
          ROS_INFO_THROTTLE(0.8,
                            "[FSM] keep_current_state2state=%s, reason=%s",
                            keep_current ? "yes" : "no",
                            keep_reason.c_str());
          if (!keep_current)
          {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
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

    if (new_state == WAIT_TARGET || new_state == EMERGENCY_STOP)
    {
      resetPlannedTaskTargets();
    }

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
    bool trajectory_leaves_corridor = false;

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


      if (dangerous)
      {
        if (planFromLocalTraj(1)) 
        {
          ROS_INFO("Plan success when detect collision at future t=%f", t);
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
        }
        else
        {
          if (t - t_cur < emergency_time_) 
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

    if (trajectory_leaves_corridor)
    {
      const double now = ros::Time::now().toSec();
      const bool replan_allowed =
          (last_replan_time_ < 0.0) || (now - last_replan_time_ > min_replan_interval_);

      if (replan_allowed && exec_state_ == EXEC_TRAJ)
      {
        ROS_WARN("Trajectory leaves corridor, trigger replan.");
        changeFSMExecState(REPLAN_TRAJ, "CORRIDOR_CHECK");
      }
      return;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    resetPlannedTaskTargets();
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    polyTraj2ROSMsg(poly_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(poly_msg);

    return true;
  }

  bool EGOReplanFSM::currentTrajStillUsable(double lookahead_time) const
  {
    const LocalTrajData *info = &planner_manager_->traj_.local_traj;
    if (info->traj_id <= 0)
    {
      return false;
    }

    const double t_now = ros::Time::now().toSec();
    const double t_cur = t_now - info->start_time;
    if (t_cur >= info->duration - 0.05)
    {
      return false;
    }

    const double t_end = std::min(info->duration, t_cur + std::max(lookahead_time, 0.3));
    for (double t = t_cur; t <= t_end + 1.0e-6; t += 0.05)
    {
      const double sample_t = std::min(t, info->duration);
      const Eigen::Vector3d pt = info->traj.getPos(sample_t);
      if (planner_manager_->grid_map_->getInflateOccupancy(pt))
      {
        return false;
      }
    }

    return true;
  }

  bool EGOReplanFSM::stateToStateCanKeepCurrentTraj(const LocalTrajData *info,
                                                    double t_cur,
                                                    std::string *reason)
  {
    const auto set_reason = [&](const std::string &msg)
    {
      if (reason != nullptr)
      {
        *reason = msg;
      }
    };

    if (!state2state_keep_current_traj_)
    {
      set_reason("keep_disabled");
      return false;
    }
    if (info == nullptr)
    {
      set_reason("null_local_traj");
      return false;
    }
    if (plan_monitor_ && !plan_monitor_->hasValidLocalTraj(*info))
    {
      set_reason("invalid_local_traj");
      return false;
    }
    if (!have_planned_local_target_ || !have_planned_final_goal_)
    {
      set_reason("missing_planned_targets");
      return false;
    }

    const double remaining_t = std::max(0.0, info->duration - t_cur);
    if (remaining_t < state2state_min_rest_time_)
    {
      set_reason("remaining_time_short");
      return false;
    }

    const double lookahead = std::min(state2state_keep_lookahead_, remaining_t);
    if (!currentTrajStillUsable(lookahead))
    {
      set_reason("future_collision_or_invalid");
      return false;
    }

    const double local_target_shift =
        (local_target_pt_ - planned_local_target_pt_).norm();
    if (local_target_shift > state2state_replan_target_shift_thresh_)
    {
      set_reason("local_target_shift");
      return false;
    }

    const double final_goal_shift =
        (final_goal_ - planned_final_goal_).norm();
    if (final_goal_shift > state2state_replan_target_shift_thresh_)
    {
      set_reason("final_goal_shift");
      return false;
    }

    const bool near_goal =
        touch_goal_ ||
        (final_goal_ - odom_pos_).norm() < near_goal_replan_radius_;
    if (near_goal)
    {
      set_reason("near_goal");
      return true;
    }

    set_reason("current_traj_valid");
    return true;
  }

  bool EGOReplanFSM::shouldForcePlainReplan() const
  {
    if (!planner_manager_ || !planner_manager_->corridorModeEnabled())
    {
      return false;
    }

    const double now = ros::Time::now().toSec();

    if (corridor_disabled_until_ > now)
    {
      return true;
    }

    if (last_corridor_fail_time_ > 0.0 &&
        (now - last_corridor_fail_time_) < corridor_fail_cooldown_)
    {
      return true;
    }

    return false;
  }

  void EGOReplanFSM::markCorridorFailure(EGOPlannerManager::CorridorFailureType failure_type)
  {
    if (failure_type != EGOPlannerManager::FAIL_CORRIDOR_GENERATION &&
        failure_type != EGOPlannerManager::FAIL_CORRIDOR_INIT &&
        failure_type != EGOPlannerManager::FAIL_CORRIDOR_OPT)
    {
      return;
    }

    const double now = ros::Time::now().toSec();
    last_corridor_fail_time_ = now;
    corridor_fail_count_++;

    if (corridor_fail_count_ >= corridor_disable_fail_threshold_)
    {
      corridor_disabled_until_ = std::max(corridor_disabled_until_,
                                          now + corridor_disable_duration_);
      ROS_WARN("Corridor planning temporarily disabled until %.3f after %d consecutive failures.",
              corridor_disabled_until_, corridor_fail_count_);
    }
  }

  void EGOReplanFSM::resetCorridorFailureState(bool clear_disable)
  {
    corridor_fail_count_ = 0;
    last_corridor_fail_time_ = -1.0;
    if (clear_disable)
    {
      corridor_disabled_until_ = -1.0;
    }
  }

  void EGOReplanFSM::resetPlannedTaskTargets()
  {
    have_planned_local_target_ = false;
    have_planned_final_goal_ = false;
    planned_local_target_pt_.setZero();
    planned_final_goal_.setZero();
    have_planned_tracking_target_now_ = false;
    have_planned_tracking_ref_end_ = false;
    planned_tracking_target_pos_now_.setZero();
    planned_tracking_ref_end_.setZero();
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (use_tracking_task_)
    {
      refreshTrackingReference();
    }

    const bool tracking_active =
        use_tracking_task_ &&
        have_tracking_ref_ &&
        tracking_reference_.valid();

    if (tracking_active)
    {
      local_target_pt_ = tracking_target_pos_now_;
      local_target_vel_ = tracking_target_vel_now_;
      touch_goal_ = false;
    }
    else
    {
      planner_manager_->getLocalTarget(
          planning_horizen_, start_pt_, final_goal_,
          local_target_pt_, local_target_vel_,
          touch_goal_);
    }

    const bool force_plain = shouldForcePlainReplan();

    core::PlanningContext planning_context;
    if (context_builder_)
    {
      planning_context = context_builder_->build(
          planner_manager_->pp_.drone_id,
          planner_manager_->grid_map_ != nullptr,
          planner_manager_->grid_map_ != nullptr,
          planner_manager_->grid_map_ != nullptr && planner_manager_->grid_map_->esdfEnabled(),
          planner_manager_->getJpsAstar() != nullptr,
          planner_manager_->grid_map_,
          planner_manager_->getJpsAstar(),
          odom_pos_,
          odom_vel_,
          Eigen::Vector3d::Zero(),
          final_goal_,
          local_target_pt_,
          local_target_vel_,
          planner_manager_->pp_.max_vel_,
          planner_manager_->pp_.max_acc_,
          planner_manager_->pp_.polyTraj_piece_length,
          planner_manager_->getGuideMinClearance(),
          planner_manager_->getGuideSparseMinInner(),
          planner_manager_->getGuideSparseMaxInner(),
          planner_manager_->getGuideTurnAngleDeg(),
          planner_manager_->getSfcProgress(),
          planner_manager_->getSfcRange(),
          planner_manager_->getSfcCorridorMargin(),
          &planner_manager_->traj_.local_traj);
    }

    // FSM only manages runtime state transitions.
    // Task semantics come from TaskFactory, and the compiler owns problem construction.
    bool task_force_plain = force_plain;
    bool task_prefer_corridor = false;
    bool task_prefer_esdf = false;
    std::string resolved_state2state_pref = "plain";
    if (!tracking_active)
    {
      if (task_force_plain || state2state_space_model_preference_ == "plain")
      {
        task_force_plain = true;
        resolved_state2state_pref = "plain";
      }
      else if (state2state_space_model_preference_ == "corridor")
      {
        task_prefer_corridor = true;
        resolved_state2state_pref = "corridor";
      }
      else if (state2state_space_model_preference_ == "esdf")
      {
        task_prefer_esdf = true;
        resolved_state2state_pref = "esdf";
      }
      else
      {
        if (planner_manager_->corridorModeEnabled())
        {
          task_prefer_corridor = true;
          resolved_state2state_pref = "auto->corridor";
        }
        else if (planner_manager_->esdfModeEnabled())
        {
          task_prefer_esdf = true;
          resolved_state2state_pref = "auto->esdf";
        }
        else
        {
          task_force_plain = true;
          resolved_state2state_pref = "auto->plain";
        }
      }
      ROS_INFO("[FSM] state2state task preference=%s resolved=%s force_plain=%s",
               state2state_space_model_preference_.c_str(),
               resolved_state2state_pref.c_str(),
               task_force_plain ? "yes" : "no");
    }

    core::TaskDefinition task_definition = tracking_active
                                               ? tasks::TaskFactory::makeTrackingDefinition(
                                                     tracking_reference_,
                                                     start_pt_,
                                                     start_vel_,
                                                     start_acc_,
                                                     flag_use_poly_init,
                                                     flag_randomPolyTraj,
                                                     force_plain)
                                               : tasks::TaskFactory::makeStateToStateDefinition(
                                                     start_pt_,
                                                     start_vel_,
                                                     start_acc_,
                                                     local_target_pt_,
                                                     local_target_vel_,
                                                     touch_goal_,
                                                     flag_use_poly_init,
                                                     flag_randomPolyTraj,
                                                     task_force_plain,
                                                     task_prefer_corridor,
                                                     task_prefer_esdf);

    core::PlanningSolution planning_solution;
    const bool plan_success = task_executor_
                                  ? task_executor_->execute(planning_context, task_definition, planning_solution)
                                  : planner_manager_->solveTask(planning_context, task_definition, planning_solution);

    if (plan_success)
    {
      traj_utils::PolyTraj poly_msg;
      polyTraj2ROSMsg(poly_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(poly_msg);

      last_replan_time_ = ros::Time::now().toSec();

      if (!force_plain)
      {
        resetCorridorFailureState(true);
      }

      if (tracking_active)
      {
        planned_tracking_target_pos_now_ = tracking_target_pos_now_;
        have_planned_tracking_target_now_ = true;
        planned_tracking_ref_end_ = tracking_reference_.p_ref.back();
        have_planned_tracking_ref_end_ = true;
        have_planned_local_target_ = false;
        have_planned_final_goal_ = false;
      }
      else
      {
        planned_local_target_pt_ = local_target_pt_;
        planned_final_goal_ = final_goal_;
        have_planned_local_target_ = true;
        have_planned_final_goal_ = true;
        have_planned_tracking_target_now_ = false;
        have_planned_tracking_ref_end_ = false;
      }

      return true;
    }

    if (!force_plain)
    {
      markCorridorFailure(planner_manager_->getLastCorridorFailureType());
    }

    if (!planning_solution.message.empty())
    {
      ROS_WARN("[FSM] planning failed: %s", planning_solution.message.c_str());
    }

    return false;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times) 
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;

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
    resetPlannedTaskTargets();
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      final_goal_ = next_wp;
      corridor_fail_count_ = 0;


      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.evaluate(i * step_size_t, 0);
      }

      have_target_ = true;
      have_new_target_ = true;
     
      if ( have_odom_)
      {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
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
    if (use_tracking_task_)
    {
      return false;
    }

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

    if (use_tracking_task_ && tracking_relay_goal_)
    {
      if (tracking_target_goal_pub_)
      {
        tracking_target_goal_pub_.publish(*msg);
      }

      Eigen::Vector3d goal(msg->goal[0], msg->goal[1], msg->goal[2]);
      if (visualization_)
      {
        visualization_->displayGoalPoint(goal,
                                         Eigen::Vector4d(1.0, 0.8, 0.1, 1.0),
                                         0.24,
                                         3000 + planner_manager_->pp_.drone_id);
      }
      ROS_INFO("Tracking relay goal: [%.2f %.2f %.2f] -> %s",
               goal.x(), goal.y(), goal.z(),
               tracking_target_goal_topic_.c_str());
      return;
    }

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

  bool EGOReplanFSM::trackingDistanceSatisfied(const Eigen::Vector3d &ego_pos,
                                               const Eigen::Vector3d &target_pos,
                                               const double planar_buffer,
                                               const double height_buffer) const
  {
    const Eigen::Vector3d delta = target_pos - ego_pos;
    const double planar_dist = delta.head<2>().norm();
    const double dist_lo = std::max(0.0, tracking_distance_min_ - planar_buffer);
    const double dist_hi = std::max(dist_lo + 0.05, tracking_distance_max_ + planar_buffer);
    const double z_tol = std::max(0.0, tracking_height_tolerance_ + height_buffer);
    return planar_dist >= dist_lo &&
           planar_dist <= dist_hi &&
           std::abs(delta.z()) <= z_tol;
  }

  void EGOReplanFSM::trackingTargetOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (!msg)
    {
      return;
    }

    tracking_target_odom_pos_.x() = msg->pose.pose.position.x;
    tracking_target_odom_pos_.y() = msg->pose.pose.position.y;
    tracking_target_odom_pos_.z() = msg->pose.pose.position.z;
    tracking_target_odom_vel_.x() = msg->twist.twist.linear.x;
    tracking_target_odom_vel_.y() = msg->twist.twist.linear.y;
    tracking_target_odom_vel_.z() = msg->twist.twist.linear.z;
    tracking_target_pos_now_ = tracking_target_odom_pos_;
    tracking_target_vel_now_ = tracking_target_odom_vel_;
    have_tracking_target_odom_ = true;
    last_tracking_target_odom_recv_time_ = ros::Time::now().toSec();

    const double clamped_speed =
        std::min(tracking_target_odom_vel_.norm(), std::max(0.0, tracking_prediction_max_speed_));
    tracking_target_moving_ =
        clamped_speed > tracking_resume_target_vel_thresh_;
  }

  bool EGOReplanFSM::synthesizeTrackingReferenceFromOdom()
  {
    if (!have_tracking_target_odom_)
    {
      return false;
    }

    cost_functional::TrackingReference ref;
    const double horizon = std::max(0.2, tracking_prediction_horizon_);
    const double dt = std::max(0.05, tracking_prediction_dt_);
    const int sample_num =
        std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);

    Eigen::Vector3d pred_vel = tracking_target_odom_vel_;
    const double speed = pred_vel.norm();
    if (speed > std::max(0.1, tracking_prediction_max_speed_))
    {
      pred_vel *= tracking_prediction_max_speed_ / speed;
    }

    ref.t_ref.reserve(static_cast<std::size_t>(sample_num));
    ref.p_ref.reserve(static_cast<std::size_t>(sample_num));
    ref.v_ref.reserve(static_cast<std::size_t>(sample_num));

    for (int i = 0; i < sample_num; ++i)
    {
      const double t = std::min(horizon, static_cast<double>(i) * dt);
      ref.t_ref.push_back(t);
      ref.p_ref.push_back(tracking_target_odom_pos_ + pred_vel * t);
      ref.v_ref.push_back(pred_vel);
    }

    if (!ref.valid())
    {
      return false;
    }

    tracking_reference_ = ref;
    have_tracking_ref_ = true;
    have_target_ = true;
    have_new_target_ = true;
    have_trigger_ = true;
    tracking_target_pos_now_ = tracking_target_odom_pos_;
    tracking_target_vel_now_ = tracking_target_odom_vel_;
    final_goal_ = tracking_target_pos_now_;
    local_target_pt_ = tracking_target_pos_now_;
    local_target_vel_ = tracking_target_vel_now_;
    if (have_odom_)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }
    return true;
  }

  bool EGOReplanFSM::refreshTrackingReference()
  {
    if (!use_tracking_task_)
    {
      return false;
    }

    const double now = ros::Time::now().toSec();
    const bool ref_recent =
        have_tracking_ref_ &&
        tracking_reference_.valid() &&
        last_tracking_ref_recv_time_ > 0.0 &&
        (now - last_tracking_ref_recv_time_) <= std::max(0.1, tracking_reference_timeout_);

    if (ref_recent)
    {
      return true;
    }

    const bool odom_recent =
        have_tracking_target_odom_ &&
        last_tracking_target_odom_recv_time_ > 0.0 &&
        (now - last_tracking_target_odom_recv_time_) <= std::max(0.1, tracking_reference_timeout_);

    if (!odom_recent)
    {
      return have_tracking_ref_ && tracking_reference_.valid();
    }

    const bool ok = synthesizeTrackingReferenceFromOdom();
    if (ok)
    {
      ROS_INFO_THROTTLE(1.0,
                        "Tracking reference fallback: synthesize short-horizon reference from target odom.");
    }
    return ok;
  }

  bool EGOReplanFSM::trackingShouldEnterWaitTarget() const
  {
    if (!use_tracking_task_ || !have_tracking_ref_ || !tracking_reference_.valid())
    {
      return false;
    }

    const bool target_static = tracking_target_vel_now_.norm() <= tracking_wait_target_vel_thresh_;
    const bool drone_slow = odom_vel_.norm() <= tracking_wait_ego_vel_thresh_;
    const bool distance_ok =
        trackingDistanceSatisfied(odom_pos_,
                                  tracking_target_pos_now_,
                                  tracking_wait_distance_buffer_,
                                  tracking_wait_height_buffer_);
    return target_static && drone_slow && distance_ok;
  }

  bool EGOReplanFSM::trackingCanKeepCurrentTraj(const LocalTrajData *info, double t_cur)
  {
    refreshTrackingReference();
    if (!use_tracking_task_ || !have_tracking_ref_ || !tracking_reference_.valid() || info == nullptr)
    {
      return false;
    }
    if (plan_monitor_ && !plan_monitor_->hasValidLocalTraj(*info))
    {
      return false;
    }

    if (!currentTrajStillUsable(tracking_replan_current_traj_lookahead_))
    {
      return false;
    }

    if (!have_planned_tracking_target_now_)
    {
      return false;
    }

    const Eigen::Vector3d target_shift = tracking_target_pos_now_ - planned_tracking_target_pos_now_;
    const double target_shift_xy = target_shift.head<2>().norm();
    const double target_shift_z = std::abs(target_shift.z());
    const double remaining_t = std::max(0.0, info->duration - t_cur);
    if (target_shift_xy > tracking_replan_target_shift_thresh_ ||
        target_shift_z > (tracking_height_tolerance_ + tracking_replan_height_buffer_))
    {
      return false;
    }

    if (remaining_t < tracking_replan_min_rest_time_)
    {
      return false;
    }

    const double query_dt = std::max(0.0, tracking_replan_current_traj_lookahead_);
    const double sample_t = std::min(info->duration, t_cur + std::max(query_dt, 0.2));
    const Eigen::Vector3d traj_pos = info->traj.getPos(sample_t);

    Eigen::Vector3d ref_pos = tracking_target_pos_now_;
    Eigen::Vector3d ref_vel = tracking_target_vel_now_;
    if (!cost_functional::sampleTrackingReference(
            tracking_reference_,
            std::min(query_dt, tracking_reference_.t_ref.back()),
            ref_pos,
            ref_vel))
    {
      return false;
    }

    if (!trackingDistanceSatisfied(traj_pos,
                                   ref_pos,
                                   tracking_replan_distance_buffer_,
                                   tracking_replan_height_buffer_))
    {
      return false;
    }

    if (!planner_manager_->trackingSemanticHorizonValid(t_cur, tracking_replan_current_traj_lookahead_))
    {
      return false;
    }

    const Eigen::Vector3d lookahead_delta = ref_pos - traj_pos;
    ROS_INFO_THROTTLE(0.8,
                      "Tracking keep current traj: target_shift_xy=%.2f target_shift_z=%.2f lookahead_xy=%.2f lookahead_z=%.2f remaining_t=%.2f lookahead=%.2f",
                      target_shift_xy,
                      target_shift_z,
                      lookahead_delta.head<2>().norm(),
                      std::abs(lookahead_delta.z()),
                      remaining_t,
                      query_dt);

    planned_tracking_target_pos_now_ = tracking_target_pos_now_;
    have_planned_tracking_target_now_ = true;
    planned_tracking_ref_end_ = tracking_reference_.p_ref.back();
    have_planned_tracking_ref_end_ = true;
    return true;
  }

  void EGOReplanFSM::trackingReferenceCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!msg || msg->poses.size() < 2)
    {
      return;
    }

    cost_functional::TrackingReference ref;
    const int N = static_cast<int>(msg->poses.size());
    ref.t_ref.reserve(static_cast<std::size_t>(N));
    ref.p_ref.reserve(static_cast<std::size_t>(N));
    ref.v_ref.resize(static_cast<std::size_t>(N), Eigen::Vector3d::Zero());

    const double min_dt = std::max(0.02, tracking_reference_dt_);
    const double t0_stamp = msg->poses.front().header.stamp.toSec();
    double last_t = 0.0;

    for (int i = 0; i < N; ++i)
    {
      const auto &pose = msg->poses[static_cast<std::size_t>(i)].pose.position;
      ref.p_ref.emplace_back(pose.x, pose.y, pose.z);

      double ti = static_cast<double>(i) * min_dt;
      const double pose_stamp = msg->poses[static_cast<std::size_t>(i)].header.stamp.toSec();
      if (t0_stamp > 1.0e-6 && pose_stamp > 1.0e-6)
      {
        ti = std::max(0.0, pose_stamp - t0_stamp);
      }

      if (i > 0 && ti <= last_t)
      {
        ti = last_t + min_dt;
      }

      ref.t_ref.push_back(ti);
      last_t = ti;
    }

    for (int i = 0; i + 1 < N; ++i)
    {
      const double dt = std::max(1.0e-3, ref.t_ref[static_cast<std::size_t>(i + 1)] - ref.t_ref[static_cast<std::size_t>(i)]);
      ref.v_ref[static_cast<std::size_t>(i)] =
          (ref.p_ref[static_cast<std::size_t>(i + 1)] - ref.p_ref[static_cast<std::size_t>(i)]) / dt;
    }
    ref.v_ref.back() = ref.v_ref[static_cast<std::size_t>(N - 2)];

    if (!ref.valid())
    {
      ROS_WARN("Received invalid tracking reference.");
      return;
    }

    tracking_reference_ = ref;
    have_tracking_ref_ = true;
    last_tracking_ref_recv_time_ = ros::Time::now().toSec();
    have_target_ = true;
    have_new_target_ = true;
    have_trigger_ = true;

    tracking_target_pos_now_ = ref.p_ref.front();
    if (!ref.v_ref.empty())
    {
      tracking_target_vel_now_ = ref.v_ref.front();
    }
    else
    {
      const double dt = std::max(1.0e-3, ref.t_ref[1] - ref.t_ref[0]);
      tracking_target_vel_now_ = (ref.p_ref[1] - ref.p_ref[0]) / dt;
    }

    double max_ref_speed = tracking_target_vel_now_.norm();
    for (const auto &v : ref.v_ref)
    {
      max_ref_speed = std::max(max_ref_speed, v.norm());
    }
    const double horizon_displacement = (ref.p_ref.back() - ref.p_ref.front()).norm();
    tracking_target_moving_ =
        (max_ref_speed > tracking_resume_target_vel_thresh_) ||
        (horizon_displacement > tracking_resume_target_move_thresh_);

    final_goal_ = tracking_target_pos_now_;
    local_target_pt_ = tracking_target_pos_now_;
    local_target_vel_ = tracking_target_vel_now_;

    if (have_odom_)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }

    if (visualization_)
    {
      visualization_->displayGoalPoint(final_goal_,
                                       Eigen::Vector4d(1.0, 0.5, 0.0, 1.0),
                                       0.22,
                                       2000 + planner_manager_->pp_.drone_id);
    }
  }

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
