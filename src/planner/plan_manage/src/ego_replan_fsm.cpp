#include <plan_manage/ego_replan_fsm.h>

#include <chrono>
#include <thread>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_tracking_ref_ = false;
    have_tracking_target_odom_ = false;
    have_perching_target_odom_ = false;
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
    nh.param("fsm/safety_replan_min_interval", safety_replan_min_interval_, 0.20);
    nh.param("fsm/safety_replan_emergency_bypass_time", safety_replan_emergency_bypass_time_, 0.10);
    nh.param("fsm/esdf_runtime_collision_hysteresis", esdf_runtime_collision_hysteresis_, 0.03);
    nh.param("fsm/esdf_runtime_unsafe_consecutive_samples", esdf_runtime_unsafe_consecutive_samples_, 2);
    nh.param("fsm/corridor_fail_cooldown", corridor_fail_cooldown_, 0.25);
    nh.param("fsm/near_goal_replan_radius", near_goal_replan_radius_, 0.8);
    nh.param("fsm/corridor_check_margin", corridor_check_margin_, 0.05);
    nh.param("fsm/corridor_disable_fail_threshold", corridor_disable_fail_threshold_, 3);
    nh.param("fsm/corridor_disable_duration", corridor_disable_duration_, 1.0);
    nh.param("fsm/corridor_plain_fallback_enabled", corridor_plain_fallback_enabled_, false);
    nh.param("fsm/state2state_keep_current_traj", state2state_keep_current_traj_, true);
    nh.param("fsm/state2state_successor_enable", state2state_successor_enable_, true);
    nh.param("fsm/state2state_keep_lookahead", state2state_keep_lookahead_, 0.8);
    nh.param("fsm/state2state_min_rest_time", state2state_min_rest_time_, 0.8);
    nh.param("fsm/state2state_replan_target_shift_thresh", state2state_replan_target_shift_thresh_, 0.6);
    nh.param("fsm/state2state_successor_lead_time", state2state_successor_lead_time_, 0.8);
    nh.param("fsm/state2state_successor_min_progress", state2state_successor_min_progress_, 0.55);
    nh.param("fsm/state2state_successor_target_shift_thresh", state2state_successor_target_shift_thresh_, 0.35);
    nh.param("fsm/state2state_successor_horizon_ratio", state2state_successor_horizon_ratio_, 0.65);
    nh.param("fsm/state2state_successor_near_goal_hold_radius", state2state_successor_near_goal_hold_radius_, 0.5);
    nh.param("fsm/state2state_space_model_preference", state2state_space_model_preference_, std::string("auto"));
    nh.param("fsm/use_tracking_task", use_tracking_task_, false);
    nh.param("fsm/use_perching_task", use_perching_task_, false);
    nh.param("fsm/tracking_reference_topic", tracking_reference_topic_, std::string("/tracking/reference"));
    nh.param("fsm/tracking_target_odom_topic", tracking_target_odom_topic_, std::string("/tracking/target_odom"));
    nh.param("fsm/tracking_reference_dt", tracking_reference_dt_, 0.2);
    nh.param("fsm/tracking_reference_timeout", tracking_reference_timeout_, 0.6);
    nh.param("fsm/tracking_prediction_horizon", tracking_prediction_horizon_, 4.0);
    nh.param("fsm/tracking_prediction_dt", tracking_prediction_dt_, 0.2);
    nh.param("fsm/tracking_prediction_max_speed", tracking_prediction_max_speed_, 2.0);
    nh.param("fsm/tracking_anchor_side_angle_deg", tracking_anchor_side_angle_deg_, 20.0);
    nh.param("fsm/tracking_relative_offset_x", tracking_relative_offset_.x(), 0.0);
    nh.param("fsm/tracking_relative_offset_y", tracking_relative_offset_.y(), 0.0);
    nh.param("fsm/tracking_relative_offset_z", tracking_relative_offset_.z(), 0.0);
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
    nh.param("fsm/perching_target_odom_topic", perching_target_odom_topic_, std::string("/perching/target_odom"));
    nh.param("fsm/perching_trigger_topic", perching_trigger_topic_, std::string("/land_triger"));
    nh.param("fsm/perching_auto_start", perching_auto_start_, false);
    nh.param("fsm/perching_robot_l", perching_robot_l_, 0.02);
    nh.param("fsm/perching_v_plus", perching_v_plus_, 0.3);
    nh.param("fsm/perching_min_prediction_time", perching_min_prediction_time_, 1.0);
    nh.param("fsm/perching_max_prediction_time", perching_max_prediction_time_, 5.0);
    nh.param("fsm/perching_terminal_thrust", perching_terminal_thrust_, 9.81);
    nh.param("fsm/perching_terminal_thrust_range", perching_terminal_thrust_range_, 0.0);
    nh.param("fsm/perching_use_dynamics_terminal_accel", perching_use_dynamics_terminal_accel_, false);
    nh.param("fsm/perching_override_target_orientation", perching_override_target_orientation_, false);
    nh.param("fsm/perching_replan_if_unsafe", perching_replan_if_unsafe_, true);
    nh.param("fsm/perching_arrive_pos_thresh", perching_arrive_pos_thresh_, 0.45);
    nh.param("fsm/perching_arrive_vel_thresh", perching_arrive_vel_thresh_, 0.85);
    nh.param("fsm/perching_min_execute_time", perching_min_execute_time_, 0.30);
    nh.param("fsm/perching_axis_x", perching_axis_.x(), 0.0);
    nh.param("fsm/perching_axis_y", perching_axis_.y(), 1.0);
    nh.param("fsm/perching_axis_z", perching_axis_.z(), 0.0);
    nh.param("fsm/perching_theta", perching_theta_, -1.5708);
    tracking_distance_min_ = std::max(0.0, tracking_distance_min_);
    tracking_distance_max_ = std::max(tracking_distance_min_ + 0.1, tracking_distance_max_);
    tracking_height_tolerance_ = std::max(0.0, tracking_height_tolerance_);
    tracking_anchor_side_angle_deg_ = std::max(0.0, tracking_anchor_side_angle_deg_);
    state2state_keep_lookahead_ = std::max(0.2, state2state_keep_lookahead_);
    state2state_min_rest_time_ = std::max(0.1, state2state_min_rest_time_);
    state2state_replan_target_shift_thresh_ = std::max(0.05, state2state_replan_target_shift_thresh_);
    state2state_successor_lead_time_ = std::max(0.15, state2state_successor_lead_time_);
    state2state_successor_min_progress_ = std::max(0.05, std::min(0.95, state2state_successor_min_progress_));
    state2state_successor_target_shift_thresh_ = std::max(0.05, state2state_successor_target_shift_thresh_);
    state2state_successor_horizon_ratio_ = std::max(0.1, std::min(0.95, state2state_successor_horizon_ratio_));
    state2state_successor_near_goal_hold_radius_ = std::max(0.1, state2state_successor_near_goal_hold_radius_);
    safety_replan_min_interval_ = std::max(0.05, safety_replan_min_interval_);
    safety_replan_emergency_bypass_time_ = std::max(0.02, safety_replan_emergency_bypass_time_);
    esdf_runtime_collision_hysteresis_ = std::max(0.0, esdf_runtime_collision_hysteresis_);
    esdf_runtime_unsafe_consecutive_samples_ = std::max(1, esdf_runtime_unsafe_consecutive_samples_);
    perching_arrive_pos_thresh_ = std::max(0.03, perching_arrive_pos_thresh_);
    perching_arrive_vel_thresh_ = std::max(0.02, perching_arrive_vel_thresh_);
    perching_min_execute_time_ = std::max(0.0, perching_min_execute_time_);
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
    if (target_type_ == TARGET_TYPE::PERCHING_TARGET)
    {
      use_perching_task_ = true;
      use_tracking_task_ = false;
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    context_builder_.reset(new runtime::ContextBuilder());
    planner_engine_.reset(new engine::PlannerEngine(planner_manager_.get()));
    task_executor_.reset(new runtime::TaskExecutor(planner_engine_.get()));
    local_target_selector_.reset(new runtime::LocalTargetSelector());
    plan_monitor_.reset(new runtime::PlanMonitor());
    replan_trigger_.reset(new runtime::ReplanTrigger());
    tracking_reference_provider_.reset(new runtime::TrackingReferenceProvider());
    tracking_reference_provider_->configure(tracking_prediction_horizon_,
                                            tracking_prediction_dt_,
                                            tracking_prediction_max_speed_);
    tracking_anchor_selector_.reset(new runtime::TrackingAnchorSelector());
    tracking_anchor_selector_->configure(tracking_distance_min_,
                                         tracking_distance_max_,
                                         tracking_anchor_side_angle_deg_,
                                         tracking_relative_offset_);
    perching_target_provider_.reset(new runtime::PerchingTargetProvider());
    perching_target_provider_->configure(
        perching_robot_l_,
        perching_v_plus_,
        perching_min_prediction_time_,
        perching_max_prediction_time_,
        perching_terminal_thrust_,
        perching_terminal_thrust_range_,
        perching_use_dynamics_terminal_accel_,
        runtime::PerchingTargetProvider::quaternionFromAxisAngle(perching_axis_, perching_theta_),
        perching_override_target_orientation_);

    have_trigger_ = use_tracking_task_
                        ? true
                        : (use_perching_task_ ? perching_auto_start_ : !flag_realworld_experiment_);
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
      ROS_INFO("Tracking task enabled. reference_topic=%s target_odom_topic=%s relative_offset=[%.2f %.2f %.2f]",
               tracking_reference_topic_.c_str(),
               tracking_target_odom_topic_.c_str(),
               tracking_relative_offset_.x(),
               tracking_relative_offset_.y(),
               tracking_relative_offset_.z());
      if (tracking_relay_goal_)
      {
        ROS_INFO("Tracking task goal relay enabled: /goal -> %s", tracking_target_goal_topic_.c_str());
      }
    }
    else if (use_perching_task_)
    {
      perching_target_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
          perching_target_odom_topic_,
          1,
          &EGOReplanFSM::perchingTargetOdomCallback,
          this);
      perching_trigger_sub_ = nh.subscribe<std_msgs::Empty>(
          perching_trigger_topic_,
          1,
          &EGOReplanFSM::perchingTriggerCallback,
          this);
      ROS_INFO("Perching task enabled. target_odom_topic=%s trigger_topic=%s auto_start=%s robot_l=%.3f v_plus=%.3f dyn_terminal_acc=%s thrust_nom=%.2f thrust_range=%.2f",
               perching_target_odom_topic_.c_str(),
               perching_trigger_topic_.c_str(),
               perching_auto_start_ ? "yes" : "no",
               perching_robot_l_,
               perching_v_plus_,
               perching_use_dynamics_terminal_accel_ ? "yes" : "no",
               perching_terminal_thrust_,
               perching_terminal_thrust_range_);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
      else if (use_perching_task_)
      {
        if (!have_odom_ || !have_perching_target_odom_) goto force_return;
        if (!perching_auto_start_ && !perching_triggered_) goto force_return;
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
      const bool perching_runtime = use_perching_task_;

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
          !perching_runtime &&
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

      if (perching_runtime)
      {
        const Eigen::Vector3d planned_contact =
            have_planned_local_target_ ? planned_local_target_pt_ : local_target_pt_;
        const double dist_to_contact = (planned_contact - odom_pos_).norm();
        const double remaining_time = std::max(0.0, info->duration - t_cur);
        const bool min_exec_satisfied = t_cur >= perching_min_execute_time_;
        const bool near_contact =
            min_exec_satisfied &&
            dist_to_contact <= perching_arrive_pos_thresh_ &&
            odom_vel_.norm() <= perching_arrive_vel_thresh_;
        const bool terminal_capture_window =
            min_exec_satisfied &&
            remaining_time <= 0.8 &&
            dist_to_contact <= std::max(0.55, 1.5 * perching_arrive_pos_thresh_);
        const bool trajectory_finished =
            min_exec_satisfied &&
            remaining_time <= 0.03;

        ROS_INFO_THROTTLE(0.8,
                          "[FSM] perching_exec dist_to_contact=%.2f odom_vel=%.2f remaining_t=%.2f trigger_pending=%s round_active=%s",
                          dist_to_contact,
                          odom_vel_.norm(),
                          remaining_time,
                          perching_triggered_ ? "yes" : "no",
                          perching_round_active_ ? "yes" : "no");

        if (near_contact || terminal_capture_window || trajectory_finished)
        {
          perching_round_active_ = false;
          have_trigger_ = perching_triggered_;
          ROS_INFO("[FSM] perching round finished (%s). Waiting for next /land_triger or /perching/reset.",
                   near_contact ? "contact_reached" : (terminal_capture_window ? "terminal_capture" : "traj_finished"));
          changeFSMExecState(WAIT_TARGET, "PERCHING_DONE");
          break;
        }

        if (perching_replan_if_unsafe_ &&
            replan_allowed &&
            remaining_time > 0.8 &&
            dist_to_contact > std::max(0.55, 1.5 * perching_arrive_pos_thresh_) &&
            !currentTrajStillUsable(std::min(state2state_keep_lookahead_, remaining_time)))
        {
          ROS_WARN("[FSM] perching trajectory became unsafe, replan current landing round.");
          changeFSMExecState(REPLAN_TRAJ, "PERCHING_SAFETY");
        }
        break;
      }

      if (tracking_active &&
          !near_goal &&
          (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) &&
          trackingCanKeepCurrentTraj(info, t_cur))
      {
        break;
      }

      if (!tracking_active)
      {
        StateToStateDecisionDebug decision_debug;
        const StateToStateRuntimeDecision decision =
            evaluateStateToStateDecision(info, t_cur, &decision_debug);
        ROS_INFO_THROTTLE(0.8,
                          "[FSM] state2state_decision=%d keep_current_state2state=%s successor_preparation=%s immediate_replan=%s arrived_hold=%s emergency_stop=%s remaining_t=%.2f progress=%.2f preview_valid=%s preview_target_shift=%.2f reason=%s",
                          static_cast<int>(decision),
                          decision_debug.keep_current_state2state ? "yes" : "no",
                          decision_debug.successor_preparation ? "yes" : "no",
                          decision_debug.immediate_replan ? "yes" : "no",
                          decision_debug.arrived_hold ? "yes" : "no",
                          decision_debug.emergency_stop ? "yes" : "no",
                          decision_debug.remaining_time,
                          decision_debug.progress_ratio,
                          decision_debug.preview_valid ? "yes" : "no",
                          decision_debug.preview_target_shift,
                          decision_debug.reason.c_str());

        if (decision == StateToStateRuntimeDecision::EMERGENCY_STOP)
        {
          have_pending_state2state_target_selection_ = false;
          pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
          changeFSMExecState(EMERGENCY_STOP, "FSM");
        }
        else if (decision == StateToStateRuntimeDecision::ARRIVED_AND_HOLD)
        {
          have_pending_state2state_target_selection_ = false;
          pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
          have_target_ = false;
          have_trigger_ = false;
          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wpt_id_ = 0;
            planNextWaypoint(wps_[wpt_id_]);
          }
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else if (replan_allowed &&
                 (decision == StateToStateRuntimeDecision::REPLAN_IMMEDIATE ||
                  decision == StateToStateRuntimeDecision::PREPARE_SUCCESSOR))
        {
          have_pending_state2state_target_selection_ =
              (decision == StateToStateRuntimeDecision::PREPARE_SUCCESSOR) &&
              decision_debug.preview_valid &&
              decision_debug.preview_selection.valid;
          if (have_pending_state2state_target_selection_)
          {
            pending_state2state_target_selection_ = decision_debug.preview_selection;
          }
          else
          {
            pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
          }
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        else
        {
          have_pending_state2state_target_selection_ = false;
          pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
        }
        break;
      }

      if (replan_allowed &&
          !near_goal &&
          (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)))
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

    const bool task_has_target =
        use_tracking_task_ ? have_tracking_ref_
                           : (use_perching_task_ ? have_perching_target_odom_ : have_target_);
    if (!have_odom_ || !task_has_target || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
      std::cout << ". Waiting for ";
    if (!have_odom_) std::cout << "odom,";
    if (!task_has_target) std::cout << "target,";
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

    if (exec_state_ != EXEC_TRAJ || info->traj_id <= 0) return;
    if (!map) return;

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
    int marginal_esdf_unsafe_streak = 0;
    const bool use_esdf_runtime =
        planner_manager_ &&
        planner_manager_->esdfModeEnabled() &&
        map->esdfEnabled();
    const double esdf_runtime_tol = runtimeCollisionTol(map);

    const auto allowByMinInterval =
        [&](double now) -> bool
    {
      if (replan_trigger_)
      {
        if (!replan_trigger_->allowReplan(now, last_replan_time_, min_replan_interval_))
        {
          return false;
        }
      }
      else if ((last_replan_time_ >= 0.0) &&
               (now - last_replan_time_ <= min_replan_interval_))
      {
        return false;
      }

      if ((last_safety_replan_attempt_time_ >= 0.0) &&
          (now - last_safety_replan_attempt_time_ <= safety_replan_min_interval_))
      {
        return false;
      }
      return true;
    };

    for (double t = t_cur; t < t_end; t += t_step)
    {
      Eigen::Vector3d pt =info->traj.getPos(t);
      bool dangerous = false;

      double sdf_at_pt = 0.0;
      if (runtimePointUnsafe(map, pt, &sdf_at_pt))
      {
        bool treat_as_dangerous = true;
        if (use_esdf_runtime &&
            std::isfinite(sdf_at_pt) &&
            sdf_at_pt >= esdf_runtime_tol - esdf_runtime_collision_hysteresis_)
        {
          marginal_esdf_unsafe_streak++;
          treat_as_dangerous =
              marginal_esdf_unsafe_streak >= esdf_runtime_unsafe_consecutive_samples_;
        }
        else
        {
          marginal_esdf_unsafe_streak = esdf_runtime_unsafe_consecutive_samples_;
        }

        if (treat_as_dangerous)
        {
          dangerous = true;
          ROS_WARN_THROTTLE(0.8,
                            "[SAFETY] runtime unsafe sample: t=%.3f sdf=%.3f tol=%.3f streak=%d",
                            t,
                            sdf_at_pt,
                            esdf_runtime_tol,
                            marginal_esdf_unsafe_streak);
        }
      }
      else
      {
        marginal_esdf_unsafe_streak = 0;
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
        const double now = ros::Time::now().toSec();
        const double time_to_collision = std::max(0.0, t - t_cur);
        const bool emergency_bypass =
            time_to_collision <= safety_replan_emergency_bypass_time_;
        const bool replan_allowed = allowByMinInterval(now);

        if (!emergency_bypass && !replan_allowed)
        {
          ROS_WARN_THROTTLE(0.8,
                            "[SAFETY] collision predicted in %.3fs but replan is throttled (min_interval=%.2f, safety_interval=%.2f).",
                            time_to_collision,
                            min_replan_interval_,
                            safety_replan_min_interval_);
          return;
        }

        last_safety_replan_attempt_time_ = now;
        if (planFromLocalTraj(1)) 
        {
          ROS_INFO("Plan success when detect collision at future t=%f", t);
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
        }
        else
        {
          if (time_to_collision < emergency_time_) 
          {
            ROS_WARN("Emergency stop! Crash in %f seconds", time_to_collision);
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
      const bool replan_allowed = allowByMinInterval(now);

      if (replan_allowed && exec_state_ == EXEC_TRAJ)
      {
        last_safety_replan_attempt_time_ = now;
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
    if (!planner_manager_->grid_map_)
    {
      return true;
    }

    const double t_end = std::min(info->duration, t_cur + std::max(lookahead_time, 0.3));
    const bool use_esdf_runtime =
        planner_manager_ &&
        planner_manager_->esdfModeEnabled() &&
        planner_manager_->grid_map_->esdfEnabled();
    const double esdf_runtime_tol = runtimeCollisionTol(planner_manager_->grid_map_);
    int marginal_esdf_unsafe_streak = 0;
    for (double t = t_cur; t <= t_end + 1.0e-6; t += 0.05)
    {
      const double sample_t = std::min(t, info->duration);
      const Eigen::Vector3d pt = info->traj.getPos(sample_t);
      double sdf_at_pt = 0.0;
      if (runtimePointUnsafe(planner_manager_->grid_map_, pt, &sdf_at_pt))
      {
        bool treat_as_unsafe = true;
        if (use_esdf_runtime &&
            std::isfinite(sdf_at_pt) &&
            sdf_at_pt >= esdf_runtime_tol - esdf_runtime_collision_hysteresis_)
        {
          marginal_esdf_unsafe_streak++;
          treat_as_unsafe =
              marginal_esdf_unsafe_streak >= esdf_runtime_unsafe_consecutive_samples_;
        }
        else
        {
          marginal_esdf_unsafe_streak = esdf_runtime_unsafe_consecutive_samples_;
        }

        if (treat_as_unsafe)
        {
          ROS_DEBUG_THROTTLE(0.8,
                             "[FSM] currentTrajStillUsable unsafe: t=%.3f sdf=%.3f tol=%.3f streak=%d",
                             sample_t,
                             sdf_at_pt,
                             esdf_runtime_tol,
                             marginal_esdf_unsafe_streak);
          return false;
        }
      }
      else
      {
        marginal_esdf_unsafe_streak = 0;
      }
    }

    return true;
  }

  double EGOReplanFSM::runtimeCollisionTol(const GridMap::Ptr &map) const
  {
    if (!map)
    {
      return 0.0;
    }
    return -std::max(0.10, 0.5 * map->getResolution());
  }

  bool EGOReplanFSM::runtimePointUnsafe(const GridMap::Ptr &map,
                                        const Eigen::Vector3d &pt,
                                        double *signed_distance) const
  {
    if (!map)
    {
      if (signed_distance != nullptr)
      {
        *signed_distance = 0.0;
      }
      return false;
    }

    const bool use_esdf_runtime =
        planner_manager_ &&
        planner_manager_->esdfModeEnabled() &&
        map->esdfEnabled();

    if (use_esdf_runtime)
    {
      const double sdf = map->getDistance(pt);
      if (signed_distance != nullptr)
      {
        *signed_distance = sdf;
      }
      if (!std::isfinite(sdf))
      {
        return true;
      }
      return sdf < runtimeCollisionTol(map);
    }

    if (signed_distance != nullptr)
    {
      *signed_distance = map->getDistance(pt);
    }
    return map->getInflateOccupancy(pt) != 0;
  }

  bool EGOReplanFSM::stateToStateCanKeepCurrentTraj(const LocalTrajData *info,
                                                    double t_cur,
                                                    std::string *reason)
  {
    const bool enable_keep_current =
        have_active_state2state_runtime_policy_
            ? active_state2state_runtime_policy_.enable_keep_current
            : state2state_keep_current_traj_;
    const double keep_lookahead =
        have_active_state2state_runtime_policy_
            ? std::max(0.2, active_state2state_runtime_policy_.keep_lookahead)
            : state2state_keep_lookahead_;
    const auto set_reason = [&](const std::string &msg)
    {
      if (reason != nullptr)
      {
        *reason = msg;
      }
    };

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
    const double remaining_t = std::max(0.0, info->duration - t_cur);
    if (remaining_t <= 1.0e-3)
    {
      set_reason("segment_exhausted");
      return false;
    }

    if (!enable_keep_current)
    {
      set_reason("keep_current_disabled");
      return false;
    }

    const double lookahead = std::min(keep_lookahead, remaining_t);
    if (!currentTrajStillUsable(lookahead))
    {
      set_reason("future_collision_or_invalid");
      return false;
    }

    set_reason("safe_execute");
    return true;
  }

  bool EGOReplanFSM::shouldImmediateReplanStateToState(const LocalTrajData *info,
                                                       double t_cur,
                                                       std::string *reason)
  {
    return !stateToStateCanKeepCurrentTraj(info, t_cur, reason);
  }

  bool EGOReplanFSM::shouldPrepareStateToStateSuccessor(const LocalTrajData *info,
                                                        double t_cur,
                                                        std::string *reason,
                                                        double *preview_target_shift,
                                                        bool *preview_valid,
                                                        runtime::LocalTargetSelection *preview_selection)
  {
    const bool enable_keep_current =
        have_active_state2state_runtime_policy_
            ? active_state2state_runtime_policy_.enable_keep_current
            : state2state_keep_current_traj_;
    const bool enable_successor_planning =
        have_active_state2state_runtime_policy_
            ? active_state2state_runtime_policy_.enable_successor_planning
            : state2state_successor_enable_;
    const double min_rest_time =
        have_active_state2state_runtime_policy_
            ? std::max(0.1, active_state2state_runtime_policy_.min_rest_time)
            : state2state_min_rest_time_;
    const double successor_lead_time =
        have_active_state2state_runtime_policy_
            ? std::max(0.15, active_state2state_runtime_policy_.successor_lead_time)
            : state2state_successor_lead_time_;
    const double successor_min_progress =
        have_active_state2state_runtime_policy_
            ? std::max(0.05, std::min(0.95, active_state2state_runtime_policy_.successor_min_progress))
            : state2state_successor_min_progress_;
    const double successor_horizon_ratio =
        have_active_state2state_runtime_policy_
            ? std::max(0.1, std::min(0.95, active_state2state_runtime_policy_.successor_horizon_ratio))
            : state2state_successor_horizon_ratio_;
    const double successor_target_shift_thresh =
        have_active_state2state_runtime_policy_
            ? std::max(0.05, active_state2state_runtime_policy_.successor_target_shift_thresh)
            : state2state_successor_target_shift_thresh_;
    const double successor_near_goal_hold_radius =
        have_active_state2state_runtime_policy_
            ? std::max(0.1, active_state2state_runtime_policy_.successor_near_goal_hold_radius)
            : state2state_successor_near_goal_hold_radius_;
    const auto set_reason = [&](const std::string &msg)
    {
      if (reason != nullptr)
      {
        *reason = msg;
      }
    };

    if (preview_target_shift != nullptr)
    {
      *preview_target_shift = 0.0;
    }
    if (preview_valid != nullptr)
    {
      *preview_valid = false;
    }
    if (preview_selection != nullptr)
    {
      *preview_selection = runtime::LocalTargetSelection{};
    }

    if (!enable_successor_planning)
    {
      set_reason("successor_disabled");
      return false;
    }
    if (info == nullptr || info->traj_id <= 0)
    {
      set_reason("no_active_traj");
      return false;
    }

    const double remaining_t = std::max(0.0, info->duration - t_cur);
    const double progress = info->duration > 1.0e-6 ? t_cur / info->duration : 1.0;
    if (touch_goal_ || (final_goal_ - odom_pos_).norm() < successor_near_goal_hold_radius)
    {
      set_reason("near_goal_hold");
      return false;
    }
    if (!enable_keep_current)
    {
      set_reason("keep_policy_disabled");
      return true;
    }
    if (!have_planned_local_target_ || !have_planned_final_goal_)
    {
      set_reason("missing_planned_targets");
      return true;
    }
    if (remaining_t < min_rest_time)
    {
      set_reason("remaining_time_short");
      return true;
    }

    const double final_goal_shift =
        (final_goal_ - planned_final_goal_).norm();
    if (final_goal_shift > state2state_replan_target_shift_thresh_)
    {
      set_reason("final_goal_shift");
      return true;
    }

    runtime::LocalTargetSelection preview;
    const double preview_start_t =
        std::min(info->duration,
                 t_cur + std::max(0.05, 0.35 * successor_lead_time));
    const Eigen::Vector3d preview_start_pt = info->traj.evaluate(preview_start_t, 0);
    if (!local_target_selector_ ||
        !local_target_selector_->peekLocalTarget(planner_manager_->traj_,
                                                planning_horizen_,
                                                planner_manager_->pp_.max_vel_,
                                                preview_start_pt,
                                                final_goal_,
                                                preview))
    {
      set_reason("preview_failed");
      return remaining_t <= successor_lead_time;
    }

    if (preview_valid != nullptr)
    {
      *preview_valid = true;
    }
    if (preview_selection != nullptr)
    {
      *preview_selection = preview;
    }

    const double target_shift = (preview.local_target_pos - planned_local_target_pt_).norm();
    if (preview_target_shift != nullptr)
    {
      *preview_target_shift = target_shift;
    }
    const bool have_prev_target_time = planned_local_target_glb_t_ > 0.0;
    const double target_time_advance =
        have_prev_target_time ? (preview.next_glb_t_of_lc_tgt - planned_local_target_glb_t_) : 0.0;
    const double target_time_shift_thresh =
        std::max(0.20, 0.5 * successor_lead_time);
    const bool successor_horizon_window =
        remaining_t <= planning_horizen_ * successor_horizon_ratio;

    if (remaining_t <= successor_lead_time)
    {
      set_reason("lead_time_window");
      return true;
    }
    if (progress >= successor_min_progress &&
        successor_horizon_window &&
        (target_shift >= successor_target_shift_thresh ||
         target_time_advance >= target_time_shift_thresh))
    {
      set_reason(target_shift >= successor_target_shift_thresh
                     ? "preview_target_shift_advanced"
                     : "preview_target_time_advanced");
      return true;
    }

    if (progress >= std::min(0.9, successor_min_progress + 0.25) &&
        successor_horizon_window)
    {
      set_reason("late_progress_window");
      return true;
    }

    set_reason("successor_not_needed");
    return false;
  }

  EGOReplanFSM::StateToStateRuntimeDecision
  EGOReplanFSM::evaluateStateToStateDecision(const LocalTrajData *info,
                                             double t_cur,
                                             StateToStateDecisionDebug *debug)
  {
    StateToStateDecisionDebug local_debug;
    if (info != nullptr)
    {
      local_debug.remaining_time = std::max(0.0, info->duration - t_cur);
      local_debug.progress_ratio =
          info->duration > 1.0e-6 ? std::min(1.0, std::max(0.0, t_cur / info->duration)) : 1.0;
    }

    if (info == nullptr || !plan_monitor_ || !plan_monitor_->hasValidLocalTraj(*info))
    {
      local_debug.decision = StateToStateRuntimeDecision::REPLAN_IMMEDIATE;
      local_debug.immediate_replan = true;
      local_debug.reason = "invalid_local_traj";
      if (debug != nullptr)
      {
        *debug = local_debug;
      }
      return local_debug.decision;
    }

    if (mandatory_stop_)
    {
      local_debug.decision = StateToStateRuntimeDecision::EMERGENCY_STOP;
      local_debug.emergency_stop = true;
      local_debug.reason = "mandatory_stop";
      if (debug != nullptr)
      {
        *debug = local_debug;
      }
      return local_debug.decision;
    }

    const bool touch_the_goal = (local_target_pt_ - final_goal_).norm() < 1e-2;
    const bool arrived_goal =
        touch_the_goal &&
        (final_goal_ - odom_pos_).norm() < std::max(0.25, 0.5 * near_goal_replan_radius_) &&
        odom_vel_.norm() < 0.3;
    if (arrived_goal || ((t_cur > info->duration - 1e-2) && touch_the_goal))
    {
      local_debug.decision = StateToStateRuntimeDecision::ARRIVED_AND_HOLD;
      local_debug.arrived_hold = true;
      local_debug.reason = "arrived_goal";
      if (debug != nullptr)
      {
        *debug = local_debug;
      }
      return local_debug.decision;
    }

    std::string immediate_reason;
    if (shouldImmediateReplanStateToState(info, t_cur, &immediate_reason))
    {
      local_debug.decision = StateToStateRuntimeDecision::REPLAN_IMMEDIATE;
      local_debug.immediate_replan = true;
      local_debug.reason = immediate_reason.empty() ? "current_traj_unsafe" : immediate_reason;
      if (debug != nullptr)
      {
        *debug = local_debug;
      }
      return local_debug.decision;
    }

    local_debug.keep_current_state2state = true;

    std::string successor_reason;
    if (shouldPrepareStateToStateSuccessor(info,
                                           t_cur,
                                           &successor_reason,
                                           &local_debug.preview_target_shift,
                                           &local_debug.preview_valid,
                                           &local_debug.preview_selection))
    {
      local_debug.decision = StateToStateRuntimeDecision::PREPARE_SUCCESSOR;
      local_debug.successor_preparation = true;
      local_debug.reason = successor_reason;
      if (debug != nullptr)
      {
        *debug = local_debug;
      }
      return local_debug.decision;
    }

    local_debug.decision = StateToStateRuntimeDecision::KEEP_EXECUTING;
    local_debug.reason = "keep_current_segment";
    if (debug != nullptr)
    {
      *debug = local_debug;
    }
    return local_debug.decision;
  }

  bool EGOReplanFSM::shouldForcePlainReplan() const
  {
    if (!planner_manager_ || !planner_manager_->corridorModeEnabled())
    {
      return false;
    }
    if (!corridor_plain_fallback_enabled_)
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
    have_pending_state2state_target_selection_ = false;
    have_active_state2state_runtime_policy_ = false;
    active_state2state_runtime_policy_ = core::RuntimePolicy{};
    pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
    last_safety_replan_attempt_time_ = -1.0;
    planned_local_target_pt_.setZero();
    planned_local_target_glb_t_ = -1.0;
    planned_final_goal_.setZero();
    have_planned_tracking_target_now_ = false;
    have_planned_tracking_ref_end_ = false;
    perching_round_active_ = false;
    planned_tracking_target_pos_now_.setZero();
    planned_tracking_ref_end_.setZero();
  }

  bool EGOReplanFSM::callCurrentTaskPlan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    if (use_tracking_task_)
    {
      refreshTrackingReference();
    }

    const bool tracking_active =
        use_tracking_task_ &&
        have_tracking_ref_ &&
        tracking_reference_.valid();
    const bool perching_active =
        use_perching_task_ &&
        have_perching_target_odom_ &&
        perching_target_provider_ &&
        perching_target_provider_->hasTarget();
    cost_functional::TrackingReference planning_tracking_reference;
    Eigen::Vector3d tracking_anchor = local_target_pt_;
    Eigen::Vector3d tracking_anchor_vel = local_target_vel_;
    runtime::PerchingTerminalState perching_terminal;
    runtime::LocalTargetSelection target_selection;

    if (tracking_active)
    {
      if (tracking_reference_provider_ &&
          !tracking_reference_provider_->normalize(tracking_reference_, planning_tracking_reference))
      {
        ROS_WARN("[FSM] tracking reference provider normalize failed, fallback to raw reference.");
        planning_tracking_reference = tracking_reference_;
      }
      else if (!tracking_reference_provider_)
      {
        planning_tracking_reference = tracking_reference_;
      }

      bool anchor_ok = false;
      if (tracking_anchor_selector_)
      {
        anchor_ok = tracking_anchor_selector_->buildAnchorReference(planning_tracking_reference,
                                                                    start_pt_,
                                                                    start_vel_,
                                                                    planning_tracking_reference,
                                                                    tracking_anchor,
                                                                    tracking_anchor_vel);
      }
      if (!anchor_ok)
      {
        Eigen::Vector3d p_term = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_term = Eigen::Vector3d::Zero();
        if (!cost_functional::sampleTrackingTerminalReference(planning_tracking_reference, p_term, v_term))
        {
          ROS_WARN("[FSM] tracking anchor selector failed and terminal fallback unavailable.");
          return false;
        }
        tracking_anchor = p_term;
        tracking_anchor_vel = v_term;
      }

      local_target_pt_ = tracking_anchor;
      local_target_vel_ = tracking_anchor_vel;
      touch_goal_ = false;
    }
    else if (perching_active)
    {
      if (!perching_target_provider_->buildTerminalState(start_pt_,
                                                         planner_manager_->pp_.max_vel_,
                                                         perching_terminal))
      {
        ROS_WARN("[FSM] perching planning aborted: unable to build terminal state from plate odom.");
        return false;
      }

      local_target_pt_ = perching_terminal.terminal_position;
      local_target_vel_ = perching_terminal.terminal_velocity;
      final_goal_ = local_target_pt_;
      touch_goal_ = true;

      ROS_INFO("[FSM] perching terminal prediction_t=%.2f contact=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] normal=[%.2f %.2f %.2f]",
               perching_terminal.prediction_time,
               local_target_pt_.x(),
               local_target_pt_.y(),
               local_target_pt_.z(),
               local_target_vel_.x(),
               local_target_vel_.y(),
               local_target_vel_.z(),
               perching_terminal.terminal_acceleration.x(),
               perching_terminal.terminal_acceleration.y(),
               perching_terminal.terminal_acceleration.z(),
               perching_terminal.landing_normal.x(),
               perching_terminal.landing_normal.y(),
               perching_terminal.landing_normal.z());
    }
    else
    {
      const bool use_cached_preview =
          have_pending_state2state_target_selection_ &&
          pending_state2state_target_selection_.valid;

      if (use_cached_preview)
      {
        target_selection = pending_state2state_target_selection_;
      }
      else if (local_target_selector_)
      {
        local_target_selector_->peekLocalTarget(planner_manager_->traj_,
                                               planning_horizen_,
                                               planner_manager_->pp_.max_vel_,
                                               start_pt_,
                                               final_goal_,
                                               target_selection);
      }

      if (target_selection.valid)
      {
        local_target_pt_ = target_selection.local_target_pos;
        local_target_vel_ = target_selection.local_target_vel;
        touch_goal_ = target_selection.touch_goal;
      }
      else
      {
        local_target_pt_ = final_goal_;
        local_target_vel_ = Eigen::Vector3d::Zero();
        touch_goal_ = true;
      }

      ROS_INFO_THROTTLE(0.8,
                        "[FSM] state2state_plan_target preview_used=%s target=[%.2f %.2f %.2f] touch_goal=%s next_glb_t=%.2f",
                        use_cached_preview ? "yes" : "no",
                        local_target_pt_.x(),
                        local_target_pt_.y(),
                        local_target_pt_.z(),
                        touch_goal_ ? "yes" : "no",
                        target_selection.valid ? target_selection.next_glb_t_of_lc_tgt : -1.0);
    }

    if (tracking_active && !planning_tracking_reference.valid())
    {
      ROS_WARN("[FSM] tracking planning aborted: invalid planning tracking reference.");
      return false;
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
    if (!tracking_active && target_selection.valid)
    {
      planning_context.has_local_target_progress_preview = true;
      planning_context.preview_glb_t_of_lc_tgt = target_selection.next_glb_t_of_lc_tgt;
      planning_context.preview_last_glb_t_of_lc_tgt = target_selection.previous_glb_t_of_lc_tgt;
    }

    // FSM only manages runtime state transitions.
    // Task semantics come from TaskFactory, and the compiler owns problem construction.
    bool task_force_plain = force_plain;
    bool task_prefer_corridor = false;
    bool task_prefer_esdf = false;
    std::string resolved_space_pref = "plain";
    if (task_force_plain || state2state_space_model_preference_ == "plain")
    {
      task_force_plain = true;
      resolved_space_pref = "plain";
    }
    else if (state2state_space_model_preference_ == "corridor")
    {
      task_prefer_corridor = true;
      resolved_space_pref = "corridor";
    }
    else if (state2state_space_model_preference_ == "esdf")
    {
      task_prefer_esdf = true;
      resolved_space_pref = "esdf";
    }
    else
    {
      if (planner_manager_->corridorModeEnabled())
      {
        task_prefer_corridor = true;
        resolved_space_pref = "auto->corridor";
      }
      else if (planner_manager_->esdfModeEnabled())
      {
        task_prefer_esdf = true;
        resolved_space_pref = "auto->esdf";
      }
      else
      {
        task_force_plain = true;
        resolved_space_pref = "auto->plain";
      }
    }

    if (!tracking_active && !perching_active)
    {
      ROS_INFO("[FSM] state2state task preference=%s resolved=%s force_plain=%s",
               state2state_space_model_preference_.c_str(),
               resolved_space_pref.c_str(),
               task_force_plain ? "yes" : "no");
    }
    else if (perching_active)
    {
      ROS_INFO("[FSM] perching task preference=%s resolved=%s force_plain=%s",
               state2state_space_model_preference_.c_str(),
               resolved_space_pref.c_str(),
               task_force_plain ? "yes" : "no");
    }
    else
    {
      ROS_INFO("[FSM] tracking task preference=%s resolved=%s force_plain=%s",
               state2state_space_model_preference_.c_str(),
               resolved_space_pref.c_str(),
               task_force_plain ? "yes" : "no");
    }

    core::TaskDefinition task_definition;
    if (tracking_active)
    {
      task_definition = tasks::TaskFactory::makeTrackingDefinition(
          planning_tracking_reference,
          start_pt_,
          start_vel_,
          start_acc_,
          flag_use_poly_init,
          flag_randomPolyTraj,
          task_force_plain,
          task_prefer_corridor,
          task_prefer_esdf);
    }
    else if (perching_active)
    {
      task_definition = tasks::TaskFactory::makePerchingDefinition(
          start_pt_,
          start_vel_,
          start_acc_,
          perching_terminal.terminal_position,
          perching_terminal.terminal_velocity,
          perching_terminal.terminal_acceleration,
          perching_terminal.plate_position,
          perching_terminal.plate_velocity,
          perching_terminal.landing_tangent_x,
          perching_terminal.landing_tangent_y,
          perching_terminal.landing_normal,
          perching_robot_l_,
          perching_v_plus_,
          perching_terminal.terminal_thrust_nominal,
          perching_terminal.terminal_thrust_range,
          perching_terminal.use_dynamics_terminal_accel,
          task_force_plain,
          task_prefer_corridor,
          task_prefer_esdf);
      task_definition.runtime_policy.flag_poly_init = flag_use_poly_init;
      task_definition.runtime_policy.flag_random_poly_traj = flag_randomPolyTraj;
    }
    else
    {
      task_definition = tasks::TaskFactory::makeStateToStateDefinition(
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
    }

    if (!tracking_active && !perching_active)
    {
      task_definition.runtime_policy.enable_keep_current = state2state_keep_current_traj_;
      task_definition.runtime_policy.enable_successor_planning = state2state_successor_enable_;
      task_definition.runtime_policy.keep_lookahead = state2state_keep_lookahead_;
      task_definition.runtime_policy.min_rest_time = state2state_min_rest_time_;
      task_definition.runtime_policy.successor_lead_time = state2state_successor_lead_time_;
      task_definition.runtime_policy.successor_min_progress = state2state_successor_min_progress_;
      task_definition.runtime_policy.successor_horizon_ratio = state2state_successor_horizon_ratio_;
      task_definition.runtime_policy.successor_target_shift_thresh = state2state_successor_target_shift_thresh_;
      task_definition.runtime_policy.successor_near_goal_hold_radius = state2state_successor_near_goal_hold_radius_;
    }

    core::PlanningSolution planning_solution;
    const bool plan_success = task_executor_
                                  ? task_executor_->execute(planning_context, task_definition, planning_solution)
                                  : false;

    if (plan_success)
    {
      if (planning_solution.has_init_artifacts)
      {
        ROS_INFO("[FSM] solved task type=%s active_mode=%d init_source=%s guide_pts=%zu corridor_polys=%zu",
                 tracking_active ? "tracking" : (perching_active ? "perching" : "state_to_state"),
                 static_cast<int>(planning_solution.active_space_model),
                 planning_solution.init_source.c_str(),
                 planning_solution.guide_path.size(),
                 planning_solution.corridor_hpolys.size());
      }

      traj_utils::PolyTraj poly_msg;
      polyTraj2ROSMsg(poly_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(poly_msg);

      last_replan_time_ = ros::Time::now().toSec();

      if (!task_force_plain)
      {
        resetCorridorFailureState(true);
      }

      if (tracking_active)
      {
        planned_tracking_target_pos_now_ = tracking_target_pos_now_;
        have_planned_tracking_target_now_ = true;
        Eigen::Vector3d terminal_pos = tracking_anchor;
        Eigen::Vector3d terminal_vel = tracking_anchor_vel;
        if (!cost_functional::sampleTrackingTerminalReference(planning_tracking_reference,
                                                              terminal_pos,
                                                              terminal_vel))
        {
          terminal_pos = tracking_anchor;
        }
        planned_tracking_ref_end_ = terminal_pos;
        have_planned_tracking_ref_end_ = true;
        have_planned_local_target_ = false;
        have_planned_final_goal_ = false;
      }
      else if (perching_active)
      {
        have_pending_state2state_target_selection_ = false;
        pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
        planned_local_target_pt_ =
            planner_manager_->traj_.local_traj.traj.evaluate(
                planner_manager_->traj_.local_traj.duration, 0);
        planned_local_target_glb_t_ = -1.0;
        planned_final_goal_ = final_goal_;
        have_active_state2state_runtime_policy_ = false;
        have_planned_local_target_ = true;
        have_planned_final_goal_ = true;
        have_planned_tracking_target_now_ = false;
        have_planned_tracking_ref_end_ = false;
        perching_round_active_ = true;
        perching_triggered_ = false;
        have_trigger_ = false;
        ROS_INFO("[FSM] perching trigger consumed; current landing round is executing. Publish /land_triger or /perching/reset for the next round after landing.");
      }
      else
      {
        if (target_selection.valid && local_target_selector_)
        {
          local_target_selector_->commitLocalTarget(planner_manager_->traj_, target_selection);
        }
        have_pending_state2state_target_selection_ = false;
        pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
        planned_local_target_pt_ = local_target_pt_;
        planned_local_target_glb_t_ =
            target_selection.valid
                ? target_selection.next_glb_t_of_lc_tgt
                : planner_manager_->traj_.global_traj.glb_t_of_lc_tgt;
        planned_final_goal_ = final_goal_;
        active_state2state_runtime_policy_ = task_definition.runtime_policy;
        have_active_state2state_runtime_policy_ = true;
        have_planned_local_target_ = true;
        have_planned_final_goal_ = true;
        have_planned_tracking_target_now_ = false;
        have_planned_tracking_ref_end_ = false;
      }

      return true;
    }

    if (!tracking_active && !perching_active)
    {
      have_pending_state2state_target_selection_ = false;
      pending_state2state_target_selection_ = runtime::LocalTargetSelection{};
    }

    if (!task_force_plain)
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

    bool flag_random_poly_init = timesOfConsecutiveStateCalls().first > 1;

    for (int i = 0; i < trial_times; i++)
    {
      if (callCurrentTaskPlan(true, flag_random_poly_init))
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

    bool success = callCurrentTaskPlan(false, false);

    if (!success)
    {
      success = callCurrentTaskPlan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callCurrentTaskPlan(true, true);
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
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    if (use_tracking_task_ || use_perching_task_)
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
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

  void EGOReplanFSM::perchingTriggerCallback(const std_msgs::EmptyConstPtr &msg)
  {
    (void)msg;
    perching_triggered_ = true;
    have_trigger_ = true;
    ROS_INFO("Received perching trigger%s.",
             perching_round_active_ ? " and queued it for the next round" : "");
  }

  void EGOReplanFSM::perchingTargetOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (!msg || !perching_target_provider_)
    {
      return;
    }
    perching_target_provider_->updateFromOdometry(*msg);
    have_perching_target_odom_ = true;
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
    if (tracking_reference_provider_)
    {
      if (!tracking_reference_provider_->buildFromTargetOdom(tracking_target_odom_pos_,
                                                             tracking_target_odom_vel_,
                                                             ref))
      {
        return false;
      }
    }
    else
    {
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
    if (tracking_reference_provider_)
    {
      if (!tracking_reference_provider_->buildFromPath(*msg, tracking_reference_dt_, ref))
      {
        ROS_WARN("Received invalid tracking reference path (provider rejected).");
        return;
      }
    }
    else
    {
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
