#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <memory>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <CostFunctionalManager/TrackingTypes.hpp>
#include <core/planning_context.hpp>
#include <core/planning_solution.hpp>
#include <tasks/task_factory.hpp>
#include <engine/planner_engine.hpp>
#include <runtime/context_builder.hpp>
#include <runtime/local_target_selector.hpp>
#include <runtime/task_executor.hpp>
#include <runtime/plan_monitor.hpp>
#include <runtime/replan_trigger.hpp>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::to_string;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };
    enum class StateToStateRuntimeDecision
    {
      KEEP_EXECUTING = 0,
      PREPARE_SUCCESSOR,
      REPLAN_IMMEDIATE,
      ARRIVED_AND_HOLD,
      EMERGENCY_STOP
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;
    std::unique_ptr<runtime::ContextBuilder> context_builder_;
    std::unique_ptr<engine::PlannerEngine> planner_engine_;
    std::unique_ptr<runtime::TaskExecutor> task_executor_;
    std::unique_ptr<runtime::LocalTargetSelector> local_target_selector_;
    std::unique_ptr<runtime::PlanMonitor> plan_monitor_;
    std::unique_ptr<runtime::ReplanTrigger> replan_trigger_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    double min_replan_interval_{0.15};
    double corridor_fail_cooldown_{0.25};
    double near_goal_replan_radius_{0.8};
    double corridor_check_margin_{0.05};
    int corridor_disable_fail_threshold_{3};
    double corridor_disable_duration_{1.0};
    bool state2state_keep_current_traj_{true};
    double state2state_keep_lookahead_{0.8};
    double state2state_min_rest_time_{0.8};
    double state2state_replan_target_shift_thresh_{0.6};
    double state2state_successor_lead_time_{0.8};
    double state2state_successor_min_progress_{0.55};
    double state2state_successor_target_shift_thresh_{0.35};
    double state2state_successor_horizon_ratio_{0.65};
    double state2state_successor_near_goal_hold_radius_{0.5};
    std::string state2state_space_model_preference_{"auto"};
    bool use_tracking_task_{false};
    std::string tracking_reference_topic_{"/tracking/reference"};
    std::string tracking_target_odom_topic_{"/tracking/target_odom"};
    double tracking_reference_dt_{0.2};
    double tracking_reference_timeout_{0.6};
    double tracking_prediction_horizon_{4.0};
    double tracking_prediction_dt_{0.2};
    double tracking_prediction_max_speed_{2.0};
    bool tracking_relay_goal_{true};
    std::string tracking_target_goal_topic_{"/tracking/target_goal"};
    double tracking_distance_min_{1.5};
    double tracking_distance_max_{4.0};
    double tracking_height_tolerance_{0.4};
    double tracking_wait_distance_buffer_{0.35};
    double tracking_wait_height_buffer_{0.20};
    double tracking_wait_target_vel_thresh_{0.20};
    double tracking_wait_ego_vel_thresh_{0.15};
    double tracking_resume_target_vel_thresh_{0.25};
    double tracking_resume_target_move_thresh_{0.35};
    double tracking_replan_target_shift_thresh_{0.8};
    double tracking_replan_current_traj_lookahead_{0.8};
    double tracking_replan_min_rest_time_{0.8};
    double tracking_replan_distance_buffer_{0.55};
    double tracking_replan_height_buffer_{0.30};
    bool tracking_wait_for_motion_{false};
    bool tracking_target_moving_{true};
    bool have_tracking_target_odom_{false};
    bool have_planned_local_target_{false};
    bool have_planned_final_goal_{false};
    bool have_planned_tracking_target_now_{false};
    bool have_planned_tracking_ref_end_{false};
    double last_tracking_ref_recv_time_{-1.0};
    double last_tracking_target_odom_recv_time_{-1.0};
    Eigen::Vector3d tracking_target_pos_now_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tracking_target_vel_now_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tracking_target_odom_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tracking_target_odom_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d planned_local_target_pt_{Eigen::Vector3d::Zero()};
    double planned_local_target_glb_t_{-1.0};
    Eigen::Vector3d planned_final_goal_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d planned_tracking_target_pos_now_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d planned_tracking_ref_end_{Eigen::Vector3d::Zero()};

    double last_replan_time_{-1.0};
    double last_corridor_fail_time_{-1.0};
    int corridor_fail_count_{0};
    double corridor_disabled_until_{-1.0};
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    bool have_tracking_ref_{false};
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                         // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    std::vector<Eigen::Vector3d> wps_;
    cost_functional::TrackingReference tracking_reference_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, tracking_ref_sub_, tracking_target_odom_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_, tracking_target_goal_pub_;

    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool currentTrajStillUsable(double lookahead_time) const;
    bool stateToStateCanKeepCurrentTraj(const LocalTrajData *info,
                                        double t_cur,
                                        std::string *reason = nullptr);
    bool shouldPrepareStateToStateSuccessor(const LocalTrajData *info,
                                            double t_cur,
                                            std::string *reason = nullptr);
    StateToStateRuntimeDecision evaluateStateToStateDecision(const LocalTrajData *info,
                                                             double t_cur,
                                                             std::string *reason = nullptr);
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /*corridor failure manager*/
    bool shouldForcePlainReplan() const;
    void markCorridorFailure(EGOPlannerManager::CorridorFailureType failure_type);
    void resetCorridorFailureState(bool clear_disable = true);
    void resetPlannedTaskTargets();

    /* global trajectory */
    void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp);
    bool mondifyInCollisionFinalGoal();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void trackingReferenceCallback(const nav_msgs::PathConstPtr &msg);
    void trackingTargetOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    bool synthesizeTrackingReferenceFromOdom();
    bool refreshTrackingReference();
    bool trackingDistanceSatisfied(const Eigen::Vector3d &ego_pos,
                                   const Eigen::Vector3d &target_pos,
                                   double planar_buffer,
                                   double height_buffer) const;
    bool trackingShouldEnterWaitTarget() const;
    bool trackingCanKeepCurrentTraj(const LocalTrajData *info, double t_cur);
    void RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
  };

} // namespace ego_planner

#endif
