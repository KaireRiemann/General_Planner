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
#include <runtime/tracking_reference_provider.hpp>
#include <runtime/tracking_anchor_selector.hpp>
#include <runtime/perching_target_provider.hpp>

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
      REFENCE_PATH = 3,
      PERCHING_TARGET = 4
    };
    enum class StateToStateRuntimeDecision
    {
      KEEP_EXECUTING = 0,
      PREPARE_SUCCESSOR,
      REPLAN_IMMEDIATE,
      ARRIVED_AND_HOLD,
      EMERGENCY_STOP
    };
    struct StateToStateDecisionDebug
    {
      StateToStateRuntimeDecision decision{StateToStateRuntimeDecision::KEEP_EXECUTING};
      bool keep_current_state2state{false};
      bool successor_preparation{false};
      bool immediate_replan{false};
      bool arrived_hold{false};
      bool emergency_stop{false};
      bool preview_valid{false};
      double remaining_time{0.0};
      double progress_ratio{0.0};
      double preview_target_shift{0.0};
      runtime::LocalTargetSelection preview_selection;
      std::string reason;
    };

    /* planning utils */
    // planner_manager is only the ROS/resource/module host.
    // PlannerEngine owns TaskDefinition -> PlanningProblem -> solve orchestration,
    // and FSM stays responsible for runtime transitions plus successor-planning decisions.
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;
    std::unique_ptr<runtime::ContextBuilder> context_builder_;
    std::unique_ptr<engine::PlannerEngine> planner_engine_;
    std::unique_ptr<runtime::TaskExecutor> task_executor_;
    std::unique_ptr<runtime::LocalTargetSelector> local_target_selector_;
    std::unique_ptr<runtime::PlanMonitor> plan_monitor_;
    std::unique_ptr<runtime::ReplanTrigger> replan_trigger_;
    std::unique_ptr<runtime::TrackingReferenceProvider> tracking_reference_provider_;
    std::unique_ptr<runtime::TrackingAnchorSelector> tracking_anchor_selector_;
    std::unique_ptr<runtime::PerchingTargetProvider> perching_target_provider_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    double min_replan_interval_{0.15};
    double safety_replan_min_interval_{0.20};
    double safety_replan_emergency_bypass_time_{0.10};
    double esdf_runtime_collision_hysteresis_{0.03};
    int esdf_runtime_unsafe_consecutive_samples_{2};
    double corridor_fail_cooldown_{0.25};
    double near_goal_replan_radius_{0.8};
    double corridor_check_margin_{0.05};
    int corridor_disable_fail_threshold_{3};
    double corridor_disable_duration_{1.0};
    bool corridor_plain_fallback_enabled_{false};
    bool state2state_keep_current_traj_{true};
    bool state2state_successor_enable_{true};
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
    double tracking_anchor_side_angle_deg_{20.0};
    Eigen::Vector3d tracking_relative_offset_{Eigen::Vector3d::Zero()};
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
    bool use_perching_task_{false};
    bool have_perching_target_odom_{false};
    bool perching_auto_start_{false};
    bool perching_triggered_{false};
    std::string perching_target_odom_topic_{"/perching/target_odom"};
    std::string perching_trigger_topic_{"/land_triger"};
    double perching_robot_l_{0.02};
    double perching_v_plus_{0.3};
    double perching_min_prediction_time_{1.0};
    double perching_max_prediction_time_{5.0};
    double perching_terminal_thrust_{9.81};
    bool perching_use_dynamics_terminal_accel_{false};
    bool perching_override_target_orientation_{false};
    bool perching_replan_if_unsafe_{true};
    double perching_arrive_pos_thresh_{0.45};
    double perching_arrive_vel_thresh_{0.85};
    double perching_min_execute_time_{0.30};
    Eigen::Vector3d perching_axis_{Eigen::Vector3d::UnitY()};
    double perching_theta_{-1.5708};
    bool perching_round_active_{false};
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
    runtime::LocalTargetSelection pending_state2state_target_selection_;
    bool have_pending_state2state_target_selection_{false};
    core::RuntimePolicy active_state2state_runtime_policy_;
    bool have_active_state2state_runtime_policy_{false};

    double last_replan_time_{-1.0};
    double last_safety_replan_attempt_time_{-1.0};
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
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, tracking_ref_sub_, tracking_target_odom_sub_, perching_target_odom_sub_, perching_trigger_sub_;
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
    double runtimeCollisionTol(const GridMap::Ptr &map) const;
    bool runtimePointUnsafe(const GridMap::Ptr &map,
                            const Eigen::Vector3d &pt,
                            double *signed_distance = nullptr) const;
    bool stateToStateCanKeepCurrentTraj(const LocalTrajData *info,
                                        double t_cur,
                                        std::string *reason = nullptr);
    bool shouldImmediateReplanStateToState(const LocalTrajData *info,
                                           double t_cur,
                                           std::string *reason = nullptr);
    bool shouldPrepareStateToStateSuccessor(const LocalTrajData *info,
                                            double t_cur,
                                            std::string *reason = nullptr,
                                            double *preview_target_shift = nullptr,
                                            bool *preview_valid = nullptr,
                                            runtime::LocalTargetSelection *preview_selection = nullptr);
    StateToStateRuntimeDecision evaluateStateToStateDecision(const LocalTrajData *info,
                                                             double t_cur,
                                                             StateToStateDecisionDebug *debug = nullptr);
    bool callCurrentTaskPlan(bool flag_use_poly_init, bool flag_randomPolyTraj);
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
    void perchingTriggerCallback(const std_msgs::EmptyConstPtr &msg);
    void perchingTargetOdomCallback(const nav_msgs::OdometryConstPtr &msg);
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
