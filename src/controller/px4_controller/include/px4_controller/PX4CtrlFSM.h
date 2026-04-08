#ifndef PX4_CONTROLLER_FSM_H
#define PX4_CONTROLLER_FSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include <px4_controller/input.h>
#include <px4_controller/controller.h>

struct AutoTakeoffLand_t
{
  bool landed{true};
  ros::Time toggle_takeoff_land_time;
  std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
  Eigen::Vector4d start_pose;

  static constexpr double MOTORS_SPEEDUP_TIME = 3.0;
  static constexpr double DELAY_TRIGGER_TIME = 2.0;
};

class PX4CtrlFSM
{
public:
  Parameter_t& param;

  RC_Data_t rc_data;
  State_Data_t state_data;
  ExtendedState_Data_t extended_state_data;
  Odom_Data_t odom_data;
  Imu_Data_t imu_data;
  Command_Data_t cmd_data;
  Battery_Data_t bat_data;
  Takeoff_Land_Data_t takeoff_land_data;

  LinearControl& controller;

  ros::Publisher traj_start_trigger_pub;
  ros::Publisher ctrl_FCU_pub;
  ros::Publisher debug_pub;
  ros::ServiceClient set_FCU_mode_srv;
  ros::ServiceClient arming_client_srv;
  ros::ServiceClient reboot_FCU_srv;

  quadrotor_msgs::Px4ctrlDebug debug_msg;

  Eigen::Vector4d hover_pose;
  ros::Time last_set_hover_pose_time;

  enum State_t
  {
    MANUAL_CTRL = 1,
    AUTO_HOVER,
    CMD_CTRL,
    AUTO_TAKEOFF,
    AUTO_LAND
  };

  PX4CtrlFSM(Parameter_t& param, LinearControl& controller);
  void process();
  bool rc_is_received(const ros::Time& now_time);
  bool cmd_is_received(const ros::Time& now_time);
  bool odom_is_received(const ros::Time& now_time);
  bool imu_is_received(const ros::Time& now_time);
  bool bat_is_received(const ros::Time& now_time);
  bool recv_new_odom();
  State_t get_state() { return state; }
  bool get_landed() { return takeoff_land.landed; }

private:
  State_t state;
  AutoTakeoffLand_t takeoff_land;

  Desired_State_t get_hover_des();
  Desired_State_t get_cmd_des();
  void motors_idling(const Imu_Data_t& imu, Controller_Output_t& u);
  void land_detector(State_t state, const Desired_State_t& des, const Odom_Data_t& odom);
  void set_start_pose_for_takeoff_land(const Odom_Data_t& odom);
  Desired_State_t get_rotor_speed_up_des(const ros::Time now);
  Desired_State_t get_takeoff_land_des(double speed);
  void set_hov_with_odom();
  void set_hov_with_rc();

  bool toggle_offboard_mode(bool on_off);
  bool toggle_arm_disarm(bool arm);
  void reboot_FCU();

  void publish_bodyrate_ctrl(const Controller_Output_t& u, const ros::Time& stamp);
  void publish_attitude_ctrl(const Controller_Output_t& u, const ros::Time& stamp);
  void publish_trigger(const nav_msgs::Odometry& odom_msg);
};

#endif
