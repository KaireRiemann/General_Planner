#include <cmath>

#include <px4_controller/input.h>

RC_Data_t::RC_Data_t()
{
  rcv_stamp = ros::Time(0);
  last_mode = -1.0;
  last_gear = -1.0;
  last_reboot_cmd = -1.0;

  is_hover_mode = true;
  enter_hover_mode = false;
  is_command_mode = true;
  enter_command_mode = false;
  toggle_reboot = false;
  for (int i = 0; i < 4; ++i)
  {
    ch[i] = 0.0;
  }
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
  msg = *pMsg;
  rcv_stamp = ros::Time::now();

  for (int i = 0; i < 4; i++)
  {
    ch[i] = (static_cast<double>(msg.channels[i]) - 1500.0) / 500.0;
    if (ch[i] > DEAD_ZONE)
      ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
    else if (ch[i] < -DEAD_ZONE)
      ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
    else
      ch[i] = 0.0;
  }

  mode = (static_cast<double>(msg.channels[4]) - 1000.0) / 1000.0;
  gear = (static_cast<double>(msg.channels[5]) - 1000.0) / 1000.0;
  reboot_cmd = (static_cast<double>(msg.channels[7]) - 1000.0) / 1000.0;

  check_validity();

  if (!have_init_last_mode)
  {
    have_init_last_mode = true;
    last_mode = mode;
  }
  if (!have_init_last_gear)
  {
    have_init_last_gear = true;
    last_gear = gear;
  }
  if (!have_init_last_reboot_cmd)
  {
    have_init_last_reboot_cmd = true;
    last_reboot_cmd = reboot_cmd;
  }

  enter_hover_mode = (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE);
  is_hover_mode = mode > API_MODE_THRESHOLD_VALUE;

  if (is_hover_mode)
  {
    enter_command_mode = (last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE);
    is_command_mode = gear > GEAR_SHIFT_VALUE;
  }

  if (!is_hover_mode && !is_command_mode)
  {
    toggle_reboot = (last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE);
  }
  else
  {
    toggle_reboot = false;
  }

  last_mode = mode;
  last_gear = gear;
  last_reboot_cmd = reboot_cmd;
}

void RC_Data_t::check_validity()
{
  if (!(mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1))
  {
    ROS_ERROR("RC data validity check failed. mode=%f, gear=%f, reboot_cmd=%f", mode, gear, reboot_cmd);
  }
}

bool RC_Data_t::check_centered()
{
  return std::abs(ch[0]) < 1e-5 && std::abs(ch[1]) < 1e-5 && std::abs(ch[2]) < 1e-5 &&
         std::abs(ch[3]) < 1e-5;
}

bool RC_Data_t::is_received(const ros::Time& now_time)
{
  return (now_time - rcv_stamp).toSec() < 0.5;
}

Odom_Data_t::Odom_Data_t()
{
  rcv_stamp = ros::Time(0);
  q.setIdentity();
  recv_new_msg = false;
}

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
  ros::Time now = ros::Time::now();

  msg = *pMsg;
  rcv_stamp = now;
  recv_new_msg = true;
  uav_utils::extract_odometry(pMsg, p, v, q, w);

  static int count = 9999;
  static ros::Time last_clear_count_time = ros::Time(0.0);
  if ((now - last_clear_count_time).toSec() > 1.0)
  {
    if (count < 100)
    {
      ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
    }
    count = 0;
    last_clear_count_time = now;
  }
  count++;
}

Imu_Data_t::Imu_Data_t() { rcv_stamp = ros::Time(0); }

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
  ros::Time now = ros::Time::now();

  msg = *pMsg;
  rcv_stamp = now;

  w(0) = msg.angular_velocity.x;
  w(1) = msg.angular_velocity.y;
  w(2) = msg.angular_velocity.z;

  a(0) = msg.linear_acceleration.x;
  a(1) = msg.linear_acceleration.y;
  a(2) = msg.linear_acceleration.z;

  q.x() = msg.orientation.x;
  q.y() = msg.orientation.y;
  q.z() = msg.orientation.z;
  q.w() = msg.orientation.w;

  static int count = 9999;
  static ros::Time last_clear_count_time = ros::Time(0.0);
  if ((now - last_clear_count_time).toSec() > 1.0)
  {
    if (count < 100)
    {
      ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
    }
    count = 0;
    last_clear_count_time = now;
  }
  count++;
}

State_Data_t::State_Data_t() {}
void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg) { current_state = *pMsg; }

ExtendedState_Data_t::ExtendedState_Data_t() {}
void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg) { current_extended_state = *pMsg; }

Command_Data_t::Command_Data_t() { rcv_stamp = ros::Time(0); }

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
  msg = *pMsg;
  rcv_stamp = ros::Time::now();

  p(0) = msg.position.x;
  p(1) = msg.position.y;
  p(2) = msg.position.z;

  v(0) = msg.velocity.x;
  v(1) = msg.velocity.y;
  v(2) = msg.velocity.z;

  a(0) = msg.acceleration.x;
  a(1) = msg.acceleration.y;
  a(2) = msg.acceleration.z;

  j(0) = msg.jerk.x;
  j(1) = msg.jerk.y;
  j(2) = msg.jerk.z;

  yaw = uav_utils::normalize_angle(msg.yaw);
  yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t() { rcv_stamp = ros::Time(0); }

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
  msg = *pMsg;
  rcv_stamp = ros::Time::now();

  double voltage = 0.0;
  for (double cell_v : pMsg->cell_voltage)
  {
    voltage += cell_v;
  }
  volt = 0.8 * volt + 0.2 * voltage;
  percentage = pMsg->percentage;

  static ros::Time last_print_t = ros::Time(0);
  if (percentage > 0.05)
  {
    if ((rcv_stamp - last_print_t).toSec() > 10.0)
    {
      ROS_INFO("[px4_controller] Voltage=%.3f, percentage=%.3f", volt, percentage);
      last_print_t = rcv_stamp;
    }
  }
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t() { rcv_stamp = ros::Time(0); }

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{
  msg = *pMsg;
  rcv_stamp = ros::Time::now();
  triggered = true;
  takeoff_land_cmd = pMsg->takeoff_land_cmd;
}
