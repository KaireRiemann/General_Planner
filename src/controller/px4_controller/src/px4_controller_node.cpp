#include <signal.h>

#include <ros/ros.h>

#include <px4_controller/PX4CtrlFSM.h>

namespace
{
std::string trimSlash(const std::string& ns)
{
  if (ns.empty() || ns == "/")
  {
    return "";
  }
  if (ns.front() == '/')
  {
    return ns;
  }
  return "/" + ns;
}

std::string withDefault(const ros::NodeHandle& nh,
                        const std::string& name,
                        const std::string& default_value)
{
  std::string value;
  nh.param(name, value, default_value);
  return value;
}

std::string nsJoin(const std::string& ns, const std::string& leaf)
{
  const std::string clean = trimSlash(ns);
  return clean.empty() ? "/" + leaf : clean + "/" + leaf;
}

void mySigintHandler(int)
{
  ROS_INFO("[px4_controller] exit...");
  ros::shutdown();
}
}  // namespace

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "px4_controller");
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);
  ros::Duration(1.0).sleep();

  Parameter_t param;
  param.config_from_ros_handle(nh);

  std::string mavros_ns;
  nh.param("mavros_ns", mavros_ns, std::string("/mavros"));

  const std::string state_topic = withDefault(nh, "state_topic", nsJoin(mavros_ns, "state"));
  const std::string extended_state_topic =
      withDefault(nh, "extended_state_topic", nsJoin(mavros_ns, "extended_state"));
  const std::string imu_topic = withDefault(nh, "imu_topic", nsJoin(mavros_ns, "imu/data"));
  const std::string rc_topic = withDefault(nh, "rc_topic", nsJoin(mavros_ns, "rc/in"));
  const std::string battery_topic = withDefault(nh, "battery_topic", nsJoin(mavros_ns, "battery"));
  const std::string attitude_cmd_topic =
      withDefault(nh, "attitude_cmd_topic", nsJoin(mavros_ns, "setpoint_raw/attitude"));
  const std::string set_mode_service = withDefault(nh, "set_mode_service", nsJoin(mavros_ns, "set_mode"));
  const std::string arming_service = withDefault(nh, "arming_service", nsJoin(mavros_ns, "cmd/arming"));
  const std::string command_service = withDefault(nh, "command_service", nsJoin(mavros_ns, "cmd/command"));
  const std::string takeoff_land_topic = withDefault(nh, "takeoff_land_topic", "takeoff_land");
  const std::string traj_start_trigger_topic = withDefault(nh, "traj_start_trigger_topic", "/traj_start_trigger");
  const std::string debug_topic = withDefault(nh, "debug_topic", "/debugPx4ctrl");

  LinearControl controller(param);
  PX4CtrlFSM fsm(param, controller);

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>(state_topic, 10, boost::bind(&State_Data_t::feed, &fsm.state_data, _1));
  ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>(
      extended_state_topic, 10, boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "odom", 100, boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1), ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());
  ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
      "cmd", 100, boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1), ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
      imu_topic, 100, boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1), ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());

  ros::Subscriber rc_sub;
  if (!param.takeoff_land.no_RC)
  {
    rc_sub =
        nh.subscribe<mavros_msgs::RCIn>(rc_topic, 10, boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
  }

  ros::Subscriber bat_sub = nh.subscribe<sensor_msgs::BatteryState>(
      battery_topic, 100, boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1), ros::VoidConstPtr(),
      ros::TransportHints().tcpNoDelay());
  ros::Subscriber takeoff_land_sub = nh.subscribe<quadrotor_msgs::TakeoffLand>(
      takeoff_land_topic, 100, boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
      ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>(attitude_cmd_topic, 10);
  fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>(traj_start_trigger_topic, 10);
  fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>(debug_topic, 10);
  fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>(set_mode_service);
  fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>(arming_service);
  fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>(command_service);

  ROS_INFO("[px4_controller] odom<=%s cmd<=%s mavros_ns=%s", nh.resolveName("odom").c_str(),
           nh.resolveName("cmd").c_str(), trimSlash(mavros_ns).c_str());

  ros::Duration(0.5).sleep();

  if (param.takeoff_land.no_RC)
  {
    ROS_WARN("[px4_controller] RC disabled. auto_hover_on_start=%s",
             param.auto_hover_on_start ? "true" : "false");
  }
  else
  {
    ROS_INFO("[px4_controller] Waiting for RC");
    while (ros::ok())
    {
      ros::spinOnce();
      if (fsm.rc_is_received(ros::Time::now()))
      {
        ROS_INFO("[px4_controller] RC received.");
        break;
      }
      ros::Duration(0.1).sleep();
    }
  }

  int trials = 0;
  while (ros::ok() && !fsm.state_data.current_state.connected)
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    if (trials++ > 5)
    {
      ROS_ERROR("[px4_controller] Unable to connect to PX4.");
    }
  }

  ros::Rate r(param.ctrl_freq_max);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    fsm.process();
  }

  return 0;
}
