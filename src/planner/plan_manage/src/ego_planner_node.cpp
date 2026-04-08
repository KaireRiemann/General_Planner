#include <ros/ros.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;
  rebo_replan.init(nh);

  ros::spin();
  // ros::AsyncSpinner async_spinner(4);
  // async_spinner.start();
  // ros::waitForShutdown();
  return 0;
}
