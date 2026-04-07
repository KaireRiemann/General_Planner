#include<tracking_controller/tracking_controller.h>

using namespace controller;

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"tracking_control_node");
    ros::NodeHandle nh("~");

    trackingController t_ctrl;
    t_ctrl.init(nh);

    ros::spin();
    return 0;
}