#include<tracking_controller/tracking_controller.h>

//设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
#define POSTYPEMASK 0b101111111000
#define VELTYPEMASK 0b101111000111
#define ACCTYPEMASK 0b100000111111
using namespace std;

namespace controller
{
    void trackingController::init(ros::NodeHandle &nh)
    {
        have_odom = false;
        have_imu = false;
        thrust_ready = false;
        first_target_received = false;
        target_received = false;
        first_time = true;
        p_pos.setZero();
        i_pos.setZero();
        d_pos.setZero();
        p_vel.setZero();
        i_vel.setZero();
        d_vel.setZero();
        vector<double> p_pos_temp;
        vector<double> i_pos_temp;
        vector<double> d_pos_temp;
        vector<double> p_vel_temp;
        vector<double> i_vel_temp;
        vector<double> d_vel_temp;

        nh.param("controller/body_rate_control_", body_rate_control, false);
        nh.param("controller/acc_control_",acc_control,false);
        nh.param("controller/attitude_control_",attitude_control,false);
        nh.param("controller/vel_control_",vel_control,false);
        nh.param("controller/attitude_control_tau_",attitude_control_tau,0.3);
        nh.param("controller/hover_thrust_",hover_thrust,0.3);
        if(nh.getParam("controller/positon_p_",p_pos_temp))
        {
            p_pos(0) = p_pos_temp[0];
            p_pos(1) = p_pos_temp[1];
            p_pos(2) = p_pos_temp[2];
        }
        if(nh.getParam("controller/positon_i_",i_pos_temp))
        {
            i_pos(0) = i_pos_temp[0];
            i_pos(1) = i_pos_temp[1];
            i_pos(2) = i_pos_temp[2];
        }
        if(nh.getParam("controller/positon_d_",d_pos_temp))
        {
            d_pos(0) = d_pos_temp[0];
            d_pos(1) = d_pos_temp[1];
            d_pos(2) = d_pos_temp[2];
        }
        if(nh.getParam("controller/velocity_p_",p_vel_temp))
        {
            p_vel(0) = p_vel_temp[0];
            p_vel(1) = p_vel_temp[1];
            p_vel(2) = p_vel_temp[2];
        }
        if(nh.getParam("controller/velocity_i_",i_vel_temp))
        {
            i_vel(0) = i_vel_temp[0];
            i_vel(1) = i_vel_temp[1];
            i_vel(2) = i_vel_temp[2];
        }
        if(nh.getParam("controller/velocity_d_",d_vel_temp))
        {
            d_vel(0) = d_vel_temp[0];
            d_vel(1) = d_vel_temp[1];
            d_vel(2) = d_vel_temp[2];
        }

        att_cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget>("att_cmd_topic",10);
        acc_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("acc_cmd_topic",10);
        vel_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("vel_cmd_topic",10);
        pos_cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("vel_cmd_topic",10);
        pose_vis_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",10);
        hist_traj_vis_pub = nh.advertise<nav_msgs::Path>("/trajectory_history",10);
        target_vis_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_pose",10);
        hist_traj_vis_pub = nh.advertise<nav_msgs::Path>("/target_trajectory_history",10);
        vel_acc_vis_pub = nh.advertise<visualization_msgs::Marker>("/vel_acc_info",10);

        odom_sub = nh.subscribe("odom_topic",1,&trackingController::odomCallback,this);
        imu_sub = nh.subscribe("imu_topic",1,&trackingController::imuCallback,this);
        target_sub = nh.subscribe("trajectory_target_topic",1,&trackingController::targetCallback,this);

        cmd_timer = nh.createTimer(ros::Duration(0.02),&trackingController::cmdCallback,this);
        // thrust_estimator_timer = nh.createTimer(ros::Duration(0.02),&trackingController::thrustEstimateCallback,this);
        // vis_timer = nh.createTimer(ros::Duration(0.02),&trackingController::visCallback,this);
        
    }

    void trackingController::odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom = *msg;
        have_odom = true;
    }

    void trackingController::imuCallback(const sensor_msgs::ImuConstPtr &msg)
    {
        imu_data = *msg;
        have_imu = true;
    }

    void trackingController::targetCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
    {
        traj_cmd = *msg;
        first_target_received = true;
        target_received = true;
    }

    //find the reference acceleration and velocity for motore, then convert the acceleration into attitude
    void trackingController::computeCmdRef(Eigen::Vector4d &attitude_ref_quat, Eigen::Vector3d &acc_ref, Eigen::Vector3d &vel_ref)
    {
        if(first_time)
        {
            prev_time = ros::Time::now();
            delta_time = 0.0;
            pos_error_int.setZero();
            vel_error_int.setZero();
        }
        else
        {
            ros::Time current_time = ros::Time::now();
            delta_time = (current_time - prev_time).toSec();
            prev_time = current_time;
        }

        //target position,velocity and target acceleration
        Eigen::Vector3d pos_target(traj_cmd.position.x,traj_cmd.position.y,traj_cmd.position.z);
        Eigen::Vector3d vel_target(traj_cmd.velocity.x,traj_cmd.velocity.y,traj_cmd.velocity.z);
        Eigen::Vector3d acc_target(traj_cmd.acceleration.x,traj_cmd.acceleration.y,traj_cmd.acceleration.z);
        
        //feedback
        Eigen::Vector3d current_pos(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
        Eigen::Vector3d current_vel_body(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
        Eigen::Vector4d current_quat(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
        Eigen::Matrix3d current_rot = quat2RotMatrix(current_quat);
        //velocity body 2 world
        Eigen::Vector3d current_vel = current_rot * current_vel_body;

        Eigen::Vector3d position_error = pos_target - current_pos;
        Eigen::Vector3d velocity_error = vel_target - current_vel;
        pos_error_int += delta_time * position_error;
        vel_error_int += delta_time * velocity_error;

        if(first_time)
        {
            delta_pos_error.setZero();
            delta_vel_error.setZero();
            prev_pos_error = position_error;
            prev_vel_error = velocity_error;
            first_time = false;
        }
        else
        {
            delta_pos_error = (position_error - prev_pos_error)/delta_time;
            prev_pos_error = position_error;
            delta_vel_error = (velocity_error  - prev_vel_error)/delta_time;
            prev_vel_error = velocity_error;
        }

        //feedback velocity 
        Eigen::Vector3d vel_fb = p_pos.asDiagonal() * position_error + i_pos.asDiagonal() * pos_error_int + d_pos.asDiagonal() * delta_pos_error;
        //feedback acceleration
        Eigen::Vector3d acc_fb = p_pos.asDiagonal() * position_error + i_pos.asDiagonal() * pos_error_int + d_pos.asDiagonal() * delta_pos_error+
                                 p_vel.asDiagonal() * velocity_error + i_vel.asDiagonal() * vel_error_int + d_vel.asDiagonal() * delta_vel_error;

        //air drag
        Eigen::Vector3d acc_air_drag(0.0,0.0,0.0);
        //gravity
        Eigen::Vector3d gravity(0.0,0.0,-9.8);
        
        //reference velocity
        vel_ref = vel_target + vel_fb;
        //reference acceleration
        acc_ref = acc_target + acc_fb - acc_air_drag - gravity;

        //convert the reference acceleration into the reference attitude
        double yaw = controller::yaw_from_quaternion(odom.pose.pose.orientation);

        Eigen::Vector3d direction(cos(yaw),sin(yaw),0.0);
        Eigen::Vector3d z_direction = acc_ref.normalized();
        Eigen::Vector3d y_direction = (z_direction.cross(direction)).normalized();
        Eigen::Vector3d x_direction = (y_direction.cross(z_direction)).normalized();

        Eigen::Matrix3d attitude_ref_rot;
        attitude_ref_rot<<x_direction(0),y_direction(0),z_direction(0),
                          x_direction(1),y_direction(1),z_direction(1),
                          x_direction(2),y_direction(2),z_direction(2);

        attitude_ref_quat = controller::rotmat2Quaternion(attitude_ref_rot);

    }

    void trackingController::publishHoverCmd(const Eigen::Vector3d &pos_hover)
    {
        // tf2::Quaternion q(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
        // tf2::Matrix3x3 m(q);
        // double roll;
        // double pitch;
        // double yaw;
        // m.getRPY(roll,pitch,yaw);
        double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
        mavros_msgs::PositionTarget pos_cmd;
        pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pos_cmd.header.stamp = ros::Time::now();
        pos_cmd.header.frame_id="world";
        pos_cmd.type_mask = POSTYPEMASK;
        pos_cmd.position.x = pos_hover(0);
        pos_cmd.position.y = pos_hover(1);
        pos_cmd.position.z = pos_hover(2);
        pos_cmd.yaw = yaw;
        pos_cmd_pub.publish(pos_cmd);
    }

    void trackingController::publishVelCmd(const Eigen::Vector3d &vel_ref)
    {
        mavros_msgs::PositionTarget vel_cmd;
        vel_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        vel_cmd.header.stamp = ros::Time::now();
        vel_cmd.header.frame_id = "world";
        vel_cmd.type_mask = VELTYPEMASK;
        vel_cmd.velocity.x = vel_ref(0);
        vel_cmd.velocity.y = vel_ref(1);
        vel_cmd.velocity.z = vel_ref(2);
        vel_cmd.yaw = traj_cmd.yaw;

        vel_cmd_pub.publish(vel_cmd);
    }

    void trackingController::publishAccCmd(const Eigen::Vector3d &acc_ref)
    {
        mavros_msgs::PositionTarget acc_cmd;
        acc_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        acc_cmd.header.stamp = ros::Time::now();
        acc_cmd.header.frame_id = "world";
        acc_cmd.acceleration_or_force.x = acc_ref(0);
        acc_cmd.acceleration_or_force.y = acc_ref(1);
        acc_cmd.acceleration_or_force.z = acc_ref(2) - 9.8;
        acc_cmd.yaw = traj_cmd.yaw;
        acc_cmd.type_mask = ACCTYPEMASK;

        acc_cmd_pub.publish(acc_cmd);
    }

    void trackingController::publishAttitudeCmd(const Eigen::Vector4d &attitude_quat_ref, Eigen::Vector3d &acc_ref)
    {
        mavros_msgs::AttitudeTarget att_cmd;
        att_cmd.header.stamp = ros::Time::now();
        att_cmd.header.frame_id = "map";
        att_cmd.orientation.w = attitude_quat_ref(0);
        att_cmd.orientation.x = attitude_quat_ref(1);
        att_cmd.orientation.y = attitude_quat_ref(2);
        att_cmd.orientation.z = attitude_quat_ref(3);
        double thrust = acc_ref.norm();
        double thrust_percent = std::max(0.0,std::min(1.0,1.0*thrust/(9.8 * 1.0/hover_thrust)));
        cmd_thrust = thrust_percent;
        cmd_thrust_time = ros::Time::now();
        thrust_ready = true;
        att_cmd.thrust = thrust_percent;
        att_cmd.type_mask = att_cmd.IGNORE_PITCH_RATE + att_cmd.IGNORE_ROLL_RATE + att_cmd.IGNORE_YAW_RATE;
        att_cmd_pub.publish(att_cmd);
    }

    //the controller timer callback
    void trackingController::cmdCallback(const ros::TimerEvent &e)
    {
        static int num = 0;
        num++;
        if(num == 50)
        {
            if(!have_odom)
            {
                cout<<"no odom !"<<endl;
            }

            num = 0;
        }

        if(!target_received)
        {
            Eigen::Vector3d hover_position(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
            publishHoverCmd(hover_position);
        }
        else
        {
            Eigen::Vector3d vel_ref;
            Eigen::Vector3d acc_ref;
            Eigen::Vector4d attitude_quat_ref;
            computeCmdRef(attitude_quat_ref,acc_ref,vel_ref);
    
            if(vel_control)
            {
                publishVelCmd(vel_ref);
            }
            else if(acc_control)
            {
                publishAccCmd(acc_ref);
            }
            else if(attitude_control)
            {
                publishAttitudeCmd(attitude_quat_ref,acc_ref);
            }
        }


        target_received = false;

    }
}//namespace controller