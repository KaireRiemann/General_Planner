#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H

#include<ros/ros.h>
#include<Eigen/Eigen>
#include<Eigen/Dense>
#include<queue>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include<tf/tf.h>
#include<quadrotor_msgs/PositionCommand.h>
#include<mavros_msgs/AttitudeTarget.h>
#include<mavros_msgs/PositionTarget.h>
#include<tracking_controller/utils.h>

namespace controller
{
    class trackingController
    {
    private:
        //parameters
        //control mode
        bool body_rate_control;
        bool attitude_control;
        bool acc_control;
        bool vel_control;
        Eigen::Vector3d p_pos;
        Eigen::Vector3d i_pos;
        Eigen::Vector3d d_pos;
        Eigen::Vector3d p_vel;
        Eigen::Vector3d i_vel;
        Eigen::Vector3d d_vel;
        double attitude_control_tau;
        double hover_thrust;
        bool verbose;

        //controller data
        bool have_odom;
        bool have_imu;
        bool thrust_ready;
        bool first_target_received;
        bool target_received;
        bool first_time;
        nav_msgs::Odometry odom;
        sensor_msgs::Imu imu_data;
        quadrotor_msgs::PositionCommand traj_cmd;
        ros::Time prev_time;
        double delta_time;
        //integral of position error
        Eigen::Vector3d pos_error_int;
        //integral of velocity error
        Eigen::Vector3d vel_error_int;
        //delta of position error
        Eigen::Vector3d delta_pos_error;
        Eigen::Vector3d prev_pos_error;
        //delta of velocity error
        Eigen::Vector3d delta_vel_error;
        Eigen::Vector3d prev_vel_error;
        double cmd_thrust;
        ros::Time cmd_thrust_time;

        //kalman filter
        bool kf_first_time;
        ros::Time kf_start_time;
        double start_var;
        double process_noise_var;
        double measure_noise_var;
        std::deque<double> prev_estimate_thrusts;

        //visualization
        geometry_msgs::PoseStamped pos_vis;
        std::deque<geometry_msgs::PoseStamped> hist_traj;
        geometry_msgs::PoseStamped target_pos_vis;
        std::deque<geometry_msgs::PoseStamped> target_hist_traj;
        bool vel_first_time;
        Eigen::Vector3d prev_vel;
        ros::Time vel_prev_time;

        //ROS Utils
        ros::NodeHandle nh;
        //subscriber
        ros::Subscriber odom_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber target_sub; // subscribe the target trajectory state 
        //publisher
        ros::Publisher att_cmd_pub;
        ros::Publisher acc_cmd_pub;
        ros::Publisher vel_cmd_pub;
        ros::Publisher pos_cmd_pub;
        ros::Publisher pose_vis_pub;
        ros::Publisher target_vis_pub;
        ros::Publisher hist_traj_vis_pub;
        ros::Publisher target_hist_traj_vis_pub;
        ros::Publisher vel_acc_vis_pub;
        //timer
        ros::Timer cmd_timer;
        ros::Timer thrust_estimator_timer;
        ros::Timer vis_timer;

        //callback
        void odomCallback(const nav_msgs::OdometryConstPtr &msg);
        void imuCallback(const sensor_msgs::ImuConstPtr &msg);
        void targetCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);
        void cmdCallback(const ros::TimerEvent &e);
        // void thrustEstimateCallback(const ros::TimerEvent &e);
        // void visCallback(const ros::TimerEvent &e);

        //helper function
        void computeCmdRef(Eigen::Vector4d &attitude_ref_quat, Eigen::Vector3d &acc_ref, Eigen::Vector3d &vel_ref);
        void computeBodyRate(const Eigen::Vector4d &attitude_ref_quat, const Eigen::Vector3d &acc_ref, const Eigen::Vector4d &cmd);
        void publishHoverCmd(const Eigen::Vector3d &pos_hover);
        void publishVelCmd(const Eigen::Vector3d &vel_ref);
        void publishAccCmd(const Eigen::Vector3d &acc_ref);
        void publishAttitudeCmd(const Eigen::Vector4d &attitude_quat_ref, Eigen::Vector3d &acc_ref);
        void publishBodyRateCmd(const Eigen::Vector4d &cmd);
        void publishPoseVis();
        void publishHistTraj();
        void publishTargetVis();
        void publishTargetHistTraj();
        void publishVelAndAccVis();

    public:
        void init(ros::NodeHandle &nh);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };  
}// namespace controller

#endif