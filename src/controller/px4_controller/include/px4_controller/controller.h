#ifndef PX4_CONTROLLER_CONTROLLER_H
#define PX4_CONTROLLER_CONTROLLER_H

#include <queue>
#include <Eigen/Dense>

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include <px4_controller/input.h>

struct Desired_State_t
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  Eigen::Quaterniond q;
  double yaw;
  double yaw_rate;

  Desired_State_t() = default;

  explicit Desired_State_t(Odom_Data_t& odom)
      : p(odom.p),
        v(Eigen::Vector3d::Zero()),
        a(Eigen::Vector3d::Zero()),
        j(Eigen::Vector3d::Zero()),
        q(odom.q),
        yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
        yaw_rate(0.0)
  {
  }
};

struct Controller_Output_t
{
  Eigen::Quaterniond q;
  Eigen::Vector3d bodyrates;
  double thrust;
};

class LinearControl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LinearControl(Parameter_t& param);
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t& des,
                                                const Odom_Data_t& odom,
                                                const Imu_Data_t& imu,
                                                Controller_Output_t& u);
  bool estimateThrustModel(const Eigen::Vector3d& est_a, const Parameter_t& param);
  void resetThrustMapping();

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  const double rho2_ = 0.998;
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc);
  double fromQuaternion2yaw(const Eigen::Quaterniond& q);
};

#endif
