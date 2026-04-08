#include <cmath>

#include <px4_controller/controller.h>

double LinearControl::fromQuaternion2yaw(const Eigen::Quaterniond& q)
{
  return atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
               q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
}

LinearControl::LinearControl(Parameter_t& param) : param_(param) { resetThrustMapping(); }

quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(const Desired_State_t& des,
                                                             const Odom_Data_t& odom,
                                                             const Imu_Data_t& imu,
                                                             Controller_Output_t& u)
{
  Eigen::Vector3d Kp;
  Eigen::Vector3d Kv;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;

  Eigen::Vector3d des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
  des_acc += Eigen::Vector3d(0.0, 0.0, param_.gra);

  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);

  const double yaw_odom = fromQuaternion2yaw(odom.q);
  const double sin_yaw = std::sin(yaw_odom);
  const double cos_yaw = std::cos(yaw_odom);
  const double roll = (des_acc(0) * sin_yaw - des_acc(1) * cos_yaw) / param_.gra;
  const double pitch = (des_acc(0) * cos_yaw + des_acc(1) * sin_yaw) / param_.gra;

  Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = imu.q * odom.q.inverse() * q;
  u.bodyrates = Eigen::Vector3d::Zero();

  debug_msg_.des_p_x = des.p(0);
  debug_msg_.des_p_y = des.p(1);
  debug_msg_.des_p_z = des.p(2);
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  debug_msg_.des_thr = u.thrust;
  debug_msg_.thr2acc = thr2acc_;

  timed_thrust_.push(std::make_pair(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }

  return debug_msg_;
}

double LinearControl::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d& des_acc)
{
  return des_acc(2) / thr2acc_;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d& est_a, const Parameter_t&)
{
  ros::Time t_now = ros::Time::now();
  while (!timed_thrust_.empty())
  {
    const auto t_t = timed_thrust_.front();
    const double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045)
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035)
    {
      return false;
    }

    const double thr = t_t.second;
    timed_thrust_.pop();
    const double gamma = 1.0 / (rho2_ + thr * P_ * thr);
    const double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1.0 - K * thr) * P_ / rho2_;
    debug_msg_.thr2acc = thr2acc_;
    return true;
  }

  return false;
}

void LinearControl::resetThrustMapping()
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
