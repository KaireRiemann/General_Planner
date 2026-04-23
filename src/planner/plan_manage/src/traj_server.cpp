#include <nav_msgs/Odometry.h>
#include <traj_utils/PerchingTraj.h>
#include <traj_utils/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <algorithm>
#include <utility>

#include "traj_utils/minco_types.hpp"

using namespace Eigen;
using ego_planner::MINCOBoundaryState3D;
using ego_planner::MINCOTraj3D;
using ego_planner::SnapBoundaryState3D;
using ego_planner::SnapTraj3D;
using ego_planner::YawBoundaryState1D;
using ego_planner::YawTraj1D;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<MINCOTraj3D> traj_;
boost::shared_ptr<SnapTraj3D> snap_traj_;
boost::shared_ptr<YawTraj1D> yaw_traj_;
int traj_order_ = MINCOTraj3D::ORDER;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;
double heartbeat_timeout_ = 1.5;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
bool has_yaw_traj_ = false;

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

Eigen::Vector3d evalTrajectory(const double t, const int derivative_order)
{
  if (traj_order_ == SnapTraj3D::ORDER && snap_traj_)
  {
    return snap_traj_->evaluate(t, derivative_order);
  }
  return traj_ ? traj_->evaluate(t, derivative_order) : Eigen::Vector3d::Zero();
}

// ====================================================================
// ROS trajectory reception with compact MINCO decoding
// ====================================================================
void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  constexpr int kBoundaryNum = MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM;
  if (msg->order != MINCOTraj3D::ORDER)
  {
    ROS_ERROR("[traj_server] PolyTraj only supports jerk MINCO order %d now!", MINCOTraj3D::ORDER);
    return;
  }

  const int M = msg->duration.size();
  if (M == 0) return;

  Eigen::VectorXd T(M);
  for (int i = 0; i < M; ++i) T(i) = msg->duration[i];

  const int Nc = msg->coef_x.size();
  const int expected = M + 2 * kBoundaryNum - 1;
  if (Nc != expected ||
      msg->coef_y.size() != msg->coef_x.size() ||
      msg->coef_z.size() != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] Invalid PolyTraj encoding. expected=%d actual=%d", expected, Nc);
    return;
  }

  // 提取编码在系数中的边界与节点信息
  Eigen::MatrixXd C(Nc, 3);
  for (int i = 0; i < Nc; ++i)
  {
    C(i, 0) = msg->coef_x[i];
    C(i, 1) = msg->coef_y[i];
    C(i, 2) = msg->coef_z[i];
  }

  Eigen::MatrixXd P_inner;
  if (M > 1) P_inner = C.block(kBoundaryNum, 0, M - 1, 3).transpose();
  else P_inner.resize(3, 0);

  MINCOBoundaryState3D headState = MINCOBoundaryState3D::Zero();
  MINCOBoundaryState3D tailState = MINCOBoundaryState3D::Zero();
  for (int d = 0; d < kBoundaryNum; ++d)
  {
    headState.col(d) = C.row(d).transpose();
    tailState.col(d) = C.row(Nc - kBoundaryNum + d).transpose();
  }

  traj_order_ = MINCOTraj3D::ORDER;
  has_yaw_traj_ = false;
  yaw_traj_.reset();
  snap_traj_.reset();
  traj_.reset(new MINCOTraj3D());
  traj_->generate(P_inner, headState, tailState, T);
  traj_duration_ = traj_->getTotalDuration();

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;
  receive_traj_ = true;
}

void decodePerchingYawTrajectory(const traj_utils::PerchingTrajPtr &msg)
{
  has_yaw_traj_ = false;
  yaw_traj_.reset();
  if (!msg->has_yaw)
  {
    return;
  }

  if (msg->yaw_order != YawTraj1D::ORDER)
  {
    ROS_WARN("[traj_server] Unsupported yaw trajectory order %d, fallback to position-forward yaw.",
             static_cast<int>(msg->yaw_order));
    return;
  }

  const int yaw_M = static_cast<int>(msg->duration_yaw.size());
  const int yaw_boundary_num = YawTraj1D::BOUNDARY_DERIVATIVE_NUM;
  const int expected_yaw = yaw_M + 2 * yaw_boundary_num - 1;
  if (yaw_M <= 0 || static_cast<int>(msg->coef_yaw.size()) != expected_yaw)
  {
    ROS_WARN("[traj_server] Invalid yaw trajectory encoding, fallback to position-forward yaw.");
    return;
  }

  Eigen::VectorXd yaw_T(yaw_M);
  for (int i = 0; i < yaw_M; ++i)
  {
    yaw_T(i) = msg->duration_yaw[i];
  }

  Eigen::Matrix<double, 1, Eigen::Dynamic> yaw_inner(1, std::max(0, yaw_M - 1));
  if (yaw_M > 1)
  {
    for (int i = 0; i < yaw_M - 1; ++i)
    {
      yaw_inner(0, i) = msg->coef_yaw[yaw_boundary_num + i];
    }
  }

  YawBoundaryState1D yaw_head = YawBoundaryState1D::Zero();
  YawBoundaryState1D yaw_tail = YawBoundaryState1D::Zero();
  for (int d = 0; d < yaw_boundary_num; ++d)
  {
    yaw_head(0, d) = msg->coef_yaw[d];
    yaw_tail(0, d) = msg->coef_yaw[expected_yaw - yaw_boundary_num + d];
  }

  yaw_traj_.reset(new YawTraj1D());
  has_yaw_traj_ = yaw_traj_->generate(yaw_inner, yaw_head, yaw_tail, yaw_T);
  if (!has_yaw_traj_)
  {
    yaw_traj_.reset();
    ROS_WARN("[traj_server] Failed to decode yaw trajectory, fallback to position-forward yaw.");
  }
}

void perchingTrajCallback(traj_utils::PerchingTrajPtr msg)
{
  if (msg->order != SnapTraj3D::ORDER)
  {
    ROS_ERROR("[traj_server] PerchingTraj only supports snap MINCO order %d now!", SnapTraj3D::ORDER);
    return;
  }

  constexpr int kBoundaryNum = SnapTraj3D::BOUNDARY_DERIVATIVE_NUM;
  const int M = msg->duration.size();
  if (M == 0) return;

  Eigen::VectorXd T(M);
  for (int i = 0; i < M; ++i) T(i) = msg->duration[i];

  const int Nc = msg->coef_x.size();
  const int expected = M + 2 * kBoundaryNum - 1;
  if (Nc != expected ||
      msg->coef_y.size() != msg->coef_x.size() ||
      msg->coef_z.size() != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] Invalid PerchingTraj encoding. expected=%d actual=%d", expected, Nc);
    return;
  }

  Eigen::MatrixXd C(Nc, 3);
  for (int i = 0; i < Nc; ++i)
  {
    C(i, 0) = msg->coef_x[i];
    C(i, 1) = msg->coef_y[i];
    C(i, 2) = msg->coef_z[i];
  }

  Eigen::MatrixXd P_inner;
  if (M > 1) P_inner = C.block(kBoundaryNum, 0, M - 1, 3).transpose();
  else P_inner.resize(3, 0);

  SnapBoundaryState3D headState = SnapBoundaryState3D::Zero();
  SnapBoundaryState3D tailState = SnapBoundaryState3D::Zero();
  for (int d = 0; d < kBoundaryNum; ++d)
  {
    headState.col(d) = C.row(d).transpose();
    tailState.col(d) = C.row(Nc - kBoundaryNum + d).transpose();
  }

  traj_order_ = SnapTraj3D::ORDER;
  snap_traj_.reset(new SnapTraj3D());
  snap_traj_->generate(P_inner, headState, tailState, T);
  traj_.reset();
  traj_duration_ = snap_traj_->getTotalDuration();

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;
  decodePerchingYawTrajectory(msg);
  receive_traj_ = true;
}

std::pair<double, double> evaluate_yaw_traj(double t_cur)
{
  if (!has_yaw_traj_ || !yaw_traj_)
  {
    return std::make_pair(last_yaw_, 0.0);
  }
  const double t = std::min(std::max(0.0, t_cur), yaw_traj_->getTotalDuration());
  return std::make_pair(yaw_traj_->evaluate(t, 0)(0),
                        yaw_traj_->evaluate(t, 1)(0));
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? evalTrajectory(t_cur + time_forward_, 0) - pos
                            : evalTrajectory(traj_duration_, 0) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI) d_yaw -= 2 * M_PI;
  if (d_yaw <= -M_PI) d_yaw += 2 * M_PI;

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max)) d_yaw = d_yaw_max;
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI) yaw -= 2 * M_PI;
  if (yaw < -M_PI) yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;
  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  if (heartbeat_time_.toSec() <= 1e-5) return;
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > heartbeat_timeout_)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner for %.2fs, is it dead?",
              heartbeat_timeout_);
    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = evalTrajectory(t_cur, 0);
    vel = evalTrajectory(t_cur, 1);
    acc = evalTrajectory(t_cur, 2);
    jer = evalTrajectory(t_cur, 3);

    /*** calculate yaw ***/
    if (has_yaw_traj_)
    {
      yaw_yawdot = evaluate_yaw_traj(t_cur);
    }
    else
    {
      yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
    }

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI) slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI) slowly_flip_yaw_target_ += 2 * M_PI;
    
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished) return;

    pos = evalTrajectory(traj_duration_, 0);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_) finished = true;
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_) finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished) return;

    pos = evalTrajectory(traj_duration_, 0);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI) last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI) last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_) finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_) finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber perching_traj_sub = nh.subscribe("planning/perching_trajectory", 10, perchingTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/heartbeat_timeout", heartbeat_timeout_, 1.5);
  heartbeat_timeout_ = std::max(0.5, heartbeat_timeout_);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();
  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}
