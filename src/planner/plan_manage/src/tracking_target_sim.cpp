#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/GoalSet.h>

#include <plan_env/grid_map.h>
#include <path_searching/jps_a_star.hpp>

#include <algorithm>
#include <random>
#include <vector>

namespace ego_planner
{
  class TrackingTargetSim
  {
  public:
    TrackingTargetSim()
        : nh_(),
          pnh_("~"),
          rng_(std::random_device{}()),
          uni_(-1.0, 1.0)
    {
      pnh_.param("frame_id", frame_id_, std::string("world"));
      pnh_.param("ego_odom_topic", ego_odom_topic_, std::string("/drone_0_visual_slam/odom"));
      pnh_.param("reference_topic", reference_topic_, std::string("/tracking/reference"));
      pnh_.param("target_odom_topic", target_odom_topic_, std::string("/tracking/target_odom"));
      pnh_.param("target_marker_topic", target_marker_topic_, std::string("/tracking/target_marker"));
      pnh_.param("target_path_topic", target_path_topic_, std::string("/tracking/target_path"));
      pnh_.param("goal_topic", goal_topic_, std::string("/goal_with_id"));
      pnh_.param("external_goal_topic", external_goal_topic_, std::string("/tracking/target_goal"));

      pnh_.param("drone_id", drone_id_, 0);
      pnh_.param("use_jps", use_jps_, true);
      pnh_.param("publish_goal_with_id", publish_goal_with_id_, false);
      pnh_.param("use_external_goal", use_external_goal_, true);
      pnh_.param("lock_external_goal_height", lock_external_goal_height_, true);
      pnh_.param("reference_mode", reference_mode_, std::string("oracle_reference_mode"));

      pnh_.param("sim_dt", sim_dt_, 0.02);
      pnh_.param("replan_period", replan_period_, 0.5);
      pnh_.param("target_speed", target_speed_, 0.8);
      pnh_.param("goal_ahead_distance", goal_ahead_distance_, 4.0);
      pnh_.param("goal_lateral_jitter", goal_lateral_jitter_, 1.2);
      pnh_.param("goal_vertical_jitter", goal_vertical_jitter_, 0.0);
      pnh_.param("goal_reach_tolerance", goal_reach_tolerance_, 0.25);
      pnh_.param("horizon_time", horizon_time_, 4.0);
      pnh_.param("horizon_dt", horizon_dt_, 0.2);
      pnh_.param("reference_noise_std", reference_noise_std_, 0.08);
      pnh_.param("reference_velocity_noise_std", reference_velocity_noise_std_, 0.05);
      pnh_.param("reference_lag_sec", reference_lag_sec_, 0.20);
      pnh_.param("target_box_size", target_box_size_, 0.45);
      pnh_.param("min_init_separation", min_init_separation_, 2.5);
      pnh_.param("external_goal_min_separation", external_goal_min_separation_, 1.2);
      pnh_.param("tracking_eval_distance_min", tracking_eval_distance_min_, 1.5);
      pnh_.param("tracking_eval_distance_max", tracking_eval_distance_max_, 4.0);
      pnh_.param("tracking_eval_height_tolerance", tracking_eval_height_tolerance_, 0.4);
      pnh_.param("tracking_eval_los_clearance", tracking_eval_los_clearance_, 0.15);

      pnh_.param("init_offset_x", init_offset_.x(), 3.0);
      pnh_.param("init_offset_y", init_offset_.y(), 0.0);
      pnh_.param("init_offset_z", init_offset_.z(), 0.0);

      pnh_.param("jps_timeout", jps_timeout_, 0.08);
      pnh_.param("jps_jump_max_cells", jps_jump_max_cells_, 6);
      pnh_.param("jps_near_obs_radius", jps_near_obs_radius_, 1);

      marker_pub_ = nh_.advertise<visualization_msgs::Marker>(target_marker_topic_, 1);
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>(target_odom_topic_, 1);
      ref_pub_ = nh_.advertise<nav_msgs::Path>(reference_topic_, 1);
      path_pub_ = nh_.advertise<nav_msgs::Path>(target_path_topic_, 1);
      distance_violation_pub_ = nh_.advertise<std_msgs::Float64>("/tracking/metrics/distance_violation_ratio", 1);
      los_blocked_pub_ = nh_.advertise<std_msgs::Float64>("/tracking/metrics/los_blocked_ratio", 1);
      yaw_alignment_pub_ = nh_.advertise<std_msgs::Float64>("/tracking/metrics/yaw_alignment_error", 1);
      reference_available_pub_ = nh_.advertise<std_msgs::Float64>("/tracking/metrics/reference_available_ratio", 1);
      if (publish_goal_with_id_)
      {
        goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>(goal_topic_, 1);
      }

      ego_odom_sub_ = nh_.subscribe(ego_odom_topic_, 1, &TrackingTargetSim::egoOdomCallback, this);
      if (use_external_goal_)
      {
        external_goal_sub_ = nh_.subscribe(external_goal_topic_, 1, &TrackingTargetSim::externalGoalCallback, this);
      }

      grid_map_.reset(new GridMap);
      grid_map_->initMap(pnh_);

      jps_.reset(new JPSAStar(grid_map_, 0.0));
      jps_->setTimeOut(jps_timeout_);
      jps_->setJumpMaxCells(jps_jump_max_cells_);
      jps_->setJumpNearObsRadius(jps_near_obs_radius_);

      sim_timer_ = nh_.createTimer(ros::Duration(std::max(0.005, sim_dt_)),
                                   &TrackingTargetSim::simCallback, this);
      replan_timer_ = nh_.createTimer(ros::Duration(std::max(0.1, replan_period_)),
                                      &TrackingTargetSim::replanCallback, this);

      ROS_INFO("TrackingTargetSim ready. reference_topic=%s use_jps=%s",
               reference_topic_.c_str(),
               use_jps_ ? "yes" : "no");
      ROS_INFO("TrackingTargetSim reference_mode=%s", reference_mode_.c_str());
      if (use_external_goal_)
      {
        ROS_INFO("TrackingTargetSim external-goal mode enabled, waiting goal on: %s", external_goal_topic_.c_str());
      }
    }

  private:
    void egoOdomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
      ego_pos_.x() = msg->pose.pose.position.x;
      ego_pos_.y() = msg->pose.pose.position.y;
      ego_pos_.z() = msg->pose.pose.position.z;
      ego_vel_.x() = msg->twist.twist.linear.x;
      ego_vel_.y() = msg->twist.twist.linear.y;
      ego_vel_.z() = msg->twist.twist.linear.z;
      have_ego_odom_ = true;
    }

    void externalGoalCallback(const quadrotor_msgs::GoalSetConstPtr &msg)
    {
      if (!msg)
      {
        return;
      }

      if (msg->drone_id >= 0 && msg->drone_id != drone_id_)
      {
        return;
      }

      if (msg->goal[2] < -0.1)
      {
        return;
      }

      Eigen::Vector3d goal(msg->goal[0], msg->goal[1], msg->goal[2]);
      if (lock_external_goal_height_)
      {
        if (initialized_)
        {
          goal.z() = target_pos_.z();
        }
        else if (have_ego_odom_)
        {
          goal.z() = ego_pos_.z() + init_offset_.z();
        }
      }
      enforceSeparationFromEgo(goal, external_goal_min_separation_);
      goal = clampToMap(goal);

      if (!isFree(goal))
      {
        if (!findNearbyFreeGoal(goal, goal))
        {
          ROS_WARN("TrackingTargetSim: received goal in obstacle and failed to find nearby free cell.");
          return;
        }
      }

      external_goal_ = goal;
      have_external_goal_ = true;
      ROS_INFO("TrackingTargetSim received external goal: [%.2f %.2f %.2f]",
               external_goal_.x(), external_goal_.y(), external_goal_.z());
    }

    void enforceSeparationFromEgo(Eigen::Vector3d &pt, const double min_sep) const
    {
      if (!have_ego_odom_ || min_sep <= 1.0e-3)
      {
        return;
      }

      Eigen::Vector3d diff = pt - ego_pos_;
      if (diff.norm() >= min_sep)
      {
        return;
      }

      if (diff.norm() < 1.0e-3)
      {
        diff = init_offset_;
        if (diff.norm() < 1.0e-3)
        {
          diff = Eigen::Vector3d::UnitX();
        }
      }

      diff.normalize();
      pt = ego_pos_ + min_sep * diff;
    }

    bool isFree(const Eigen::Vector3d &pt) const
    {
      if (!grid_map_)
      {
        return true;
      }
      if (!mapWindowReady())
      {
        return true;
      }
      return grid_map_->getInflateOccupancy(pt) == 0;
    }

    bool mapWindowReady() const
    {
      if (!grid_map_)
      {
        return false;
      }

      const Eigen::Vector3d low = grid_map_->getUpdatedBoxLow();
      const Eigen::Vector3d high = grid_map_->getUpdatedBoxHigh();
      if (!low.allFinite() || !high.allFinite())
      {
        return false;
      }

      const double res = std::max(grid_map_->getResolution(), 1.0e-3);
      const Eigen::Vector3d span = high - low;
      return (span.array() > 6.0 * res).all();
    }

    Eigen::Vector3d clampToMap(const Eigen::Vector3d &pt) const
    {
      if (!mapWindowReady())
      {
        return pt;
      }

      const double res = std::max(grid_map_->getResolution(), 1.0e-3);
      const Eigen::Vector3d low = grid_map_->getUpdatedBoxLow() + Eigen::Vector3d::Constant(2.0 * res);
      const Eigen::Vector3d high = grid_map_->getUpdatedBoxHigh() - Eigen::Vector3d::Constant(2.0 * res);
      return pt.cwiseMax(low).cwiseMin(high);
    }

    bool findNearbyFreeGoal(const Eigen::Vector3d &seed, Eigen::Vector3d &out) const
    {
      if (isFree(seed))
      {
        out = seed;
        return true;
      }

      if (!mapWindowReady())
      {
        return false;
      }

      const double res = std::max(grid_map_->getResolution(), 1.0e-3);
      for (int ring = 1; ring <= 12; ++ring)
      {
        for (int dx = -ring; dx <= ring; ++dx)
        {
          for (int dy = -ring; dy <= ring; ++dy)
          {
            for (int dz = -1; dz <= 1; ++dz)
            {
              Eigen::Vector3d cand = seed + res * Eigen::Vector3d(dx, dy, dz);
              cand = clampToMap(cand);
              if (isFree(cand))
              {
                out = cand;
                return true;
              }
            }
          }
        }
      }
      return false;
    }

    bool planPathWithJps(const Eigen::Vector3d &start,
                         const Eigen::Vector3d &goal,
                         std::vector<Eigen::Vector3d> &path) const
    {
      path.clear();
      if (!use_jps_ || !jps_ || !mapWindowReady())
      {
        path.push_back(start);
        path.push_back(goal);
        return true;
      }
      return jps_->search(start, goal, path);
    }

    bool sampleNewGoalAndPath()
    {
      if (!have_ego_odom_ || !grid_map_)
      {
        return false;
      }

      if (use_external_goal_)
      {
        if (!have_external_goal_)
        {
          return false;
        }

        Eigen::Vector3d cand = clampToMap(external_goal_);
        if (!isFree(cand))
        {
          if (!findNearbyFreeGoal(cand, cand))
          {
            ROS_WARN_THROTTLE(1.0, "TrackingTargetSim external goal blocked and no nearby free goal found.");
            return false;
          }
          external_goal_ = cand;
        }

        std::vector<Eigen::Vector3d> new_path;
        if (!planPathWithJps(target_pos_, cand, new_path))
        {
          ROS_WARN_THROTTLE(1.0, "TrackingTargetSim: JPS failed for external goal.");
          return false;
        }
        if (new_path.size() < 2)
        {
          return false;
        }

        path_ = new_path;
        path_idx_ = 1;
        return true;
      }

      Eigen::Vector3d forward = ego_vel_;
      if (forward.norm() < 0.2)
      {
        forward = Eigen::Vector3d::UnitX();
      }
      else
      {
        forward.normalize();
      }

      Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
      Eigen::Vector3d lateral = up.cross(forward);
      if (lateral.norm() < 1.0e-3)
      {
        lateral = Eigen::Vector3d::UnitY();
      }
      lateral.normalize();
      up = forward.cross(lateral).normalized();

      Eigen::Vector3d base_goal = ego_pos_ + goal_ahead_distance_ * forward;
      base_goal.z() = target_pos_.z();

      if (!mapWindowReady())
      {
        Eigen::Vector3d coarse_goal = ego_pos_ + goal_ahead_distance_ * forward;
        coarse_goal.z() = target_pos_.z();
        path_.clear();
        path_.push_back(target_pos_);
        path_.push_back(coarse_goal);
        path_idx_ = 1;
        return true;
      }

      constexpr int kMaxTrial = 18;
      for (int k = 0; k < kMaxTrial; ++k)
      {
        Eigen::Vector3d cand = base_goal;
        cand += lateral * (goal_lateral_jitter_ * uni_(rng_));
        cand += up * (goal_vertical_jitter_ * uni_(rng_));
        cand.z() = target_pos_.z();
        cand = clampToMap(cand);

        if (!isFree(cand))
        {
          continue;
        }

        std::vector<Eigen::Vector3d> new_path;
        if (!planPathWithJps(target_pos_, cand, new_path))
        {
          continue;
        }

        if (new_path.size() < 2)
        {
          continue;
        }

        path_ = new_path;
        path_idx_ = 1;
        return true;
      }

      return false;
    }

    void advanceAlongPath(double dt)
    {
      target_vel_.setZero();
      if (path_.size() < 2 || path_idx_ >= path_.size())
      {
        return;
      }

      double remain_dist = std::max(0.0, target_speed_ * dt);
      while (remain_dist > 1.0e-6 && path_idx_ < path_.size())
      {
        const Eigen::Vector3d diff = path_[path_idx_] - target_pos_;
        const double dist = diff.norm();
        if (dist < goal_reach_tolerance_)
        {
          ++path_idx_;
          continue;
        }

        const Eigen::Vector3d dir = diff / std::max(dist, 1.0e-6);
        const double step = std::min(dist, remain_dist);
        target_pos_ += dir * step;
        target_vel_ = dir * target_speed_;
        remain_dist -= step;

        if (step >= dist - 1.0e-6)
        {
          ++path_idx_;
        }
      }

      if (use_external_goal_ && have_external_goal_)
      {
        if ((target_pos_ - external_goal_).norm() < goal_reach_tolerance_)
        {
          have_external_goal_ = false;
          path_.clear();
          path_.push_back(target_pos_);
          path_idx_ = 1;
          target_vel_.setZero();
          ROS_INFO("TrackingTargetSim reached external goal.");
        }
      }
    }

    void publishMarker() const
    {
      visualization_msgs::Marker box;
      box.header.stamp = ros::Time::now();
      box.header.frame_id = frame_id_;
      box.ns = "tracking_target";
      box.id = 0;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.pose.position.x = target_pos_.x();
      box.pose.position.y = target_pos_.y();
      box.pose.position.z = target_pos_.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = target_box_size_;
      box.scale.y = target_box_size_;
      box.scale.z = target_box_size_;
      box.color.a = 0.95;
      box.color.r = 1.0;
      box.color.g = 0.45;
      box.color.b = 0.05;
      marker_pub_.publish(box);
    }

    void publishTargetOdom() const
    {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = frame_id_;
      odom.child_frame_id = "tracking_target";
      odom.pose.pose.position.x = target_pos_.x();
      odom.pose.pose.position.y = target_pos_.y();
      odom.pose.pose.position.z = target_pos_.z();
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = target_vel_.x();
      odom.twist.twist.linear.y = target_vel_.y();
      odom.twist.twist.linear.z = target_vel_.z();
      odom_pub_.publish(odom);
    }

    std::vector<Eigen::Vector3d> sampleFutureReference() const
    {
      const int N = std::max(2, static_cast<int>(std::ceil(horizon_time_ / std::max(0.05, horizon_dt_))) + 1);
      std::vector<Eigen::Vector3d> samples;
      samples.reserve(static_cast<std::size_t>(N));

      Eigen::Vector3d sim_pos = target_pos_;
      Eigen::Vector3d sim_vel = target_vel_;
      std::size_t sim_idx = path_idx_;
      const double sim_step = std::max(0.05, horizon_dt_);

      for (int i = 0; i < N; ++i)
      {
        samples.push_back(sim_pos);

        double remain_dist = target_speed_ * sim_step;
        bool moved = false;
        while (remain_dist > 1.0e-6 && sim_idx < path_.size())
        {
          const Eigen::Vector3d diff = path_[sim_idx] - sim_pos;
          const double dist = diff.norm();
          if (dist < goal_reach_tolerance_)
          {
            ++sim_idx;
            continue;
          }

          const Eigen::Vector3d dir = diff / std::max(dist, 1.0e-6);
          const double step = std::min(dist, remain_dist);
          sim_pos += dir * step;
          sim_vel = dir * target_speed_;
          remain_dist -= step;
          moved = true;

          if (step >= dist - 1.0e-6)
          {
            ++sim_idx;
          }
        }

        if (!moved)
        {
          sim_pos += sim_vel * sim_step;
        }
      }

      return samples;
    }

    static double wrapAngle(double angle)
    {
      while (angle > M_PI)
      {
        angle -= 2.0 * M_PI;
      }
      while (angle < -M_PI)
      {
        angle += 2.0 * M_PI;
      }
      return angle;
    }

    bool lineOfSightBlocked(const Eigen::Vector3d &from,
                            const Eigen::Vector3d &to,
                            const double clearance) const
    {
      if (!grid_map_ || !mapWindowReady())
      {
        return false;
      }

      const Eigen::Vector3d ray = to - from;
      const double ray_len = ray.norm();
      if (ray_len < 1.0e-4)
      {
        return false;
      }

      const double res = std::max(grid_map_->getResolution(), 0.05);
      const int sample_num = std::max(4, static_cast<int>(std::ceil(ray_len / res)));
      for (int i = 1; i < sample_num; ++i)
      {
        const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
        const Eigen::Vector3d q = (1.0 - alpha) * from + alpha * to;
        if (grid_map_->esdfEnabled())
        {
          if (grid_map_->getDistance(q) < clearance)
          {
            return true;
          }
        }
        else if (grid_map_->getInflateOccupancy(q) != 0)
        {
          return true;
        }
      }
      return false;
    }

    void applyReferenceMode(std::vector<Eigen::Vector3d> &future)
    {
      if (reference_mode_ == "oracle_reference_mode")
      {
        return;
      }

      if (reference_mode_ == "odom_only_mode")
      {
        future.clear();
        return;
      }

      if (reference_mode_ != "noisy_reference_mode" || future.empty())
      {
        return;
      }

      const int lag_steps = std::max(0, static_cast<int>(std::round(reference_lag_sec_ / std::max(horizon_dt_, 0.05))));
      std::normal_distribution<double> pos_noise(0.0, std::max(0.0, reference_noise_std_));
      std::normal_distribution<double> vel_noise(0.0, std::max(0.0, reference_velocity_noise_std_));

      const std::vector<Eigen::Vector3d> oracle = future;
      for (std::size_t i = 0; i < future.size(); ++i)
      {
        const std::size_t lag_idx = (i < static_cast<std::size_t>(lag_steps)) ? 0 : (i - static_cast<std::size_t>(lag_steps));
        future[i] = oracle[lag_idx];

        Eigen::Vector3d local_vel = Eigen::Vector3d::Zero();
        if (lag_idx + 1 < oracle.size())
        {
          local_vel = (oracle[lag_idx + 1] - oracle[lag_idx]) / std::max(horizon_dt_, 0.05);
        }

        future[i].x() += pos_noise(rng_);
        future[i].y() += pos_noise(rng_);
        future[i].z() += 0.35 * pos_noise(rng_);
        future[i] += 0.15 * std::max(horizon_dt_, 0.05) *
                     Eigen::Vector3d(vel_noise(rng_), vel_noise(rng_), 0.35 * vel_noise(rng_));
      }
    }

    void updateMetrics(const bool reference_available)
    {
      if (!have_ego_odom_)
      {
        return;
      }

      metric_sample_count_ += 1.0;
      metric_reference_available_count_ += reference_available ? 1.0 : 0.0;

      const Eigen::Vector3d delta = target_pos_ - ego_pos_;
      const double planar_dist = delta.head<2>().norm();
      const bool distance_ok =
          planar_dist >= tracking_eval_distance_min_ &&
          planar_dist <= tracking_eval_distance_max_ &&
          std::abs(delta.z()) <= tracking_eval_height_tolerance_;
      metric_distance_violation_count_ += distance_ok ? 0.0 : 1.0;
      metric_los_blocked_count_ += lineOfSightBlocked(ego_pos_, target_pos_, tracking_eval_los_clearance_) ? 1.0 : 0.0;

      if (ego_vel_.head<2>().norm() > 0.2 && delta.head<2>().norm() > 0.2)
      {
        const double yaw_ego = std::atan2(ego_vel_.y(), ego_vel_.x());
        const double yaw_target = std::atan2(delta.y(), delta.x());
        metric_yaw_alignment_sum_ += std::abs(wrapAngle(yaw_target - yaw_ego));
        metric_yaw_alignment_count_ += 1.0;
      }
    }

    void publishMetrics()
    {
      const double inv_count = (metric_sample_count_ > 1.0e-6) ? 1.0 / metric_sample_count_ : 0.0;
      std_msgs::Float64 msg;

      msg.data = metric_distance_violation_count_ * inv_count;
      distance_violation_pub_.publish(msg);

      msg.data = metric_los_blocked_count_ * inv_count;
      los_blocked_pub_.publish(msg);

      msg.data = (metric_yaw_alignment_count_ > 1.0e-6) ? (metric_yaw_alignment_sum_ / metric_yaw_alignment_count_) : 0.0;
      yaw_alignment_pub_.publish(msg);

      msg.data = metric_reference_available_count_ * inv_count;
      reference_available_pub_.publish(msg);

      ROS_INFO_THROTTLE(1.0,
                        "TrackingTargetSim metrics: dist_violation=%.2f los_blocked=%.2f yaw_err=%.2f ref_avail=%.2f mode=%s",
                        metric_distance_violation_count_ * inv_count,
                        metric_los_blocked_count_ * inv_count,
                        (metric_yaw_alignment_count_ > 1.0e-6) ? (metric_yaw_alignment_sum_ / metric_yaw_alignment_count_) : 0.0,
                        metric_reference_available_count_ * inv_count,
                        reference_mode_.c_str());
    }

    void publishReferenceAndPath()
    {
      const ros::Time now = ros::Time::now();
      auto future = sampleFutureReference();
      applyReferenceMode(future);
      const bool reference_available = future.size() >= 2;

      nav_msgs::Path ref_msg;
      ref_msg.header.stamp = now;
      ref_msg.header.frame_id = frame_id_;
      if (reference_available)
      {
        for (std::size_t i = 0; i < future.size(); ++i)
        {
          geometry_msgs::PoseStamped ps;
          ps.header = ref_msg.header;
          ps.header.stamp = now + ros::Duration(static_cast<double>(i) * std::max(0.05, horizon_dt_));
          ps.pose.position.x = future[i].x();
          ps.pose.position.y = future[i].y();
          ps.pose.position.z = future[i].z();
          ps.pose.orientation.w = 1.0;
          ref_msg.poses.push_back(ps);
        }
        ref_pub_.publish(ref_msg);
      }

      nav_msgs::Path path_msg;
      path_msg.header = ref_msg.header;
      for (const auto &pt : path_)
      {
        geometry_msgs::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = pt.x();
        ps.pose.position.y = pt.y();
        ps.pose.position.z = pt.z();
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
      }
      path_pub_.publish(path_msg);

      if (publish_goal_with_id_ && goal_pub_)
      {
        quadrotor_msgs::GoalSet goal_msg;
        goal_msg.drone_id = static_cast<int16_t>(drone_id_);
        const Eigen::Vector3d goal = future.empty() ? target_pos_ : future.back();
        goal_msg.goal[0] = static_cast<float>(goal.x());
        goal_msg.goal[1] = static_cast<float>(goal.y());
        goal_msg.goal[2] = static_cast<float>(goal.z());
        goal_pub_.publish(goal_msg);
      }

      updateMetrics(reference_available);
      publishMetrics();
    }

    void simCallback(const ros::TimerEvent &)
    {
      if (!have_ego_odom_)
      {
        return;
      }

      if (!initialized_)
      {
        target_pos_ = ego_pos_ + init_offset_;
        target_pos_.z() = ego_pos_.z() + init_offset_.z();
        enforceSeparationFromEgo(target_pos_, min_init_separation_);
        target_pos_ = clampToMap(target_pos_);
        if (!isFree(target_pos_))
        {
          Eigen::Vector3d safe_init = target_pos_;
          if (findNearbyFreeGoal(target_pos_, safe_init))
          {
            target_pos_ = safe_init;
          }
        }
        enforceSeparationFromEgo(target_pos_, min_init_separation_);
        if (!isFree(target_pos_))
        {
          Eigen::Vector3d safe_init = target_pos_;
          if (findNearbyFreeGoal(target_pos_, safe_init))
          {
            target_pos_ = safe_init;
          }
        }
        path_.clear();
        path_.push_back(target_pos_);
        path_idx_ = 1;
        initialized_ = true;
        ROS_INFO("TrackingTargetSim initialized target at [%.2f %.2f %.2f], ego=[%.2f %.2f %.2f], sep=%.2f",
                 target_pos_.x(), target_pos_.y(), target_pos_.z(),
                 ego_pos_.x(), ego_pos_.y(), ego_pos_.z(),
                 (target_pos_ - ego_pos_).norm());
      }

      if (path_.size() < 2 || path_idx_ >= path_.size())
      {
        sampleNewGoalAndPath();
      }

      advanceAlongPath(std::max(0.005, sim_dt_));
      publishMarker();
      publishTargetOdom();
      publishReferenceAndPath();
    }

    void replanCallback(const ros::TimerEvent &)
    {
      if (!initialized_)
      {
        return;
      }

      if (use_external_goal_ && !have_external_goal_)
      {
        return;
      }

      const bool path_exhausted = (path_.size() < 2 || path_idx_ >= path_.size());
      const bool near_path_end =
          (!path_exhausted && (path_.back() - target_pos_).norm() < std::max(0.6, goal_ahead_distance_ * 0.2));

      if (path_exhausted || near_path_end)
      {
        sampleNewGoalAndPath();
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    GridMap::Ptr grid_map_;
    JPSAStar::Ptr jps_;

    ros::Publisher marker_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher ref_pub_;
    ros::Publisher path_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher distance_violation_pub_;
    ros::Publisher los_blocked_pub_;
    ros::Publisher yaw_alignment_pub_;
    ros::Publisher reference_available_pub_;
    ros::Subscriber ego_odom_sub_;
    ros::Subscriber external_goal_sub_;
    ros::Timer sim_timer_;
    ros::Timer replan_timer_;

    std::string frame_id_;
    std::string ego_odom_topic_;
    std::string reference_topic_;
    std::string target_odom_topic_;
    std::string target_marker_topic_;
    std::string target_path_topic_;
    std::string goal_topic_;
    std::string external_goal_topic_;
    std::string reference_mode_;

    int drone_id_{0};
    bool use_jps_{true};
    bool publish_goal_with_id_{false};
    bool use_external_goal_{true};
    bool lock_external_goal_height_{true};
    bool have_ego_odom_{false};
    bool initialized_{false};
    bool have_external_goal_{false};

    double sim_dt_{0.02};
    double replan_period_{0.5};
    double target_speed_{0.8};
    double goal_ahead_distance_{4.0};
    double goal_lateral_jitter_{1.2};
    double goal_vertical_jitter_{0.0};
    double goal_reach_tolerance_{0.25};
    double horizon_time_{4.0};
    double horizon_dt_{0.2};
    double reference_noise_std_{0.08};
    double reference_velocity_noise_std_{0.05};
    double reference_lag_sec_{0.20};
    double target_box_size_{0.45};
    double min_init_separation_{2.5};
    double external_goal_min_separation_{1.2};
    double tracking_eval_distance_min_{1.5};
    double tracking_eval_distance_max_{4.0};
    double tracking_eval_height_tolerance_{0.4};
    double tracking_eval_los_clearance_{0.15};
    double jps_timeout_{0.08};
    int jps_jump_max_cells_{6};
    int jps_near_obs_radius_{1};
    double metric_sample_count_{0.0};
    double metric_distance_violation_count_{0.0};
    double metric_los_blocked_count_{0.0};
    double metric_reference_available_count_{0.0};
    double metric_yaw_alignment_sum_{0.0};
    double metric_yaw_alignment_count_{0.0};

    Eigen::Vector3d init_offset_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ego_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ego_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_vel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d external_goal_{Eigen::Vector3d::Zero()};

    std::vector<Eigen::Vector3d> path_;
    std::size_t path_idx_{1};

    mutable std::mt19937 rng_;
    mutable std::uniform_real_distribution<double> uni_;
  };
} // namespace ego_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking_target_sim");
  ego_planner::TrackingTargetSim node;
  ros::spin();
  return 0;
}
