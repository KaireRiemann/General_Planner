#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include "traj_utils/minco_types.hpp"

using std::vector;

namespace ego_planner
{
  typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  struct GlobalTrajData
  {
    MINCOTraj3D traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time. 
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    
    /* Global traj time. 
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;
  };

  struct LocalTrajData
  {
    MINCOTraj3D traj;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos;
    double des_clearance;
    std::vector<double> yaw_time;
    std::vector<double> yaw_ref;
    bool has_yaw_ref{false};

    double sampleYaw(double t_local) const
    {
      if (!has_yaw_ref || yaw_time.empty() || yaw_ref.empty() || yaw_time.size() != yaw_ref.size())
      {
        return 0.0;
      }

      if (t_local <= yaw_time.front())
      {
        return yaw_ref.front();
      }
      if (t_local >= yaw_time.back())
      {
        return yaw_ref.back();
      }

      const auto upper = std::upper_bound(yaw_time.begin(), yaw_time.end(), t_local);
      const std::size_t idx1 = static_cast<std::size_t>(std::distance(yaw_time.begin(), upper));
      const std::size_t idx0 = idx1 - 1;
      const double t0 = yaw_time[idx0];
      const double t1 = yaw_time[idx1];
      const double alpha = (t_local - t0) / std::max(1.0e-6, t1 - t0);
      return (1.0 - alpha) * yaw_ref[idx0] + alpha * yaw_ref[idx1];
    }
  };

  typedef std::vector<LocalTrajData> SwarmTrajData;

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    LocalTrajData local_traj;
    SwarmTrajData swarm_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}
    void setGlobalTraj(const MINCOTraj3D &trajectory, const double &world_time)
    {
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration(); 
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      local_traj.drone_id = -1;
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
    }

    void setLocalTraj(const MINCOTraj3D &trajectory, const double &world_time, const int drone_id = -1)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      
      local_traj.duration = trajectory.getTotalDuration(); 
      local_traj.start_pos = trajectory.evaluate(0.0, 0); 
      
      local_traj.start_time = world_time;
      local_traj.end_time = world_time + local_traj.duration;
      local_traj.traj = trajectory;
      local_traj.yaw_time.clear();
      local_traj.yaw_ref.clear();
      local_traj.has_yaw_ref = false;
    }

    void setLocalYawRef(const std::vector<double> &t_ref,
                        const std::vector<double> &yaw_ref)
    {
      local_traj.yaw_time = t_ref;
      local_traj.yaw_ref = yaw_ref;
      local_traj.has_yaw_ref =
          !local_traj.yaw_time.empty() &&
          local_traj.yaw_time.size() == local_traj.yaw_ref.size();
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;     // physical limits
    double polyTraj_piece_length;  // distance between adjacient B-spline control points
    double feasibility_tolerance_; // permitted ratio of vel/acc exceeding limits
    double planning_horizen_;
    bool use_multitopology_trajs;
    bool touch_goal;
    int drone_id; // single drone: drone_id <= -1, swarm: drone_id >= 0

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace ego_planner

#endif
