#ifndef _TRAJ_TYPES_H_
#define _TRAJ_TYPES_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <traj_utils/minco_types.hpp>
#include <traj_utils/plan_container.hpp>
#include "MINCOTrajectory/MINCOOptimizer.hpp"
#include "TemporalMap/IdentityTimeMap.hpp"
#include "TemporalMap/QuadInvTimeMap.hpp"
#include "SpatialMap/IdentityMap.hpp"
#include "SpatialMap/PolytopeSpatialMap.hpp"

namespace ego_planner
{
  // =====================================================
  //  Type aliases for the MINCO-based trajectory system
  // =====================================================
  constexpr int TRAJ_DIM = MINCO_TRAJ_DIM;
  constexpr int MINCO_S = MINCO_TRAJ_S;
  constexpr int MINCO_ORDER = MINCO_TRAJ_ORDER;
  constexpr int SNAP_S = SNAP_TRAJ_S;
  constexpr int SNAP_ORDER = SNAP_TRAJ_ORDER;
  using MINCOOpt = minco::MINCOOptimizer<TRAJ_DIM, MINCO_S, temporal_map::QuadInvTimeMap, spatial_map::IdentitySpatialMap<TRAJ_DIM>>;
  using ESDFMINCOOpt = MINCOOpt;
  using CorridorMINCOOpt = minco::MINCOOptimizer<TRAJ_DIM, MINCO_S, temporal_map::QuadInvTimeMap, spatial_map::PolytopeSpatialMap>;
  using SnapOpt = minco::MINCOOptimizer<TRAJ_DIM, SNAP_S, temporal_map::QuadInvTimeMap, spatial_map::IdentitySpatialMap<TRAJ_DIM>>;
  using ESDFSnapOpt = SnapOpt;
  using CorridorSnapOpt = minco::MINCOOptimizer<TRAJ_DIM, SNAP_S, temporal_map::QuadInvTimeMap, spatial_map::PolytopeSpatialMap>;
  using MINCOTraj = MINCOTraj3D;
  using SnapTraj = SnapTraj3D;
  using Vec3 = Eigen::Vector3d;
  using WaypointsMat = Eigen::Matrix<double, Eigen::Dynamic, TRAJ_DIM>;
  

  // =====================================================
  //  ConstraintPoints: deformation points for obstacle avoidance
  // =====================================================
  class ConstraintPoints
  {
  public:
    int cp_size{0}; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point;
    std::vector<std::vector<Eigen::Vector3d>> direction;
    std::vector<bool> flag_temp;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;
      base_point.clear();
      direction.clear();
      flag_temp.clear();
      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }
      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

} // namespace ego_planner
#endif
