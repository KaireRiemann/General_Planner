#ifndef _MINCO_TYPES_HPP_
#define _MINCO_TYPES_HPP_

#include "MINCOTrajectory/MINCOTrajectory.hpp"

namespace ego_planner
{

constexpr int MINCO_TRAJ_DIM = 3;
constexpr int JERK_TRAJ_S = 3;
constexpr int JERK_TRAJ_ORDER = 2 * JERK_TRAJ_S - 1;
constexpr int YAW_TRAJ_DIM = 1;
constexpr int YAW_TRAJ_S = 2;
constexpr int YAW_TRAJ_ORDER = 2 * YAW_TRAJ_S - 1;
constexpr int SNAP_TRAJ_S = 4;
constexpr int SNAP_TRAJ_ORDER = 2 * SNAP_TRAJ_S - 1;

using JerkTraj3D = minco::MINCOTrajectory<MINCO_TRAJ_DIM, JERK_TRAJ_S>;
using JerkBoundaryState3D = typename JerkTraj3D::BoundaryState;
using SnapTraj3D = minco::MINCOTrajectory<MINCO_TRAJ_DIM, SNAP_TRAJ_S>;
using SnapBoundaryState3D = typename SnapTraj3D::BoundaryState;
using YawTraj1D = minco::MINCOTrajectory<YAW_TRAJ_DIM, YAW_TRAJ_S>;
using YawBoundaryState1D = typename YawTraj1D::BoundaryState;

// Backward-compatible names for the existing minimum-jerk trajectory path.
// New code should prefer Jerk* for S=3 and Snap* for S=4.
constexpr int MINCO_TRAJ_S = JERK_TRAJ_S;
constexpr int MINCO_TRAJ_ORDER = JERK_TRAJ_ORDER;
using MINCOTraj3D = JerkTraj3D;
using MINCOBoundaryState3D = JerkBoundaryState3D;

} // namespace ego_planner

#endif
