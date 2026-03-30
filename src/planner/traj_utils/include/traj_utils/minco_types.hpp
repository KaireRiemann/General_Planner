#ifndef _MINCO_TYPES_HPP_
#define _MINCO_TYPES_HPP_

#include "MINCOTrajectory/MINCOTrajectory.hpp"

namespace ego_planner
{

constexpr int MINCO_TRAJ_DIM = 3;
constexpr int MINCO_TRAJ_S = 3;
constexpr int MINCO_TRAJ_ORDER = 2 * MINCO_TRAJ_S - 1;

using MINCOTraj3D = minco::MINCOTrajectory<MINCO_TRAJ_DIM, MINCO_TRAJ_S>;
using MINCOBoundaryState3D = typename MINCOTraj3D::BoundaryState;

} // namespace ego_planner

#endif
