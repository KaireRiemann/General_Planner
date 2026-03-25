#pragma once

#include "optimizer/traj_types.h"

#include <plan_env/grid_map.h>

namespace traj_opt_adapters
{
struct EgoPlanningTypesAdapter
{
    using Vec3 = ego_planner::Vec3;
    using ConstraintPoints = ego_planner::ConstraintPoints;
    using SwarmTrajData = ego_planner::SwarmTrajData;
    using GridMapPtr = GridMap::Ptr;
};
} // namespace traj_opt_adapters
