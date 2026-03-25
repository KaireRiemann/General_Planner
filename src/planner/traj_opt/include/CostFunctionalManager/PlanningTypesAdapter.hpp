#ifndef PLANNING_TYPES_ADAPTER_HPP
#define PLANNING_TYPES_ADAPTER_HPP

#include "optimizer/traj_types.h"
#include "plan_env/grid_map.h"

namespace cost_functional
{
    struct PlanningTypesAdapter
    {
        using Vec3 = ego_planner::Vec3;
        using ConstraintPoints = ego_planner::ConstraintPoints;
        using SwarmTrajData = ego_planner::SwarmTrajData;
        using GridMapPtr = GridMap::Ptr;
    };
} // namespace traj_opt_adapters

#endif
