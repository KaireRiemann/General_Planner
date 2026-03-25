#ifndef _COST_FUNCTIONS_H_
#define _COST_FUNCTIONS_H_

#include "TrajectoryOptAdapters/EgoIntegralCostAdapter.hpp"
#include "TrajectoryOptAdapters/EgoVarianceSampleCostAdapter.hpp"
#include "TrajectoryOptComponents/LinearTimeCost.hpp"

namespace ego_planner
{
  using TimeCostFunction = traj_opt_components::LinearTimeCost;
  using IntegralCostFunction = traj_opt_adapters::EgoIntegralCostAdapter;
  using SampleCostFunction = traj_opt_adapters::EgoVarianceSampleCostAdapter;

} // namespace ego_planner
#endif
