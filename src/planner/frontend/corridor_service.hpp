#ifndef PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_
#define PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_

#include <core/feasible_set_spec.hpp>
#include <frontend/guide_path_service.hpp>
#include <core/planning_context.hpp>

namespace ego_planner::frontend
{

class CorridorService
{
public:
  bool buildFromGuidePath(const core::PlanningContext &context,
                          const GuidePathArtifact &artifact,
                          core::FeasibleSetSpec &set_spec) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_
