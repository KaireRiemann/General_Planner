#ifndef PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_
#define PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_

#include <core/feasible_set_spec.hpp>
#include <frontend/guide_path_service.hpp>

namespace ego_planner::frontend
{

class CorridorService
{
public:
  bool buildFromGuidePath(const GuidePathArtifact &artifact,
                          core::FeasibleSetSpec &set_spec) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_CORRIDOR_SERVICE_HPP_

