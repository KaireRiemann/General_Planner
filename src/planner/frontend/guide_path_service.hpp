#ifndef PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_
#define PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_

#include <Eigen/Core>

#include <vector>

#include <core/planning_context.hpp>
#include <core/task_definition.hpp>
#include <core/task_spec.hpp>

namespace ego_planner::frontend
{

struct GuidePathArtifact
{
  std::vector<Eigen::Vector3d> points;
  std::vector<double> times;

  bool valid() const
  {
    return points.size() >= 2 && times.size() == points.size();
  }
};

class GuidePathService
{
public:
  bool searchStateToStateDensePath(const core::PlanningContext &context,
                                   const core::TaskDefinition &task_definition,
                                   std::vector<Eigen::Vector3d> &path) const;

  bool searchStateToStateDensePath(const core::PlanningContext &context,
                                   const core::TaskSpec &task,
                                   std::vector<Eigen::Vector3d> &path) const;

  bool buildStateToStateGuide(const core::PlanningContext &context,
                              const core::TaskDefinition &task_definition,
                              GuidePathArtifact &artifact) const;

  bool buildStateToStateGuide(const core::PlanningContext &context,
                              const core::TaskSpec &task,
                              GuidePathArtifact &artifact) const;

  bool buildFromWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                          GuidePathArtifact &artifact) const;

private:
  bool sanitizePoint(const core::PlanningContext &context,
                     const Eigen::Vector3d &raw,
                     Eigen::Vector3d &safe) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_
