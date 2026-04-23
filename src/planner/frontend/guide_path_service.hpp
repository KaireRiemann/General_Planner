#ifndef PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_
#define PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_

#include <Eigen/Core>

#include <vector>

#include <core/planning_context.hpp>
#include <core/task_definition.hpp>
#include <core/task_spec.hpp>

namespace ego_planner::frontend
{

struct GuidePathRuntimeConfig
{
  GridMap::Ptr grid_map;
  JPSAStar *jps_astar{nullptr};

  double poly_piece_length{0.2};
  double guide_min_clearance{0.35};
  int guide_sparse_min_inner{2};
  int guide_sparse_max_inner{5};
  double guide_turn_angle_deg{25.0};
  double sfc_range{0.8};
};

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
  // Solver-compatible frontend entry points. These preserve the historical
  // state-to-state initializer behavior while moving guide construction out of
  // the solver layer.
  bool sanitizeLocalTarget(const GuidePathRuntimeConfig &config,
                           const Eigen::Vector3d &raw_target,
                           Eigen::Vector3d &safe_target) const;

  bool prepareLocalAStarPath(const GuidePathRuntimeConfig &config,
                             const Eigen::Vector3d &start_pt,
                             const Eigen::Vector3d &goal_pt,
                             std::vector<Eigen::Vector3d> &dense_path,
                             Eigen::Vector3d &safe_goal) const;

  bool sparsifyGuidePath(const GuidePathRuntimeConfig &config,
                         const std::vector<Eigen::Vector3d> &dense_path,
                         std::vector<Eigen::Vector3d> &sparse_path) const;

  bool pathHasClearance(const GuidePathRuntimeConfig &config,
                        const std::vector<Eigen::Vector3d> &path,
                        double min_clearance = -1.0) const;

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

  bool buildTrackingGuideFromWaypoints(const core::PlanningContext &context,
                                       const std::vector<Eigen::Vector3d> &waypoints,
                                       double connect_dist,
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
