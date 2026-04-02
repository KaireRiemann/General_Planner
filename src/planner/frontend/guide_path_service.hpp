#ifndef PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_
#define PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_

#include <Eigen/Core>

#include <vector>

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
  bool buildFromWaypoints(const std::vector<Eigen::Vector3d> &waypoints,
                          GuidePathArtifact &artifact) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_GUIDE_PATH_SERVICE_HPP_

