#ifndef PLANNER_RUNTIME_TRACKING_REFERENCE_PROVIDER_HPP_
#define PLANNER_RUNTIME_TRACKING_REFERENCE_PROVIDER_HPP_

#include <CostFunctionalManager/TrackingTypes.hpp>
#include <Eigen/Core>
#include <nav_msgs/Path.h>

namespace ego_planner::runtime
{

class TrackingReferenceProvider
{
public:
  void configure(double horizon_sec, double sample_dt, double max_speed);

  bool buildFromPath(const nav_msgs::Path &msg,
                     double fallback_dt,
                     cost_functional::TrackingReference &reference) const;

  bool buildFromTargetOdom(const Eigen::Vector3d &target_pos,
                           const Eigen::Vector3d &target_vel,
                           cost_functional::TrackingReference &reference) const;

  bool normalize(const cost_functional::TrackingReference &raw,
                 cost_functional::TrackingReference &normalized) const;

private:
  bool trimAndNormalize(const cost_functional::TrackingReference &raw,
                        cost_functional::TrackingReference &normalized) const;

private:
  double horizon_sec_{4.0};
  double sample_dt_{0.2};
  double max_speed_{2.0};
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_TRACKING_REFERENCE_PROVIDER_HPP_
