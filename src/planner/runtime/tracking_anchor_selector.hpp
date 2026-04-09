#ifndef PLANNER_RUNTIME_TRACKING_ANCHOR_SELECTOR_HPP_
#define PLANNER_RUNTIME_TRACKING_ANCHOR_SELECTOR_HPP_

#include <CostFunctionalManager/TrackingTypes.hpp>
#include <Eigen/Core>
#include <vector>

namespace ego_planner::runtime
{

class TrackingAnchorSelector
{
public:
  void configure(double tracking_d_min,
                 double tracking_d_max,
                 double side_angle_deg,
                 const Eigen::Vector3d &preferred_relative_offset = Eigen::Vector3d::Zero());

  void reset();

  bool buildAnchorReference(const cost_functional::TrackingReference &target_reference,
                            const Eigen::Vector3d &ego_pos,
                            const Eigen::Vector3d &ego_vel,
                            cost_functional::TrackingReference &planning_reference,
                            Eigen::Vector3d &terminal_anchor,
                            Eigen::Vector3d &terminal_vel);

private:
  Eigen::Vector2d chooseBaseDirection(const cost_functional::TrackingReference &target_reference,
                                      const Eigen::Vector3d &ego_pos) const;
  Eigen::Vector2d rotate2D(const Eigen::Vector2d &dir, double angle_rad) const;
  double scoreDirection(const Eigen::Vector2d &candidate_dir,
                        const Eigen::Vector2d &ego_to_target_dir,
                        const Eigen::Vector2d &target_vel_dir,
                        double current_dist) const;

private:
  double tracking_d_min_{1.5};
  double tracking_d_max_{4.0};
  double side_angle_deg_{20.0};
  bool use_preferred_relative_offset_{false};
  Eigen::Vector3d preferred_relative_offset_{Eigen::Vector3d::Zero()};
  bool have_prev_dir_{false};
  Eigen::Vector2d prev_dir_{Eigen::Vector2d::UnitX()};
};

} // namespace ego_planner::runtime

#endif // PLANNER_RUNTIME_TRACKING_ANCHOR_SELECTOR_HPP_
