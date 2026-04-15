#ifndef PLANNER_FRONTEND_PERCHING_INIT_SERVICE_HPP_
#define PLANNER_FRONTEND_PERCHING_INIT_SERVICE_HPP_

#include <frontend/init_artifact.hpp>
#include <frontend/transit_init_service.hpp>

#include <core/planning_problem.hpp>

namespace ego_planner::frontend
{

struct PerchingDecodedContactSemantics
{
  bool valid{false};
  Eigen::Vector3d plate_position_ref{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_velocity{Eigen::Vector3d::Zero()};
  double reference_time{0.0};
  Eigen::Vector3d surface_x{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d surface_y{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d surface_z{Eigen::Vector3d::UnitZ()};
  double robot_l{0.0};
  double v_plus{0.0};
  double thrust_nominal{9.81};
  double thrust_range{0.0};
  bool use_dynamics_terminal_accel{false};
  Eigen::Vector2d nu_seed{Eigen::Vector2d::Zero()};
  double tau_f_seed{0.0};
  double pre_contact_distance{0.4};
};

struct PerchingPredictedContactState
{
  bool valid{false};
  double prediction_time{0.0};
  Eigen::Vector3d plate_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_acceleration{Eigen::Vector3d::Zero()};
};

struct PerchingPreContactAnchorState
{
  bool valid{false};
  double pre_contact_distance{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct PerchingInitArtifact
{
  bool valid{false};
  core::ActiveSpaceModel selected_mode{core::ActiveSpaceModel::PLAIN};
  std::string message;

  PerchingDecodedContactSemantics decoded_contact_semantics;
  PerchingPredictedContactState predicted_contact_state;
  PerchingPreContactAnchorState pre_contact_anchor_state;
  InitArtifact transit_init;
};

// PerchingInitService keeps perching-specific terminal/contact semantics out of
// the shared transit initializer. It decodes the moving-contact manifold,
// predicts the real contact state, builds a pre-contact anchor, then reuses the
// shared TransitInitService only for the free-space approach segment.
class PerchingInitService
{
public:
  PerchingInitService() = default;

  bool decodeContactSemantics(const core::PlanningProblem &problem,
                              PerchingDecodedContactSemantics &semantics,
                              std::string *reason = nullptr) const;

  bool initialize(const TransitInitRuntimeConfig &config,
                  const core::PlanningProblem &problem,
                  PerchingInitArtifact &artifact) const;
};

} // namespace ego_planner::frontend

#endif // PLANNER_FRONTEND_PERCHING_INIT_SERVICE_HPP_
