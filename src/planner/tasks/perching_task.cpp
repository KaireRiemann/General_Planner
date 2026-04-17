#include <tasks/perching_task.hpp>
#include <tasks/state_to_state_task.hpp>

#include <algorithm>

namespace
{

ego_planner::core::TaskDefinition buildPerchingTransitTemplate(const Eigen::Vector3d &start_pt,
                                                               const Eigen::Vector3d &start_vel,
                                                               const Eigen::Vector3d &start_acc,
                                                               const Eigen::Vector3d &approach_anchor_pt,
                                                               const Eigen::Vector3d &approach_anchor_vel,
                                                               const Eigen::Vector3d &approach_anchor_acc,
                                                               const bool force_plain,
                                                               const bool prefer_corridor,
                                                               const bool prefer_esdf)
{
  auto task = ego_planner::tasks::StateToStateTask::buildDefinition(start_pt,
                                                                    start_vel,
                                                                    start_acc,
                                                                    approach_anchor_pt,
                                                                    approach_anchor_vel,
                                                                    true,
                                                                    false,
                                                                    false,
                                                                    force_plain,
                                                                    prefer_corridor,
                                                                    prefer_esdf);
  task.task_name = "perching";
  task.type = ego_planner::core::TaskType::PERCHING;
  task.goal.state.acceleration = approach_anchor_acc;
  if (!task.phases.empty())
  {
    task.phases.front().goal.state.acceleration = approach_anchor_acc;
  }
  return task;
}

} // namespace

namespace ego_planner::tasks
{

core::TaskDefinition PerchingTask::buildDefinition(const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   const Eigen::Vector3d &approach_anchor_pt,
                                                   const Eigen::Vector3d &approach_anchor_vel,
                                                   const Eigen::Vector3d &approach_anchor_acc,
                                                   const Eigen::Vector3d &contact_pt,
                                                   const Eigen::Vector3d &contact_vel,
                                                   const Eigen::Vector3d &contact_acc,
                                                   const Eigen::Vector3d &plate_position_ref,
                                                   const Eigen::Vector3d &plate_velocity,
                                                   const double prediction_time,
                                                   const Eigen::Vector3d &landing_tangent_x,
                                                   const Eigen::Vector3d &landing_tangent_y,
                                                   const Eigen::Vector3d &landing_normal,
                                                   const Eigen::Vector2d &tangential_velocity_seed,
                                                   const double thrust_phase_seed,
                                                   const double robot_l,
                                                   const double v_plus,
                                                   const double terminal_thrust_nominal,
                                                   const double terminal_thrust_range,
                                                   const bool use_dynamics_terminal_accel,
                                                   const bool force_plain,
                                                   const bool prefer_corridor,
                                                   const bool prefer_esdf)
{
  // Perching is a transit-style task with extra landing/contact semantics:
  // start/space-model/runtime transit defaults come from the shared
  // state-to-state template, while terminal-manifold data stays perching-specific.
  core::TaskDefinition task = buildPerchingTransitTemplate(start_pt,
                                                           start_vel,
                                                           start_acc,
                                                           approach_anchor_pt,
                                                           approach_anchor_vel,
                                                           approach_anchor_acc,
                                                           force_plain,
                                                           prefer_corridor,
                                                           prefer_esdf);

  task.goal.semantic = core::GoalSemanticType::TERMINAL_MANIFOLD;
  task.goal.state.valid = true;
  task.goal.state.position = contact_pt;
  task.goal.state.velocity = contact_vel;
  task.goal.state.acceleration = contact_acc;
  task.goal.touch_goal = true;
  const Eigen::Vector3d safe_normal =
      landing_normal.norm() > 1.0e-6 ? landing_normal.normalized()
                                     : Eigen::Vector3d::UnitZ();
  Eigen::Vector3d safe_tangent_x =
      landing_tangent_x.norm() > 1.0e-6 ? landing_tangent_x.normalized()
                                        : Eigen::Vector3d::UnitX();
  Eigen::Vector3d safe_tangent_y =
      landing_tangent_y.norm() > 1.0e-6 ? landing_tangent_y.normalized()
                                        : safe_normal.cross(safe_tangent_x);
  if (safe_tangent_y.norm() < 1.0e-6)
  {
    safe_tangent_y = Eigen::Vector3d::UnitY();
  }
  safe_tangent_y.normalize();
  safe_tangent_x = safe_tangent_y.cross(safe_normal);
  if (safe_tangent_x.norm() < 1.0e-6)
  {
    safe_tangent_x = Eigen::Vector3d::UnitX();
  }
  safe_tangent_x.normalize();

  task.goal.manifold_params.resize(30);
  task.goal.manifold_params.segment<3>(0) = contact_pt;
  task.goal.manifold_params.segment<3>(3) = contact_vel;
  // The terminal manifold stores the plate state sampled at prediction_time
  // together with that reference time. Dynamic terminal mapping then evaluates
  // the true terminal state at optimized final time T using
  // Xi(T) = Xi(T_ref) + Xi_dot * (T - T_ref), which matches the flexible
  // terminal-adjustment semantics in adaptive perching.
  task.goal.manifold_params.segment<3>(6) = plate_position_ref;
  task.goal.manifold_params.segment<3>(9) = plate_velocity;
  task.goal.manifold_params.segment<3>(12) = safe_tangent_x;
  task.goal.manifold_params.segment<3>(15) = safe_tangent_y;
  task.goal.manifold_params.segment<3>(18) = safe_normal;
  task.goal.manifold_params(21) = robot_l;
  task.goal.manifold_params(22) = v_plus;
  task.goal.manifold_params(23) = terminal_thrust_nominal;
  task.goal.manifold_params(24) = terminal_thrust_range;
  task.goal.manifold_params(25) = tangential_velocity_seed.x();
  task.goal.manifold_params(26) = tangential_velocity_seed.y();
  task.goal.manifold_params(27) = thrust_phase_seed;
  task.goal.manifold_params(28) = use_dynamics_terminal_accel ? 1.0 : 0.0;
  task.goal.manifold_params(29) = std::max(0.0, prediction_time);

  task.runtime_policy.touch_goal = true;
  task.runtime_policy.enable_keep_current = false;
  task.runtime_policy.enable_successor_planning = false;

  task.phases.clear();
  core::PhaseDefinition approach;
  approach.name = "approach_anchor";
  approach.goal.semantic = core::GoalSemanticType::TERMINAL_SET;
  approach.goal.state.valid = true;
  approach.goal.state.position = approach_anchor_pt;
  approach.goal.state.velocity = approach_anchor_vel;
  approach.goal.state.acceleration = approach_anchor_acc;
  task.phases.push_back(approach);

  core::PhaseDefinition contact;
  contact.name = "contact_final";
  contact.goal = task.goal;
  task.phases.push_back(contact);

  return task;
}

core::TaskDefinition PerchingTask::buildDefinition(const Eigen::Vector3d &start_pt,
                                                   const Eigen::Vector3d &start_vel,
                                                   const Eigen::Vector3d &start_acc,
                                                   const Eigen::Vector3d &contact_pt,
                                                   const Eigen::Vector3d &contact_vel,
                                                   const bool force_plain)
{
  return buildDefinition(start_pt,
                         start_vel,
                         start_acc,
                         contact_pt - 0.4 * Eigen::Vector3d::UnitZ(),
                         contact_vel,
                         Eigen::Vector3d::Zero(),
                         contact_pt,
                         contact_vel,
                         Eigen::Vector3d::Zero(),
                         contact_pt,
                         Eigen::Vector3d::Zero(),
                         0.0,
                         Eigen::Vector3d::UnitX(),
                         Eigen::Vector3d::UnitY(),
                         Eigen::Vector3d::UnitZ(),
                         Eigen::Vector2d::Zero(),
                         0.0,
                         0.0,
                         0.0,
                         9.81,
                         0.0,
                         false,
                         force_plain,
                         false,
                         false);
}

core::TaskSpec PerchingTask::build(const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &approach_anchor_pt,
                                   const Eigen::Vector3d &approach_anchor_vel,
                                   const Eigen::Vector3d &approach_anchor_acc,
                                   const Eigen::Vector3d &contact_pt,
                                   const Eigen::Vector3d &contact_vel,
                                   const Eigen::Vector3d &contact_acc,
                                   const Eigen::Vector3d &plate_position_ref,
                                   const Eigen::Vector3d &plate_velocity,
                                   const double prediction_time,
                                   const Eigen::Vector3d &landing_tangent_x,
                                   const Eigen::Vector3d &landing_tangent_y,
                                   const Eigen::Vector3d &landing_normal,
                                   const Eigen::Vector2d &tangential_velocity_seed,
                                   const double thrust_phase_seed,
                                   const double robot_l,
                                   const double v_plus,
                                   const double terminal_thrust_nominal,
                                   const double terminal_thrust_range,
                                   const bool use_dynamics_terminal_accel,
                                   const bool force_plain,
                                   const bool prefer_corridor,
                                   const bool prefer_esdf)
{
  return buildDefinition(start_pt,
                         start_vel,
                         start_acc,
                         approach_anchor_pt,
                         approach_anchor_vel,
                         approach_anchor_acc,
                         contact_pt,
                         contact_vel,
                         contact_acc,
                         plate_position_ref,
                         plate_velocity,
                         prediction_time,
                         landing_tangent_x,
                         landing_tangent_y,
                         landing_normal,
                         tangential_velocity_seed,
                         thrust_phase_seed,
                         robot_l,
                         v_plus,
                         terminal_thrust_nominal,
                         terminal_thrust_range,
                         use_dynamics_terminal_accel,
                         force_plain,
                         prefer_corridor,
                         prefer_esdf)
      .toTaskSpec();
}

core::TaskSpec PerchingTask::build(const Eigen::Vector3d &start_pt,
                                   const Eigen::Vector3d &start_vel,
                                   const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &contact_pt,
                                   const Eigen::Vector3d &contact_vel,
                                   const bool force_plain)
{
  return build(start_pt,
               start_vel,
               start_acc,
               contact_pt - 0.4 * Eigen::Vector3d::UnitZ(),
               contact_vel,
               Eigen::Vector3d::Zero(),
               contact_pt,
               contact_vel,
               Eigen::Vector3d::Zero(),
               contact_pt,
               Eigen::Vector3d::Zero(),
               0.0,
               Eigen::Vector3d::UnitX(),
               Eigen::Vector3d::UnitY(),
               Eigen::Vector3d::UnitZ(),
               Eigen::Vector2d::Zero(),
               0.0,
               0.0,
               0.0,
               9.81,
               0.0,
               false,
               force_plain,
               false,
               false);
}

} // namespace ego_planner::tasks
