#include <compiler/problem_compiler.hpp>
#include <core/feasible_set_spec.hpp>
#include <core/planning_problem.hpp>
#include <frontend/perching_init_service.hpp>
#include <optimization/problem_adapter.hpp>
#include <runtime/perching_target_provider.hpp>
#include <tasks/task_factory.hpp>

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <traj_utils/plan_container.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

namespace
{

class DummyProblemAdapter final : public ego_planner::optimization::ProblemAdapter
{
public:
  bool solveCompatibility(const ego_planner::core::PlanningProblem &,
                          ego_planner::core::PlanningSolution &) override
  {
    return true;
  }
};

const char *activeSpaceModelString(const ego_planner::core::ActiveSpaceModel mode)
{
  switch (mode)
  {
  case ego_planner::core::ActiveSpaceModel::ESDF:
    return "ESDF";
  case ego_planner::core::ActiveSpaceModel::CORRIDOR:
    return "CORRIDOR";
  case ego_planner::core::ActiveSpaceModel::VISIBLE_REGION:
    return "VISIBLE_REGION";
  case ego_planner::core::ActiveSpaceModel::TERMINAL_MANIFOLD:
    return "TERMINAL_MANIFOLD";
  case ego_planner::core::ActiveSpaceModel::PLAIN:
  default:
    return "PLAIN";
  }
}

std::string vecToString(const Eigen::Vector3d &vec)
{
  std::ostringstream oss;
  oss << "[" << std::fixed << std::setprecision(3)
      << vec.x() << ", " << vec.y() << ", " << vec.z() << "]";
  return oss.str();
}

void printDivider(const std::string &title)
{
  std::cout << "\n=== " << title << " ===" << std::endl;
}

} // namespace

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  ros::Time::init();
  setenv("PERCHING_ONLY_DEBUG", "1", 1);

  const Eigen::Vector3d ego_position(0.0, 0.0, 1.0);
  const Eigen::Vector3d ego_velocity(0.0, 0.0, 0.0);
  const Eigen::Vector3d ego_acceleration(0.0, 0.0, 0.0);

  ego_planner::runtime::PerchingTargetProvider provider;
  provider.configure(0.25,
                     0.30,
                     0.35,
                     0.50,
                     2.00,
                     10.00,
                     0.40,
                     true,
                     Eigen::Quaterniond::Identity(),
                     false);

  nav_msgs::Odometry target_odom;
  target_odom.pose.pose.position.x = 1.75;
  target_odom.pose.pose.position.y = -0.30;
  target_odom.pose.pose.position.z = 1.80;
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom.twist.twist.linear.x = 0.45;
  target_odom.twist.twist.linear.y = -0.05;
  target_odom.twist.twist.linear.z = 0.10;
  provider.updateFromOdometry(target_odom);
  provider.setTerminalWarmStartHint(Eigen::Vector2d(0.08, -0.04), 0.25);

  ego_planner::runtime::PerchingTerminalState terminal;
  if (!provider.buildTerminalState(ego_position, 2.0, terminal))
  {
    std::cerr << "perching_debug_probe: failed to build perching terminal" << std::endl;
    return 1;
  }

  ego_planner::compiler::ProblemCompiler compiler;
  DummyProblemAdapter adapter;
  compiler.setProblemAdapter(&adapter);

  ego_planner::core::PlanningContext context;
  context.allow_warm_start = false;
  context.odom_pos = ego_position;
  context.odom_vel = ego_velocity;
  context.global_goal = terminal.terminal_position;
  context.local_target = terminal.approach_anchor_position;
  context.local_target_vel = terminal.approach_anchor_velocity;

  const auto task_definition = ego_planner::tasks::TaskFactory::makePerchingDefinition(
      ego_position,
      ego_velocity,
      ego_acceleration,
      terminal.approach_anchor_position,
      terminal.approach_anchor_velocity,
      terminal.approach_anchor_acceleration,
      terminal.terminal_position,
      terminal.terminal_velocity,
      terminal.terminal_acceleration,
      terminal.plate_position_now,
      terminal.plate_velocity,
      terminal.prediction_time,
      terminal.landing_tangent_x,
      terminal.landing_tangent_y,
      terminal.landing_normal,
      terminal.tangential_velocity_seed,
      terminal.thrust_phase_seed,
      0.25,
      0.30,
      terminal.terminal_thrust_nominal,
      terminal.terminal_thrust_range,
      terminal.use_dynamics_terminal_accel,
      true,
      false,
      false);

  ego_planner::core::PlanningProblem problem;
  if (!compiler.compile(context, task_definition, problem))
  {
    std::cerr << "perching_debug_probe: compiler failed to lower perching task" << std::endl;
    return 1;
  }

  const Eigen::Vector3d overshoot_point =
      terminal.approach_anchor_position + 0.20 * terminal.landing_normal;
  problem.references.guide_path = {
      ego_position,
      0.5 * (ego_position + terminal.approach_anchor_position),
      overshoot_point,
      terminal.terminal_position};
  problem.references.guide_times = {0.0, 0.7, 1.2, 1.6};

  ego_planner::core::FeasibleSetSpec debug_feasible_set;
  debug_feasible_set.enabled = true;
  debug_feasible_set.type = ego_planner::core::FeasibleSetType::CORRIDOR_POLYTOPE;
  debug_feasible_set.label = "perching_debug_preserved_set";
  problem.feasible_sets.push_back(debug_feasible_set);

  problem.seed.valid = true;
  problem.seed.kind = ego_planner::core::SeedSpec::Kind::GUIDE_PATH_INIT;
  problem.seed.anchor_points = problem.references.guide_path;
  problem.variable_layout.piece_num =
      std::max(0, static_cast<int>(problem.seed.anchor_points.size()) - 1);
  problem.variable_layout.inner_point_num =
      std::max(0, static_cast<int>(problem.seed.anchor_points.size()) - 2);

  ego_planner::PlanParameters plan_params;
  plan_params.max_vel_ = 2.0;
  plan_params.max_acc_ = 6.0;
  plan_params.polyTraj_piece_length = 0.8;
  plan_params.feasibility_tolerance_ = 0.05;
  plan_params.planning_horizen_ = 8.0;
  plan_params.use_multitopology_trajs = false;
  plan_params.touch_goal = true;
  plan_params.drone_id = 0;

  ego_planner::TrajContainer traj_container;
  int continuous_failures = 0;

  ego_planner::frontend::TransitInitRuntimeConfig init_config;
  init_config.plan_params = &plan_params;
  init_config.traj_container = &traj_container;
  init_config.continuous_failures_count = &continuous_failures;

  ego_planner::frontend::PerchingInitArtifact init_artifact;
  const bool init_ok =
      ego_planner::frontend::PerchingInitService{}.initialize(init_config, problem, init_artifact);

  std::cout << std::boolalpha;
  printDivider("Perching Provider Terminal");
  std::cout << "terminal_valid: " << terminal.valid << "\n";
  std::cout << "terminal_position: " << vecToString(terminal.terminal_position) << "\n";
  std::cout << "terminal_velocity: " << vecToString(terminal.terminal_velocity) << "\n";
  std::cout << "approach_anchor_position: " << vecToString(terminal.approach_anchor_position) << "\n";
  std::cout << "approach_anchor_velocity: " << vecToString(terminal.approach_anchor_velocity) << "\n";
  std::cout << "approach_anchor_acceleration: " << vecToString(terminal.approach_anchor_acceleration) << "\n";
  std::cout << "plate_position_ref: " << vecToString(terminal.plate_position) << "\n";
  std::cout << "plate_velocity: " << vecToString(terminal.plate_velocity) << "\n";

  printDivider("Compiled Problem");
  std::cout << "active_space_model: " << activeSpaceModelString(problem.active_space_model) << "\n";
  std::cout << "phase_count: " << problem.phase_specs.size() << "\n";
  std::cout << "phase0_name: "
            << (problem.phase_specs.empty() ? std::string("n/a") : problem.phase_specs.front().name) << "\n";
  std::cout << "phase1_name: "
            << (problem.phase_specs.size() > 1 ? problem.phase_specs[1].name : std::string("n/a")) << "\n";
  std::cout << "guide_pts_before_init: " << problem.references.guide_path.size() << "\n";
  std::cout << "guide_times_before_init: " << problem.references.guide_times.size() << "\n";
  std::cout << "feasible_sets_before_init: " << problem.feasible_sets.size() << "\n";
  std::cout << "seed_pts_before_init: " << problem.seed.anchor_points.size() << "\n";

  printDivider("Perching Init");
  std::cout << "init_ok: " << init_ok << "\n";
  std::cout << "init_message: " << init_artifact.message << "\n";
  std::cout << "selected_active_space_model: "
            << activeSpaceModelString(init_artifact.selected_mode) << "\n";
  std::cout << "approach_anchor_source: " << init_artifact.approach_anchor_source << "\n";
  std::cout << "final_manifold_source: " << init_artifact.final_manifold_source << "\n";
  std::cout << "predicted_contact_position: "
            << vecToString(init_artifact.predicted_contact_state.contact_position) << "\n";
  std::cout << "predicted_contact_velocity: "
            << vecToString(init_artifact.predicted_contact_state.contact_velocity) << "\n";
  std::cout << "predicted_contact_acceleration: "
            << vecToString(init_artifact.predicted_contact_state.contact_acceleration) << "\n";
  std::cout << "init_approach_anchor_position: "
            << vecToString(init_artifact.pre_contact_anchor_state.position) << "\n";
  std::cout << "init_approach_anchor_velocity: "
            << vecToString(init_artifact.pre_contact_anchor_state.velocity) << "\n";
  std::cout << "init_approach_anchor_acceleration: "
            << vecToString(init_artifact.pre_contact_anchor_state.acceleration) << "\n";
  std::cout << "guide_preserved: " << init_artifact.handoff_debug.guide_path_preserved
            << " (" << init_artifact.handoff_debug.guide_points_before
            << " -> " << init_artifact.handoff_debug.guide_points_after << ")\n";
  std::cout << "feasible_sets_preserved: " << init_artifact.handoff_debug.feasible_sets_preserved
            << " (" << init_artifact.handoff_debug.feasible_sets_before
            << " -> " << init_artifact.handoff_debug.feasible_sets_after << ")\n";
  std::cout << "seed_preserved: " << init_artifact.handoff_debug.seed_hint_preserved
            << " (" << init_artifact.handoff_debug.seed_anchor_points_before
            << " -> " << init_artifact.handoff_debug.seed_anchor_points_after << ")\n";
  std::cout << "guide_trimmed_to_anchor: " << init_artifact.handoff_debug.guide_path_trimmed_to_anchor << "\n";
  std::cout << "seed_trimmed_to_anchor: " << init_artifact.handoff_debug.seed_trimmed_to_anchor << "\n";

  printDivider("Planner-Engine-Like Solve Inputs");
  std::cout << "transit_source: " << init_artifact.transit_init.source << "\n";
  std::cout << "transit_guide_pts: " << init_artifact.transit_init.guide_path.size() << "\n";
  std::cout << "transit_inner_points: " << init_artifact.transit_init.inner_points.cols() << "\n";
  std::cout << "transit_duration_pieces: " << init_artifact.transit_init.durations.size() << "\n";
  std::cout << "corridor_polys: " << init_artifact.transit_init.corridor_hpolys.size() << "\n";
  std::cout << "plate_ref_for_mapping: "
            << vecToString(init_artifact.decoded_contact_semantics.plate_position_ref) << "\n";
  std::cout << "plate_velocity_for_mapping: "
            << vecToString(init_artifact.decoded_contact_semantics.plate_velocity) << "\n";
  std::cout << "nu_seed_for_mapping: ["
            << init_artifact.decoded_contact_semantics.nu_seed.x() << ", "
            << init_artifact.decoded_contact_semantics.nu_seed.y() << "]\n";
  std::cout << "tau_f_seed_for_mapping: "
            << init_artifact.decoded_contact_semantics.tau_f_seed << "\n";

  return init_ok ? 0 : 1;
}
