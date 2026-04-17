#include <compiler/problem_compiler.hpp>
#include <core/planning_problem.hpp>
#include <core/planning_solution.hpp>
#include <optimization/problem_adapter.hpp>
#include <runtime/perching_target_provider.hpp>
#include <tasks/task_factory.hpp>

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <iostream>
#include <string>

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

bool approxScalar(const double a, const double b, const double tol = 1.0e-6)
{
  return std::abs(a - b) <= tol;
}

bool approxVector(const Eigen::Vector3d &a,
                  const Eigen::Vector3d &b,
                  const double tol = 1.0e-6)
{
  return (a - b).norm() <= tol;
}

void require(const bool condition, const std::string &message)
{
  if (!condition)
  {
    std::cerr << "[perching_semantics_regression] " << message << std::endl;
    std::exit(1);
  }
}

ego_planner::runtime::PerchingTerminalState buildReferenceTerminal()
{
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

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = -0.5;
  odom.pose.pose.position.z = 2.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 0.4;
  odom.twist.twist.linear.y = -0.1;
  odom.twist.twist.linear.z = 0.2;
  provider.updateFromOdometry(odom);

  ego_planner::runtime::PerchingTerminalState terminal;
  require(provider.buildTerminalStateAtPrediction(1.0, terminal),
          "provider failed to build terminal state at fixed prediction time");
  return terminal;
}

void runProviderWarmStartRegression()
{
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

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = -0.5;
  odom.pose.pose.position.z = 2.0;
  odom.pose.pose.orientation.w = 1.0;
  provider.updateFromOdometry(odom);
  provider.setTerminalWarmStartHint(Eigen::Vector2d(0.12, -0.08), 0.33);

  ego_planner::runtime::PerchingTerminalState terminal;
  require(provider.buildTerminalStateAtPrediction(1.0, terminal),
          "provider failed to build terminal state with warm-start hint");
  require(approxScalar(terminal.tangential_velocity_seed.x(), 0.12),
          "provider lost tangential warm-start x");
  require(approxScalar(terminal.tangential_velocity_seed.y(), -0.08),
          "provider lost tangential warm-start y");
  require(approxScalar(terminal.thrust_phase_seed, 0.33),
          "provider lost thrust-phase warm-start");
}

void runProviderRegression()
{
  const auto terminal = buildReferenceTerminal();
  const Eigen::Vector3d expected_normal = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d expected_plate_position(1.4, -0.6, 2.2);
  const Eigen::Vector3d expected_contact = expected_plate_position + 0.25 * expected_normal;
  const double expected_approach_distance = 0.45;
  const Eigen::Vector3d expected_anchor =
      expected_contact - expected_approach_distance * expected_normal;
  const Eigen::Vector3d expected_terminal_velocity(0.4, -0.1, -0.1);
  const Eigen::Vector3d expected_anchor_velocity(0.4, -0.1, 0.095);
  const Eigen::Vector3d expected_terminal_acceleration(0.0, 0.0, 0.19);

  require(terminal.valid, "terminal bundle should be valid");
  require(approxScalar(terminal.prediction_time, 1.0), "prediction time mismatch");
  require(approxScalar(terminal.approach_distance, expected_approach_distance),
          "approach distance mismatch");
  require(approxVector(terminal.plate_position, expected_plate_position),
          "plate prediction mismatch");
  require(approxVector(terminal.terminal_position, expected_contact),
          "contact point mismatch");
  require(approxVector(terminal.approach_anchor_position, expected_anchor),
          "approach anchor mismatch");
  require(approxVector(terminal.terminal_velocity, expected_terminal_velocity),
          "terminal velocity mismatch");
  require(approxVector(terminal.approach_anchor_velocity, expected_anchor_velocity),
          "approach anchor velocity mismatch");
  require(approxVector(terminal.approach_anchor_acceleration, Eigen::Vector3d::Zero()),
          "approach anchor acceleration should be zero for the approach stage");
  require(approxVector(terminal.terminal_acceleration, expected_terminal_acceleration, 1.0e-5),
          "terminal acceleration mismatch");
}

void runCompilerRegression()
{
  const auto terminal = buildReferenceTerminal();

  ego_planner::compiler::ProblemCompiler compiler;
  DummyProblemAdapter adapter;
  compiler.setProblemAdapter(&adapter);

  ego_planner::core::PlanningContext context;
  context.allow_warm_start = false;
  context.odom_pos = Eigen::Vector3d::Zero();
  context.odom_vel = Eigen::Vector3d::Zero();
  context.global_goal = terminal.terminal_position;
  context.local_target = terminal.approach_anchor_position;
  context.local_target_vel = terminal.approach_anchor_velocity;

  const auto task_definition = ego_planner::tasks::TaskFactory::makePerchingDefinition(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      terminal.approach_anchor_position,
      terminal.approach_anchor_velocity,
      terminal.approach_anchor_acceleration,
      terminal.terminal_position,
      terminal.terminal_velocity,
      terminal.terminal_acceleration,
      terminal.plate_position,
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
  require(compiler.compile(context, task_definition, problem),
          "problem compiler rejected valid perching definition");
  require(problem.valid, "compiled problem should be valid with dummy adapter");
  require(problem.task_semantics.perching.valid, "perching semantic artifact should be valid");
  require(problem.task_semantics.perching.approach_anchor_state.valid,
          "compiled perching artifact lost approach anchor state");
  require(approxVector(problem.task_semantics.perching.approach_anchor_state.position,
                       terminal.approach_anchor_position),
          "compiled approach anchor position mismatch");
  require(approxVector(problem.task_semantics.perching.contact_state.position,
                       terminal.terminal_position),
          "compiled contact state mismatch");
  require(approxScalar(problem.task_semantics.perching.approach_distance,
                       terminal.approach_distance),
          "compiled approach distance mismatch");
  require(problem.phase_specs.size() == 2, "perching task should compile two explicit phases");
  require(problem.phase_specs.front().terminal_is_set,
          "phase-1 should remain a terminal-set approach anchor");
  require(problem.phase_specs.back().terminal_is_manifold,
          "phase-2 should remain the final contact manifold");
  require(problem.phase_specs.front().goal.isTerminalSet(),
          "phase-1 goal should lower into solver-facing IR");
  require(problem.phase_specs.front().has_cached_goal_state,
          "phase-1 should cache the approach anchor state");
  require(approxVector(problem.phase_specs.front().cached_goal_state.position,
                       terminal.approach_anchor_position),
          "phase-1 cached approach anchor mismatch");
  require(problem.phase_specs.back().goal.isTerminalManifold(),
          "phase-2 goal should lower into solver-facing IR");
  require(problem.phase_specs.back().has_cached_manifold_params,
          "phase-2 should cache terminal manifold parameters");
  require(problem.phase_specs.back().cached_manifold_params.size() == 30,
          "phase-2 cached manifold parameters should preserve perching semantics");
}

} // namespace

int main()
{
  runProviderRegression();
  runProviderWarmStartRegression();
  runCompilerRegression();
  std::cout << "perching semantics regression passed" << std::endl;
  return 0;
}
