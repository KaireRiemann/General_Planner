#ifndef PLANNER_CORE_PLANNING_PROBLEM_HPP_
#define PLANNER_CORE_PLANNING_PROBLEM_HPP_

#include <functional>
#include <string>
#include <vector>
#include <cstdint>

#include <Eigen/Core>

#include <core/feasible_set_spec.hpp>
#include <core/planning_context.hpp>
#include <core/task_spec.hpp>

namespace ego_planner::core
{

struct PlanningSolution;
struct PlanningProblem;

using SolveCallback = std::function<bool(const PlanningProblem &, PlanningSolution &)>;

enum class RepresentationKind
{
  UNKNOWN = 0,
  MINCO
};

struct BoundaryConditionSpec
{
  bool valid{false};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct CompiledReferenceSpec
{
  // Generic geometric guide for transit/corridor construction.
  std::vector<Eigen::Vector3d> guide_path;
  std::vector<double> guide_times;

  // Time-varying task references.
  std::vector<double> t_ref;
  std::vector<Eigen::Vector3d> p_ref;
  std::vector<Eigen::Vector3d> v_ref;
};

struct SeedSpec
{
  enum class Kind
  {
    NONE = 0,
    WARM_START,
    GUIDE_PATH,
    CORRIDOR_AWARE
  };

  Kind kind{Kind::NONE};
  bool valid{false};
  bool corridor_aware{false};
  std::vector<Eigen::Vector3d> anchor_points;
  Eigen::VectorXd durations;
};

struct VariableLayoutSpec
{
  int piece_num{0};
  int inner_point_num{0};
  int boundary_derivative_num{0};
};

struct PhaseProblemSpec
{
  std::string name;
  bool terminal_is_set{false};
  bool terminal_is_manifold{false};
  std::vector<int> feasible_set_indices;
  uint32_t objective_mask{0U};
  uint32_t constraint_mask{0U};
};

struct PlanningProblem
{
  std::string problem_name{"planning_problem"};
  bool valid{false};

  PlanningContext context;
  TaskSpec task;

  std::vector<FeasibleSetSpec> feasible_sets;
  uint32_t objective_mask{0U};
  uint32_t constraint_mask{0U};

  // Compiled solver-facing IR. New pipeline should prefer these fields.
  RepresentationKind representation{RepresentationKind::UNKNOWN};
  BoundaryConditionSpec start_boundary;
  BoundaryConditionSpec terminal_boundary;
  CompiledReferenceSpec references;
  SeedSpec seed;
  VariableLayoutSpec variable_layout;
  std::vector<PhaseProblemSpec> phase_specs;
  bool prefer_legacy_fallback{true};

  SolveCallback solve_callback;
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PLANNING_PROBLEM_HPP_
