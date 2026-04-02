#ifndef PLANNER_CORE_PHASE_SPEC_HPP_
#define PLANNER_CORE_PHASE_SPEC_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

#include <core/feasible_set_spec.hpp>

namespace ego_planner::core
{

enum class TerminalSpecType
{
  FIXED_STATE = 0,
  TERMINAL_SET,
  TERMINAL_MANIFOLD
};

struct PhaseSpec
{
  std::string name{"phase"};
  double t_start{0.0};
  double t_end{0.0};

  TerminalSpecType terminal_type{TerminalSpecType::FIXED_STATE};
  Eigen::Vector3d terminal_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d terminal_velocity{Eigen::Vector3d::Zero()};
  Eigen::VectorXd terminal_manifold_params;

  std::vector<FeasibleSetSpec> feasible_sets;
  uint32_t active_objective_mask{0U};
  uint32_t active_constraint_mask{0U};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_PHASE_SPEC_HPP_

