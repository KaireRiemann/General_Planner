#ifndef PLANNER_CORE_TASK_SPEC_HPP_
#define PLANNER_CORE_TASK_SPEC_HPP_

#include <Eigen/Core>

#include <string>
#include <vector>

#include <CostFunctionalManager/TrackingTypes.hpp>
#include <core/phase_spec.hpp>

namespace ego_planner::core
{

enum class TaskType
{
  UNKNOWN = 0,
  STATE_TO_STATE,
  TRACKING,
  PERCHING
};

struct TaskSpec
{
  TaskType type{TaskType::UNKNOWN};
  std::string task_name{"unknown_task"};

  Eigen::Vector3d start_pt{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d start_acc{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal_pt{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal_vel{Eigen::Vector3d::Zero()};

  bool touch_goal{false};
  bool force_plain{false};
  bool flag_poly_init{false};
  bool flag_random_poly_traj{false};

  // Tracking semantics are stored explicitly in the task.
  cost_functional::TrackingReference tracking_reference;
  std::vector<Eigen::Vector3d> preferred_guide_path;

  std::vector<PhaseSpec> phases;
  uint32_t active_component_mask{0U};
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_TASK_SPEC_HPP_

