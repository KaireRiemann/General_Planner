#ifndef PLANNER_CORE_TASK_DEFINITION_HPP_
#define PLANNER_CORE_TASK_DEFINITION_HPP_

#include <core/goal_definition.hpp>
#include <core/phase_definition.hpp>
#include <core/reference_definition.hpp>
#include <core/runtime_policy.hpp>
#include <core/space_model_policy.hpp>
#include <core/task_spec.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ego_planner::core
{

struct ObjectiveConstraintPolicy
{
  uint32_t objective_mask{0U};
  uint32_t constraint_mask{0U};
  bool use_default_task_policy{true};
};

inline TerminalSpecType toTerminalSpecType(const GoalSemanticType semantic)
{
  switch (semantic)
  {
  case GoalSemanticType::TERMINAL_SET:
    return TerminalSpecType::TERMINAL_SET;
  case GoalSemanticType::TERMINAL_MANIFOLD:
    return TerminalSpecType::TERMINAL_MANIFOLD;
  case GoalSemanticType::FIXED_STATE:
  default:
    return TerminalSpecType::FIXED_STATE;
  }
}

inline GoalSemanticType toGoalSemanticType(const TerminalSpecType semantic)
{
  switch (semantic)
  {
  case TerminalSpecType::TERMINAL_SET:
    return GoalSemanticType::TERMINAL_SET;
  case TerminalSpecType::TERMINAL_MANIFOLD:
    return GoalSemanticType::TERMINAL_MANIFOLD;
  case TerminalSpecType::FIXED_STATE:
  default:
    return GoalSemanticType::FIXED_STATE;
  }
}

struct TaskDefinition
{
  TaskType type{TaskType::UNKNOWN};
  std::string task_name{"unknown_task"};

  StateDefinition start_state;
  GoalDefinition goal;
  std::vector<PhaseDefinition> phases;
  std::vector<ReferenceDefinition> references;

  SpaceModelPolicy space_model_policy;
  ObjectiveConstraintPolicy objective_constraint_policy;
  RuntimePolicy runtime_policy;

  uint32_t active_component_mask{0U};

  const ReferenceDefinition *findActiveReference(const ReferenceSemanticType semantic) const
  {
    for (const auto &reference : references)
    {
      if (reference.active && reference.semantic == semantic && reference.valid())
      {
        return &reference;
      }
    }
    return nullptr;
  }

  TaskSpec toTaskSpec() const
  {
    TaskSpec task;
    task.type = type;
    task.task_name = task_name;
    task.start_pt = start_state.position;
    task.start_vel = start_state.velocity;
    task.start_acc = start_state.acceleration;
    task.touch_goal = runtime_policy.touch_goal || goal.touch_goal;
    task.flag_poly_init = runtime_policy.flag_poly_init;
    task.flag_random_poly_traj = runtime_policy.flag_random_poly_traj;
    task.force_plain =
        space_model_policy.force_plain ||
        space_model_policy.preferred == SpaceModelPreference::PLAIN;
    task.prefer_corridor =
        !task.force_plain &&
        space_model_policy.preferred == SpaceModelPreference::CORRIDOR;
    task.prefer_esdf =
        !task.force_plain &&
        space_model_policy.preferred == SpaceModelPreference::ESDF;
    task.active_component_mask = active_component_mask;

    const GoalDefinition *effective_goal = &goal;
    if (!effective_goal->state.valid && !phases.empty())
    {
      effective_goal = &phases.back().goal;
    }

    if (effective_goal->state.valid)
    {
      task.goal_pt = effective_goal->state.position;
      task.goal_vel = effective_goal->state.velocity;
    }

    for (const auto &reference : references)
    {
      if (!reference.active || !reference.valid())
      {
        continue;
      }

      if ((reference.semantic == ReferenceSemanticType::GUIDE_PATH ||
           reference.semantic == ReferenceSemanticType::WAYPOINT_SEQUENCE) &&
          task.preferred_guide_path.empty())
      {
        task.preferred_guide_path = reference.points;
      }
      else if (reference.semantic == ReferenceSemanticType::TRACKING_TRAJECTORY &&
               !task.tracking_reference.valid())
      {
        task.tracking_reference = reference.tracking_reference;
      }
    }

    if (phases.empty())
    {
      PhaseSpec phase;
      phase.name = "phase";
      phase.terminal_type = toTerminalSpecType(effective_goal->semantic);
      phase.terminal_position = task.goal_pt;
      phase.terminal_velocity = task.goal_vel;
      phase.terminal_manifold_params = effective_goal->manifold_params;
      phase.active_objective_mask = objective_constraint_policy.objective_mask;
      phase.active_constraint_mask = objective_constraint_policy.constraint_mask;
      task.phases.push_back(phase);
    }
    else
    {
      task.phases.reserve(phases.size());
      for (const auto &phase_def : phases)
      {
        PhaseSpec phase;
        phase.name = phase_def.name;
        phase.terminal_type = toTerminalSpecType(phase_def.goal.semantic);
        phase.terminal_position = phase_def.goal.state.position;
        phase.terminal_velocity = phase_def.goal.state.velocity;
        phase.terminal_manifold_params = phase_def.goal.manifold_params;
        phase.active_objective_mask = phase_def.objective_mask;
        phase.active_constraint_mask = phase_def.constraint_mask;
        task.phases.push_back(phase);
      }
    }

    return task;
  }

  static TaskDefinition fromTaskSpec(const TaskSpec &task)
  {
    TaskDefinition definition;
    definition.type = task.type;
    definition.task_name = task.task_name;
    definition.start_state.valid = true;
    definition.start_state.position = task.start_pt;
    definition.start_state.velocity = task.start_vel;
    definition.start_state.acceleration = task.start_acc;
    definition.active_component_mask = task.active_component_mask;

    definition.runtime_policy.touch_goal = task.touch_goal;
    definition.runtime_policy.flag_poly_init = task.flag_poly_init;
    definition.runtime_policy.flag_random_poly_traj = task.flag_random_poly_traj;
    definition.runtime_policy.preserve_legacy_compatibility = true;

    definition.space_model_policy.force_plain = task.force_plain;
    switch (task.type)
    {
    case TaskType::TRACKING:
      definition.space_model_policy.preferred =
          task.force_plain ? SpaceModelPreference::PLAIN
                           : SpaceModelPreference::VISIBLE_REGION;
      break;
    case TaskType::PERCHING:
      definition.space_model_policy.preferred =
          task.force_plain ? SpaceModelPreference::PLAIN
                           : SpaceModelPreference::TERMINAL_MANIFOLD;
      break;
    case TaskType::STATE_TO_STATE:
      if (task.force_plain)
      {
        definition.space_model_policy.preferred = SpaceModelPreference::PLAIN;
      }
      else if (task.prefer_corridor)
      {
        definition.space_model_policy.preferred = SpaceModelPreference::CORRIDOR;
      }
      else if (task.prefer_esdf)
      {
        definition.space_model_policy.preferred = SpaceModelPreference::ESDF;
      }
      else
      {
        definition.space_model_policy.preferred = SpaceModelPreference::AUTO;
      }
      break;
    case TaskType::UNKNOWN:
    default:
      definition.space_model_policy.preferred = SpaceModelPreference::AUTO;
      break;
    }

    if (!task.preferred_guide_path.empty())
    {
      ReferenceDefinition guide_reference;
      guide_reference.semantic = ReferenceSemanticType::GUIDE_PATH;
      guide_reference.name = "preferred_guide_path";
      guide_reference.active = true;
      guide_reference.points = task.preferred_guide_path;
      definition.references.push_back(guide_reference);
    }

    if (task.tracking_reference.valid())
    {
      ReferenceDefinition tracking_reference;
      tracking_reference.semantic = ReferenceSemanticType::TRACKING_TRAJECTORY;
      tracking_reference.name = "tracking_reference";
      tracking_reference.active = true;
      tracking_reference.tracking_reference = task.tracking_reference;
      definition.references.push_back(tracking_reference);
    }

    if (!task.phases.empty())
    {
      definition.phases.reserve(task.phases.size());
      for (const auto &phase_spec : task.phases)
      {
        PhaseDefinition phase;
        phase.name = phase_spec.name;
        phase.goal.semantic = toGoalSemanticType(phase_spec.terminal_type);
        phase.goal.state.valid = true;
        phase.goal.state.position = phase_spec.terminal_position;
        phase.goal.state.velocity = phase_spec.terminal_velocity;
        phase.goal.manifold_params = phase_spec.terminal_manifold_params;
        phase.objective_mask = phase_spec.active_objective_mask;
        phase.constraint_mask = phase_spec.active_constraint_mask;
        if (!definition.references.empty())
        {
          for (std::size_t i = 0; i < definition.references.size(); ++i)
          {
            if (definition.references[i].active)
            {
              phase.active_reference_indices.push_back(i);
            }
          }
        }
        definition.phases.push_back(phase);
      }
      definition.goal = definition.phases.back().goal;
    }
    else
    {
      definition.goal.semantic = GoalSemanticType::FIXED_STATE;
      definition.goal.state.valid = true;
      definition.goal.state.position = task.goal_pt;
      definition.goal.state.velocity = task.goal_vel;
    }

    definition.goal.touch_goal = task.touch_goal;
    if (!definition.goal.state.valid)
    {
      definition.goal.state.valid = true;
      definition.goal.state.position = task.goal_pt;
      definition.goal.state.velocity = task.goal_vel;
    }

    return definition;
  }
};

} // namespace ego_planner::core

#endif // PLANNER_CORE_TASK_DEFINITION_HPP_
