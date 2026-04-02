#include <compiler/problem_compiler.hpp>

#include <optimization/constraint_registry.hpp>
#include <optimization/objective_registry.hpp>

namespace
{

using ego_planner::core::ActiveSpaceModel;
using ego_planner::core::SpaceModelPreference;
using ego_planner::core::TaskDefinition;
using ego_planner::core::TaskType;

ActiveSpaceModel selectStateToStateSpaceModel(const TaskDefinition &task_definition,
                                              const ego_planner::core::PlanningContext &context)
{
  const auto &policy = task_definition.space_model_policy;
  if (policy.force_plain || policy.preferred == SpaceModelPreference::PLAIN)
  {
    return ActiveSpaceModel::PLAIN;
  }

  if (policy.preferred == SpaceModelPreference::CORRIDOR &&
      policy.allow_corridor && context.has_grid_map && context.has_jps)
  {
    return ActiveSpaceModel::CORRIDOR;
  }

  if (policy.preferred == SpaceModelPreference::ESDF &&
      policy.allow_esdf && context.has_esdf)
  {
    return ActiveSpaceModel::ESDF;
  }

  if (policy.allow_corridor && context.has_grid_map && context.has_jps)
  {
    return ActiveSpaceModel::CORRIDOR;
  }

  if (policy.allow_esdf && context.has_esdf)
  {
    return ActiveSpaceModel::ESDF;
  }

  return ActiveSpaceModel::PLAIN;
}

ActiveSpaceModel selectActiveSpaceModel(const TaskDefinition &task_definition,
                                        const ego_planner::core::PlanningContext &context)
{
  const auto &policy = task_definition.space_model_policy;
  switch (task_definition.type)
  {
  case TaskType::STATE_TO_STATE:
    return selectStateToStateSpaceModel(task_definition, context);
  case TaskType::TRACKING:
    if (policy.force_plain || policy.preferred == SpaceModelPreference::PLAIN)
    {
      return ActiveSpaceModel::PLAIN;
    }
    return policy.allow_visible_region ? ActiveSpaceModel::VISIBLE_REGION
                                       : ActiveSpaceModel::PLAIN;
  case TaskType::PERCHING:
    if (policy.force_plain || policy.preferred == SpaceModelPreference::PLAIN)
    {
      return ActiveSpaceModel::PLAIN;
    }
    return policy.allow_terminal_manifold ? ActiveSpaceModel::TERMINAL_MANIFOLD
                                          : ActiveSpaceModel::PLAIN;
  case TaskType::UNKNOWN:
  default:
    return ActiveSpaceModel::PLAIN;
  }
}

void applyDefaultObjectiveConstraintPolicy(const TaskDefinition &task_definition,
                                          ego_planner::core::PlanningProblem &problem)
{
  problem.objective_mask =
      ego_planner::optimization::OBJ_SMOOTHNESS |
      ego_planner::optimization::OBJ_FEASIBILITY |
      ego_planner::optimization::OBJ_OBSTACLE;
  problem.constraint_mask =
      ego_planner::optimization::CON_DYNAMICS |
      ego_planner::optimization::CON_COLLISION;

  if (problem.active_space_model == ActiveSpaceModel::CORRIDOR)
  {
    problem.objective_mask |= ego_planner::optimization::OBJ_CORRIDOR;
    problem.constraint_mask |= ego_planner::optimization::CON_CORRIDOR;
  }
  if (problem.active_space_model == ActiveSpaceModel::VISIBLE_REGION ||
      task_definition.type == TaskType::TRACKING)
  {
    problem.objective_mask |=
        ego_planner::optimization::OBJ_TRACKING_DISTANCE |
        ego_planner::optimization::OBJ_TRACKING_VIEW |
        ego_planner::optimization::OBJ_TRACKING_VISIBILITY |
        ego_planner::optimization::OBJ_TERMINAL_SOFT;
    problem.constraint_mask |= ego_planner::optimization::CON_VISIBLE_REGION;
  }
  if (problem.active_space_model == ActiveSpaceModel::TERMINAL_MANIFOLD)
  {
    problem.objective_mask |= ego_planner::optimization::OBJ_TERMINAL_SOFT;
  }
}

} // namespace

namespace ego_planner::compiler
{

bool ProblemCompiler::compile(const core::PlanningContext &context,
                              const core::TaskSpec &task,
                              core::PlanningProblem &problem) const
{
  return compile(context, core::TaskDefinition::fromTaskSpec(task), problem);
}

bool ProblemCompiler::compile(const core::PlanningContext &context,
                              const core::TaskDefinition &task_definition,
                              core::PlanningProblem &problem) const
{
  problem = core::PlanningProblem{};
  problem.problem_name = task_definition.task_name;
  problem.context = context;
  problem.task_definition = task_definition;
  problem.task = task_definition.toTaskSpec();
  problem.prefer_legacy_fallback =
      task_definition.runtime_policy.preserve_legacy_compatibility &&
      (task_definition.type != core::TaskType::STATE_TO_STATE);
  problem.active_space_model = selectActiveSpaceModel(task_definition, context);

  if (!reference_builder_.build(context, task_definition, problem))
  {
    return false;
  }
  if (!feasible_set_builder_.build(context, task_definition, problem))
  {
    return false;
  }
  if (!seed_builder_.build(context, task_definition, problem))
  {
    return false;
  }

  if (!problem.phase_specs.empty())
  {
    std::vector<int> all_set_indices;
    all_set_indices.reserve(problem.feasible_sets.size());
    for (int i = 0; i < static_cast<int>(problem.feasible_sets.size()); ++i)
    {
      all_set_indices.push_back(i);
    }
    for (auto &phase : problem.phase_specs)
    {
      if (phase.feasible_set_indices.empty())
      {
        phase.feasible_set_indices = all_set_indices;
      }
    }
  }

  if (task_definition.objective_constraint_policy.use_default_task_policy)
  {
    applyDefaultObjectiveConstraintPolicy(task_definition, problem);
  }
  else
  {
    problem.objective_mask = task_definition.objective_constraint_policy.objective_mask;
    problem.constraint_mask = task_definition.objective_constraint_policy.constraint_mask;
  }

  if (adapter_ != nullptr)
  {
    switch (task_definition.type)
    {
    case core::TaskType::STATE_TO_STATE:
      problem.solve_callback =
          [adapter = adapter_](const core::PlanningProblem &p, core::PlanningSolution &s) -> bool
      {
        return adapter->solveStateToStateCompiled(p, s);
      };
      break;
    case core::TaskType::TRACKING:
      problem.solve_callback =
          [adapter = adapter_](const core::PlanningProblem &p, core::PlanningSolution &s) -> bool
      {
        return adapter->solveTrackingLegacy(p, s);
      };
      break;
    case core::TaskType::PERCHING:
      problem.solve_callback =
          [adapter = adapter_](const core::PlanningProblem &p, core::PlanningSolution &s) -> bool
      {
        return adapter->solvePerchingLegacy(p, s);
      };
      break;
    case core::TaskType::UNKNOWN:
    default:
      problem.solve_callback =
          [adapter = adapter_](const core::PlanningProblem &p, core::PlanningSolution &s) -> bool
      {
        return adapter->solveCompatibility(p, s);
      };
      break;
    }
  }

  problem.valid = static_cast<bool>(problem.solve_callback);
  return problem.valid;
}

} // namespace ego_planner::compiler
