#include <compiler/problem_compiler.hpp>

#include <optimization/constraint_registry.hpp>
#include <optimization/objective_registry.hpp>

namespace ego_planner::compiler
{

bool ProblemCompiler::compile(const core::PlanningContext &context,
                              const core::TaskSpec &task,
                              core::PlanningProblem &problem) const
{
  problem = core::PlanningProblem{};
  problem.problem_name = task.task_name;
  problem.context = context;
  problem.task = task;
  problem.prefer_legacy_fallback = (task.type != core::TaskType::STATE_TO_STATE);

  // Single authoritative active-space-model selection.
  switch (task.type)
  {
  case core::TaskType::STATE_TO_STATE:
    if (task.force_plain)
    {
      problem.active_space_model = core::ActiveSpaceModel::PLAIN;
    }
    else if (task.prefer_corridor && context.has_grid_map && context.has_jps)
    {
      problem.active_space_model = core::ActiveSpaceModel::CORRIDOR;
    }
    else if (task.prefer_esdf && context.has_esdf)
    {
      problem.active_space_model = core::ActiveSpaceModel::ESDF;
    }
    else
    {
      problem.active_space_model = core::ActiveSpaceModel::PLAIN;
    }
    break;
  case core::TaskType::TRACKING:
    problem.active_space_model =
        task.force_plain ? core::ActiveSpaceModel::PLAIN : core::ActiveSpaceModel::VISIBLE_REGION;
    break;
  case core::TaskType::PERCHING:
    problem.active_space_model = core::ActiveSpaceModel::TERMINAL_MANIFOLD;
    break;
  case core::TaskType::UNKNOWN:
  default:
    problem.active_space_model = core::ActiveSpaceModel::PLAIN;
    break;
  }

  if (!reference_builder_.build(context, task, problem))
  {
    return false;
  }
  if (!feasible_set_builder_.build(context, task, problem))
  {
    return false;
  }
  if (!seed_builder_.build(context, task, problem))
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

  // Default objective/constraint masks are generic and task-agnostic at solver layer.
  problem.objective_mask =
      optimization::OBJ_SMOOTHNESS |
      optimization::OBJ_FEASIBILITY |
      optimization::OBJ_OBSTACLE;
  problem.constraint_mask =
      optimization::CON_DYNAMICS |
      optimization::CON_COLLISION;

  if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
  {
    problem.objective_mask |= optimization::OBJ_CORRIDOR;
    problem.constraint_mask |= optimization::CON_CORRIDOR;
  }
  if (problem.active_space_model == core::ActiveSpaceModel::VISIBLE_REGION ||
      task.type == core::TaskType::TRACKING)
  {
    problem.objective_mask |=
        optimization::OBJ_TRACKING_DISTANCE |
        optimization::OBJ_TRACKING_VIEW |
        optimization::OBJ_TRACKING_VISIBILITY |
        optimization::OBJ_TERMINAL_SOFT;
    problem.constraint_mask |= optimization::CON_VISIBLE_REGION;
  }
  if (problem.active_space_model == core::ActiveSpaceModel::TERMINAL_MANIFOLD)
  {
    problem.objective_mask |= optimization::OBJ_TERMINAL_SOFT;
  }

  if (adapter_ != nullptr)
  {
    switch (task.type)
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
