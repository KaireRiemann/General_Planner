#include <compiler/problem_compiler.hpp>

#include <optimization/constraint_registry.hpp>
#include <optimization/objective_registry.hpp>

namespace
{

using ego_planner::core::ActiveSpaceModel;
using ego_planner::core::PlanningProblem;
using ego_planner::core::SpaceModelPreference;
using ego_planner::core::TaskSemanticArtifact;
using ego_planner::core::TaskDefinition;
using ego_planner::core::TaskType;

double extractPerchingApproachDistance(const TaskDefinition &task_definition,
                                       const ego_planner::core::StateDefinition &contact_state,
                                       const ego_planner::core::StateDefinition &approach_state)
{
  if (!contact_state.valid || !approach_state.valid)
  {
    return 0.0;
  }

  const Eigen::VectorXd *manifold_params = nullptr;
  if (task_definition.goal.manifold_params.size() >= 21)
  {
    manifold_params = &task_definition.goal.manifold_params;
  }
  else if (!task_definition.phases.empty() &&
           task_definition.phases.back().goal.manifold_params.size() >= 21)
  {
    manifold_params = &task_definition.phases.back().goal.manifold_params;
  }

  if (manifold_params != nullptr)
  {
    const Eigen::Vector3d surface_z = manifold_params->segment<3>(18);
    if (surface_z.allFinite() && surface_z.norm() > 1.0e-6)
    {
      return std::max(0.0,
                      (contact_state.position - approach_state.position)
                          .dot(surface_z.normalized()));
    }
  }

  return std::max(0.0, (contact_state.position - approach_state.position).norm());
}

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
  switch (task_definition.type)
  {
  case TaskType::STATE_TO_STATE:
    return selectStateToStateSpaceModel(task_definition, context);
  case TaskType::TRACKING:
    // Tracking V1 follows the same space-model selection as state-to-state:
    // plain/esdf/corridor. Visible-region semantic mode is intentionally not
    // used in this migration step.
    return selectStateToStateSpaceModel(task_definition, context);
  case TaskType::PERCHING:
    // Perching V1 uses the same spatial model pipeline as state-to-state.
    // The terminal manifold is task semantics/feasible-set data, not the
    // active obstacle model used by the backend solver.
    return selectStateToStateSpaceModel(task_definition, context);
  case TaskType::UNKNOWN:
  default:
    return ActiveSpaceModel::PLAIN;
  }
}

TaskSemanticArtifact buildTaskSemanticArtifact(const TaskDefinition &task_definition)
{
  TaskSemanticArtifact artifact;
  artifact.task_type = task_definition.type;

  artifact.transit.start_state = task_definition.start_state;
  artifact.transit.terminal_state = task_definition.goal.state;
  if (!artifact.transit.terminal_state.valid && !task_definition.phases.empty())
  {
    artifact.transit.terminal_state = task_definition.phases.back().goal.state;
  }
  artifact.transit.touch_goal =
      task_definition.runtime_policy.touch_goal || task_definition.goal.touch_goal;
  artifact.transit.valid =
      artifact.transit.start_state.valid && artifact.transit.terminal_state.valid;

  if (task_definition.type == TaskType::TRACKING)
  {
    artifact.tracking = task_definition.tracking_semantics;
    if (!artifact.tracking.consistent())
    {
      const auto *tracking_reference =
          task_definition.findActiveReference(ego_planner::core::ReferenceSemanticType::TRACKING_TRAJECTORY);
      if (tracking_reference != nullptr && tracking_reference->tracking_reference.valid())
      {
        artifact.tracking =
            ego_planner::core::TrackingSemanticArtifact::fromTrackingReference(
                tracking_reference->tracking_reference);
      }
    }
  }

  if (task_definition.type == TaskType::PERCHING)
  {
    artifact.perching.contact_state = task_definition.goal.state;
    artifact.perching.terminal_manifold_params = task_definition.goal.manifold_params;
    if ((!artifact.perching.contact_state.valid ||
         artifact.perching.terminal_manifold_params.size() == 0) &&
        !task_definition.phases.empty())
    {
      const auto &final_phase = task_definition.phases.back();
      if (!artifact.perching.contact_state.valid)
      {
        artifact.perching.contact_state = final_phase.goal.state;
      }
      if (artifact.perching.terminal_manifold_params.size() == 0)
      {
        artifact.perching.terminal_manifold_params = final_phase.goal.manifold_params;
      }
    }

    if (!task_definition.phases.empty())
    {
      artifact.perching.approach_anchor_state = task_definition.phases.front().goal.state;
      artifact.perching.approach_distance =
          extractPerchingApproachDistance(task_definition,
                                          artifact.perching.contact_state,
                                          artifact.perching.approach_anchor_state);
    }

    artifact.perching.valid = artifact.perching.contact_state.valid;
    artifact.perching.touch_goal =
        task_definition.runtime_policy.touch_goal || task_definition.goal.touch_goal;
  }

  return artifact;
}

bool validateCompiledProblem(PlanningProblem &problem)
{
  if (problem.representation != ego_planner::core::RepresentationKind::MINCO)
  {
    problem.compile_message = "compiled problem representation is not MINCO";
    return false;
  }
  if (!problem.start_boundary.valid)
  {
    problem.compile_message = "compiled problem is missing valid start boundary";
    return false;
  }
  if (!problem.terminal_boundary.valid)
  {
    problem.compile_message = "compiled problem is missing valid terminal boundary";
    return false;
  }
  if (problem.task_definition.type != TaskType::STATE_TO_STATE)
  {
    return true;
  }
  // For state-to-state, guide/corridor/seed are solver-owned initialization artifacts.
  // The compiler only needs to provide valid semantics plus optional hints.
  return true;
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
  if (task_definition.type == TaskType::TRACKING)
  {
    problem.objective_mask |=
        ego_planner::optimization::OBJ_TRACKING_DISTANCE |
        ego_planner::optimization::OBJ_TRACKING_VIEW |
        ego_planner::optimization::OBJ_TRACKING_VISIBILITY |
        ego_planner::optimization::OBJ_TERMINAL_SOFT;
  }
  if (problem.active_space_model == ActiveSpaceModel::VISIBLE_REGION)
  {
    problem.constraint_mask |= ego_planner::optimization::CON_VISIBLE_REGION;
  }
  if (problem.active_space_model == ActiveSpaceModel::TERMINAL_MANIFOLD)
  {
    problem.objective_mask |= ego_planner::optimization::OBJ_TERMINAL_SOFT;
  }
  if (task_definition.type == TaskType::PERCHING)
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
  problem.compile_message.clear();
  problem.context = context;
  problem.task_definition = task_definition;
  problem.task_semantics = buildTaskSemanticArtifact(task_definition);
  problem.task = task_definition.toTaskSpec();
  problem.prefer_legacy_fallback =
      task_definition.runtime_policy.preserve_legacy_compatibility &&
      (task_definition.type != core::TaskType::STATE_TO_STATE);
  problem.active_space_model = selectActiveSpaceModel(task_definition, context);

  if (!reference_builder_.build(context, task_definition, problem))
  {
    if (problem.compile_message.empty())
    {
      problem.compile_message = "reference builder rejected task";
    }
    return false;
  }
  if (!feasible_set_builder_.build(context, task_definition, problem))
  {
    if (problem.compile_message.empty())
    {
      problem.compile_message = "feasible-set builder rejected task";
    }
    return false;
  }
  if (!seed_builder_.build(context, task_definition, problem))
  {
    if (problem.compile_message.empty())
    {
      problem.compile_message = "seed builder rejected task";
    }
    return false;
  }

  if (!validateCompiledProblem(problem))
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
        return adapter->solveTrackingCompiled(p, s);
      };
      break;
    case core::TaskType::PERCHING:
      problem.solve_callback =
          [adapter = adapter_](const core::PlanningProblem &p, core::PlanningSolution &s) -> bool
      {
        return adapter->solveStateToStateCompiled(p, s);
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
  if (!problem.valid && problem.compile_message.empty())
  {
    problem.compile_message = "compiled planning problem is missing solve callback";
  }
  return problem.valid;
}

} // namespace ego_planner::compiler
