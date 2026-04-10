#include <compiler/feasible_set_builder.hpp>

namespace ego_planner::compiler
{

bool FeasibleSetBuilder::ensureTransitGuidePath(const core::PlanningContext &context,
                                                const core::TaskDefinition &task_definition,
                                                core::PlanningProblem &problem) const
{
  frontend::GuidePathArtifact guide_artifact;
  if (problem.references.guide_path.size() >= 2)
  {
    guide_artifact.points = problem.references.guide_path;
    if (problem.references.guide_times.size() == problem.references.guide_path.size())
    {
      guide_artifact.times = problem.references.guide_times;
    }
    else if (!guide_path_service_.buildFromWaypoints(problem.references.guide_path, guide_artifact))
    {
      problem.compile_message = "failed to rebuild guide-path timing from compiled guide points";
      return false;
    }
  }
  else if (!guide_path_service_.buildStateToStateGuide(context, task_definition, guide_artifact))
  {
    problem.compile_message = "failed to generate state-to-state guide path";
    return false;
  }

  if (!guide_artifact.valid())
  {
    problem.compile_message = "compiled guide path is invalid";
    return false;
  }

  problem.references.guide_path = guide_artifact.points;
  problem.references.guide_times = guide_artifact.times;
  return true;
}

bool FeasibleSetBuilder::buildPlainFeasibleSets(const core::PlanningContext &context,
                                                const core::TaskDefinition &task_definition,
                                                core::PlanningProblem &problem) const
{
  (void)context;
  (void)task_definition;
  (void)problem;
  return true;
}

bool FeasibleSetBuilder::buildEsdfFeasibleSets(const core::PlanningContext &context,
                                               const core::TaskDefinition &task_definition,
                                               core::PlanningProblem &problem) const
{
  if (problem.references.guide_path.size() >= 2)
  {
    frontend::GuidePathArtifact guide_artifact;
    guide_artifact.points = problem.references.guide_path;
    if (problem.references.guide_times.size() == problem.references.guide_path.size())
    {
      guide_artifact.times = problem.references.guide_times;
    }
    else
    {
      guide_path_service_.buildFromWaypoints(problem.references.guide_path, guide_artifact);
    }
    if (guide_artifact.valid())
    {
      problem.references.guide_path = guide_artifact.points;
      problem.references.guide_times = guide_artifact.times;
    }
    return true;
  }

  frontend::GuidePathArtifact guide_artifact;
  if (guide_path_service_.buildStateToStateGuide(context, task_definition, guide_artifact) &&
      guide_artifact.valid())
  {
    problem.references.guide_path = guide_artifact.points;
    problem.references.guide_times = guide_artifact.times;
  }
  return true;
}

bool FeasibleSetBuilder::buildCorridorFeasibleSets(const core::PlanningContext &context,
                                                   const core::TaskDefinition &task_definition,
                                                   core::PlanningProblem &problem) const
{
  // For state-to-state corridor mode, compiler only preserves optional guide hints.
  // The authoritative corridor construction happens in solver-side stable helpers.
  if (!ensureTransitGuidePath(context, task_definition, problem))
  {
    problem.compile_message.clear();
    problem.references.guide_path.clear();
    problem.references.guide_times.clear();
  }
  return true;
}

bool FeasibleSetBuilder::buildTransitFeasibleSets(const core::PlanningContext &context,
                                                  const core::TaskDefinition &task_definition,
                                                  core::PlanningProblem &problem) const
{
  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    return buildCorridorFeasibleSets(context, task_definition, problem);
  case core::ActiveSpaceModel::ESDF:
    return buildEsdfFeasibleSets(context, task_definition, problem);
  case core::ActiveSpaceModel::PLAIN:
  default:
    return buildPlainFeasibleSets(context, task_definition, problem);
  }
}

bool FeasibleSetBuilder::buildTrackingFeasibleSets(const core::PlanningContext &context,
                                                   const core::TaskDefinition &task_definition,
                                                   core::PlanningProblem &problem) const
{
  (void)task_definition;
  // Tracking V1 reuses the same initializer/solver path as state-to-state.
  // Compiler-side guide/corridor outputs remain optional hints.
  return buildTransitFeasibleSets(context, task_definition, problem);
}

bool FeasibleSetBuilder::buildPerchingFeasibleSets(const core::PlanningContext &context,
                                                   const core::TaskDefinition &task_definition,
                                                   core::PlanningProblem &problem) const
{
  (void)context;
  if (task_definition.goal.isTerminalManifold())
  {
    core::FeasibleSetSpec terminal_set;
    terminal_set.type = core::FeasibleSetType::TERMINAL_MANIFOLD;
    terminal_set.label = "perching_terminal_manifold";
    terminal_set.manifold_params = task_definition.goal.manifold_params;
    terminal_set.enabled = task_definition.goal.manifold_params.size() > 0;
    problem.feasible_sets.push_back(terminal_set);
  }

  // Perching V1 uses the state-to-state spatial model for guide/corridor/ESDF
  // initialization, while the terminal manifold remains a terminal semantic.
  if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
  {
    return buildCorridorFeasibleSets(context, task_definition, problem);
  }
  if (problem.active_space_model == core::ActiveSpaceModel::ESDF)
  {
    return buildEsdfFeasibleSets(context, task_definition, problem);
  }
  return true;
}

bool FeasibleSetBuilder::build(const core::PlanningContext &context,
                               const core::TaskDefinition &task_definition,
                               core::PlanningProblem &problem) const
{
  problem.feasible_sets.clear();
  switch (task_definition.type)
  {
  case core::TaskType::STATE_TO_STATE:
    return buildTransitFeasibleSets(context, task_definition, problem);
  case core::TaskType::TRACKING:
    return buildTrackingFeasibleSets(context, task_definition, problem);
  case core::TaskType::PERCHING:
    return buildPerchingFeasibleSets(context, task_definition, problem);
  case core::TaskType::UNKNOWN:
  default:
    return false;
  }
}

} // namespace ego_planner::compiler
