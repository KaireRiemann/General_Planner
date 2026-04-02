#include <compiler/feasible_set_builder.hpp>

namespace ego_planner::compiler
{

bool FeasibleSetBuilder::buildTransitFeasibleSets(const core::PlanningContext &context,
                                                  const core::TaskSpec &task,
                                                  core::PlanningProblem &problem) const
{
  frontend::GuidePathArtifact guide_artifact;
  if (!guide_path_service_.buildStateToStateGuide(context, task, guide_artifact))
  {
    return false;
  }
  problem.references.guide_path = guide_artifact.points;
  problem.references.guide_times = guide_artifact.times;

  if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
  {
    core::FeasibleSetSpec corridor_set;
    if (corridor_service_.buildFromGuidePath(context, guide_artifact, corridor_set))
    {
      problem.feasible_sets.push_back(corridor_set);
    }
  }
  return true;
}

bool FeasibleSetBuilder::buildTrackingFeasibleSets(const core::PlanningContext &context,
                                                   const core::TaskSpec &task,
                                                   core::PlanningProblem &problem) const
{
  // Tracking preserves compatibility path for optimization internals.
  // This builder still compiles explicit guide/corridor artifacts when available.
  frontend::GuidePathArtifact guide_artifact;
  if (!task.preferred_guide_path.empty() &&
      guide_path_service_.buildFromWaypoints(task.preferred_guide_path, guide_artifact))
  {
    problem.references.guide_path = guide_artifact.points;
    problem.references.guide_times = guide_artifact.times;
    if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR ||
        problem.active_space_model == core::ActiveSpaceModel::VISIBLE_REGION)
    {
      core::FeasibleSetSpec corridor_set;
      if (corridor_service_.buildFromGuidePath(context, guide_artifact, corridor_set))
      {
        problem.feasible_sets.push_back(corridor_set);
      }
    }
  }

  for (const auto &phase : task.phases)
  {
    for (const auto &set : phase.feasible_sets)
    {
      problem.feasible_sets.push_back(set);
    }
  }
  return true;
}

bool FeasibleSetBuilder::buildPerchingFeasibleSets(const core::PlanningContext &context,
                                                   const core::TaskSpec &task,
                                                   core::PlanningProblem &problem) const
{
  (void)context;
  // Perching keeps manifold structures from phases.
  for (const auto &phase : task.phases)
  {
    for (const auto &set : phase.feasible_sets)
    {
      problem.feasible_sets.push_back(set);
    }
  }
  return true;
}

bool FeasibleSetBuilder::build(const core::PlanningContext &context,
                               const core::TaskSpec &task,
                               core::PlanningProblem &problem) const
{
  problem.feasible_sets.clear();
  switch (task.type)
  {
  case core::TaskType::STATE_TO_STATE:
    return buildTransitFeasibleSets(context, task, problem);
  case core::TaskType::TRACKING:
    return buildTrackingFeasibleSets(context, task, problem);
  case core::TaskType::PERCHING:
    return buildPerchingFeasibleSets(context, task, problem);
  case core::TaskType::UNKNOWN:
  default:
    return false;
  }
}

} // namespace ego_planner::compiler
