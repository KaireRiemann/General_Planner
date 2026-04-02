#include <compiler/feasible_set_builder.hpp>

namespace
{

const ego_planner::core::ReferenceDefinition *findGuideReference(const ego_planner::core::TaskDefinition &task_definition)
{
  const auto *guide_ref =
      task_definition.findActiveReference(ego_planner::core::ReferenceSemanticType::GUIDE_PATH);
  if (guide_ref != nullptr)
  {
    return guide_ref;
  }
  return task_definition.findActiveReference(ego_planner::core::ReferenceSemanticType::WAYPOINT_SEQUENCE);
}

} // namespace

namespace ego_planner::compiler
{

bool FeasibleSetBuilder::buildTransitFeasibleSets(const core::PlanningContext &context,
                                                  const core::TaskDefinition &task_definition,
                                                  core::PlanningProblem &problem) const
{
  frontend::GuidePathArtifact guide_artifact;
  if (!guide_path_service_.buildStateToStateGuide(context, task_definition, guide_artifact))
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
                                                   const core::TaskDefinition &task_definition,
                                                   core::PlanningProblem &problem) const
{
  frontend::GuidePathArtifact guide_artifact;
  const auto *guide_ref = findGuideReference(task_definition);
  if (guide_ref != nullptr &&
      guide_path_service_.buildFromWaypoints(guide_ref->points, guide_artifact))
  {
    if (guide_ref->times.size() == guide_ref->points.size())
    {
      guide_artifact.times = guide_ref->times;
    }
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

  (void)task_definition;
  return true;
}

bool FeasibleSetBuilder::buildPerchingFeasibleSets(const core::PlanningContext &context,
                                                   const core::TaskDefinition &task_definition,
                                                   core::PlanningProblem &problem) const
{
  (void)context;
  (void)task_definition;
  (void)problem;
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
