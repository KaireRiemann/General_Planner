#include <compiler/feasible_set_builder.hpp>

namespace ego_planner::compiler
{

bool FeasibleSetBuilder::build(const core::PlanningContext &context,
                               const core::TaskSpec &task,
                               core::PlanningProblem &problem) const
{
  (void)context;
  problem.feasible_sets.clear();

  if (!task.preferred_guide_path.empty())
  {
    frontend::GuidePathArtifact guide;
    guide.points = task.preferred_guide_path;
    guide.times.resize(guide.points.size(), 0.0);
    for (std::size_t i = 1; i < guide.points.size(); ++i)
    {
      guide.times[i] = guide.times[i - 1] + (guide.points[i] - guide.points[i - 1]).norm();
    }

    core::FeasibleSetSpec corridor;
    if (corridor_service_.buildFromGuidePath(guide, corridor))
    {
      problem.feasible_sets.push_back(corridor);
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

} // namespace ego_planner::compiler
