#include <compiler/seed_builder.hpp>

#include <algorithm>

#include <traj_utils/minco_types.hpp>

namespace
{

const ego_planner::core::FeasibleSetSpec *findCorridorSet(const ego_planner::core::PlanningProblem &problem)
{
  for (const auto &set : problem.feasible_sets)
  {
    if (set.enabled &&
        set.type == ego_planner::core::FeasibleSetType::CORRIDOR_POLYTOPE &&
        !set.corridor.empty())
    {
      return &set;
    }
  }
  return nullptr;
}

ego_planner::core::SeedSpec::Kind guideHintKindFor(const ego_planner::core::ActiveSpaceModel mode)
{
  switch (mode)
  {
  case ego_planner::core::ActiveSpaceModel::ESDF:
    return ego_planner::core::SeedSpec::Kind::ESDF_INIT;
  case ego_planner::core::ActiveSpaceModel::VISIBLE_REGION:
    return ego_planner::core::SeedSpec::Kind::SEMANTIC_INIT;
  case ego_planner::core::ActiveSpaceModel::PLAIN:
  case ego_planner::core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  default:
    return ego_planner::core::SeedSpec::Kind::PLAIN_INIT;
  }
}

int hintPieceNum(const ego_planner::core::SeedSpec &seed)
{
  return std::max(0, static_cast<int>(seed.anchor_points.size()) - 1);
}

int hintInnerPointNum(const ego_planner::core::SeedSpec &seed)
{
  return std::max(0, static_cast<int>(seed.anchor_points.size()) - 2);
}

bool isTransitTask(const ego_planner::core::TaskType type)
{
  return type == ego_planner::core::TaskType::STATE_TO_STATE ||
         type == ego_planner::core::TaskType::TRACKING ||
         type == ego_planner::core::TaskType::PERCHING;
}

} // namespace

namespace ego_planner::compiler
{

bool SeedBuilder::buildGuideHint(const core::PlanningProblem &problem,
                                 core::SeedSpec &seed) const
{
  if (problem.references.guide_path.size() < 2)
  {
    return false;
  }

  seed = core::SeedSpec{};
  seed.valid = true;
  seed.kind = guideHintKindFor(problem.active_space_model);
  seed.anchor_points = problem.references.guide_path;
  return true;
}

bool SeedBuilder::buildCorridorHint(const core::PlanningProblem &problem,
                                    core::SeedSpec &seed) const
{
  const auto *corridor_set = findCorridorSet(problem);
  if (corridor_set == nullptr)
  {
    return false;
  }

  seed = core::SeedSpec{};
  seed.valid = true;
  seed.kind = core::SeedSpec::Kind::CORRIDOR_INIT;
  seed.corridor_aware = true;

  if (corridor_set->corridor_seed_path.size() >= 2)
  {
    seed.anchor_points = corridor_set->corridor_seed_path;
  }
  else if (problem.references.guide_path.size() >= 2)
  {
    seed.anchor_points = problem.references.guide_path;
  }

  return true;
}

bool SeedBuilder::buildTransitHint(const core::PlanningProblem &problem,
                                   core::SeedSpec &seed) const
{
  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    return buildCorridorHint(problem, seed);
  case core::ActiveSpaceModel::ESDF:
  case core::ActiveSpaceModel::PLAIN:
  case core::ActiveSpaceModel::VISIBLE_REGION:
  case core::ActiveSpaceModel::TERMINAL_MANIFOLD:
  default:
    return buildGuideHint(problem, seed);
  }
}

bool SeedBuilder::build(const core::PlanningContext &context,
                        const core::TaskDefinition &task_definition,
                        core::PlanningProblem &problem) const
{
  problem.seed = core::SeedSpec{};

  core::SeedSpec seed_hint;
  bool seed_ok = false;
  if (isTransitTask(task_definition.type))
  {
    seed_ok = buildTransitHint(problem, seed_hint);
  }

  if (seed_ok)
  {
    problem.seed = seed_hint;
  }
  else if (isTransitTask(task_definition.type))
  {
    // Transit tasks treat compiler seeds as optional hints only. Final
    // initialization artifacts are produced later by frontend::TransitInitService.
    seed_ok = true;
  }

  if (context.allow_warm_start)
  {
    core::WarmStartCache warm_start;
    if (warm_start_service_.fetch(context, warm_start))
    {
      problem.context.warm_start = warm_start;
      if (!problem.seed.valid)
      {
        problem.seed.valid = true;
        problem.seed.kind = core::SeedSpec::Kind::WARM_START;
        problem.seed.anchor_points = warm_start.anchor_points;
      }
    }
  }

  problem.variable_layout.piece_num = problem.seed.valid ? hintPieceNum(problem.seed) : 0;
  problem.variable_layout.inner_point_num = problem.seed.valid ? hintInnerPointNum(problem.seed) : 0;
  problem.variable_layout.boundary_derivative_num = MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM;

  if (isTransitTask(task_definition.type))
  {
    return true;
  }
  return problem.seed.valid;
}

} // namespace ego_planner::compiler
