#include <compiler/seed_builder.hpp>

#include <algorithm>
#include <traj_utils/minco_types.hpp>

namespace ego_planner::compiler
{

bool SeedBuilder::buildSeedFromGuide(const core::PlanningContext &context,
                                     const core::PlanningProblem &problem,
                                     core::SeedSpec &seed) const
{
  std::vector<Eigen::Vector3d> seed_path = problem.references.guide_path;
  std::vector<double> seed_times = problem.references.guide_times;

  if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
  {
    for (const auto &set : problem.feasible_sets)
    {
      if (!set.enabled || set.type != core::FeasibleSetType::CORRIDOR_POLYTOPE)
      {
        continue;
      }
      if (set.corridor_seed_path.size() >= 2)
      {
        seed_path = set.corridor_seed_path;
        seed_times = set.corridor_seed_times;
        break;
      }
    }
  }

  if (seed_path.size() < 2)
  {
    return false;
  }

  seed = core::SeedSpec{};
  seed.valid = true;
  seed.anchor_points = seed_path;
  seed.kind = core::SeedSpec::Kind::GUIDE_PATH_INIT;

  const int piece_num = static_cast<int>(seed.anchor_points.size()) - 1;
  seed.durations.resize(piece_num);
  for (int i = 0; i < piece_num; ++i)
  {
    const double seg_len =
        (seed.anchor_points[static_cast<std::size_t>(i + 1)] -
         seed.anchor_points[static_cast<std::size_t>(i)]).norm();
    seed.durations(i) = std::max(seg_len / std::max(0.1, context.max_vel), 0.2);
  }

  if (seed_times.size() == seed.anchor_points.size())
  {
    for (int i = 0; i < piece_num; ++i)
    {
      const double dt = std::max(
          0.05,
          seed_times[static_cast<std::size_t>(i + 1)] -
              seed_times[static_cast<std::size_t>(i)]);
      seed.durations(i) = 0.5 * seed.durations(i) + 0.5 * dt;
    }
  }

  const bool has_corridor = std::any_of(problem.feasible_sets.begin(),
                                        problem.feasible_sets.end(),
                                        [](const core::FeasibleSetSpec &set)
                                        {
                                          return set.enabled &&
                                                 set.type == core::FeasibleSetType::CORRIDOR_POLYTOPE &&
                                                 !set.corridor.empty();
                                        });
  if (has_corridor)
  {
    seed.kind = core::SeedSpec::Kind::CORRIDOR_INIT;
    seed.corridor_aware = true;
  }

  return seed.valid;
}

bool SeedBuilder::build(const core::PlanningContext &context,
                        const core::TaskDefinition &task_definition,
                        core::PlanningProblem &problem) const
{
  problem.seed = core::SeedSpec{};

  core::SeedSpec guide_seed;
  if (buildSeedFromGuide(context, problem, guide_seed))
  {
    problem.seed = guide_seed;
    if (problem.active_space_model == core::ActiveSpaceModel::PLAIN)
    {
      problem.seed.kind = core::SeedSpec::Kind::PLAIN_INIT;
    }
    else if (problem.active_space_model == core::ActiveSpaceModel::ESDF)
    {
      problem.seed.kind = core::SeedSpec::Kind::ESDF_INIT;
    }
    else if (problem.active_space_model == core::ActiveSpaceModel::CORRIDOR)
    {
      problem.seed.kind = core::SeedSpec::Kind::CORRIDOR_INIT;
      problem.seed.corridor_aware = true;
    }
    else if (problem.active_space_model == core::ActiveSpaceModel::VISIBLE_REGION)
    {
      problem.seed.kind = core::SeedSpec::Kind::SEMANTIC_INIT;
    }
  }

  if (context.allow_warm_start)
  {
    core::WarmStartCache warm_start;
    if (warm_start_service_.fetch(context, warm_start))
    {
      problem.context.warm_start = warm_start;
      if (problem.seed.valid && warm_start.durations.size() == problem.seed.durations.size())
      {
        problem.seed.kind = core::SeedSpec::Kind::WARM_START;
        problem.seed.durations = 0.65 * warm_start.durations.cwiseMax(0.03) +
                                 0.35 * problem.seed.durations.cwiseMax(0.03);
      }
    }
  }

  problem.variable_layout.piece_num =
      problem.seed.valid ? static_cast<int>(problem.seed.anchor_points.size()) - 1 : 0;
  problem.variable_layout.inner_point_num =
      std::max(0, problem.variable_layout.piece_num - 1);
  problem.variable_layout.boundary_derivative_num = MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM;

  return problem.seed.valid || task_definition.type != core::TaskType::STATE_TO_STATE;
}

} // namespace ego_planner::compiler
