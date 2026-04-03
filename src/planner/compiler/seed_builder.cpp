#include <compiler/seed_builder.hpp>

#include <algorithm>
#include <cmath>

#include <SpatialMap/CorridorInit.hpp>
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

bool applyWarmStartTimingProfile(const Eigen::VectorXd &warm_durations,
                                 Eigen::VectorXd &durations)
{
  if (warm_durations.size() <= 0 || durations.size() <= 0)
  {
    return false;
  }
  if (!warm_durations.allFinite() || !durations.allFinite())
  {
    return false;
  }

  const double warm_total = warm_durations.sum();
  const double init_total = durations.sum();
  if (warm_total <= 1.0e-6 || init_total <= 1.0e-6)
  {
    return false;
  }

  const Eigen::VectorXd warm_safe = warm_durations.cwiseMax(0.03);
  Eigen::VectorXd adjusted = durations.cwiseMax(0.03);

  if (warm_safe.size() == adjusted.size())
  {
    adjusted = 0.35 * adjusted + 0.65 * warm_safe;
  }
  else
  {
    const double scale = std::min(2.5, std::max(0.4, warm_total / init_total));
    adjusted *= scale;
  }

  const double adjusted_sum = adjusted.sum();
  if (adjusted_sum <= 1.0e-6)
  {
    return false;
  }

  adjusted *= warm_total / adjusted_sum;
  durations = adjusted.cwiseMax(0.03);
  return true;
}

bool assembleInitialGuessFromAnchors(const ego_planner::core::PlanningContext &context,
                                     const std::vector<Eigen::Vector3d> &anchors,
                                     Eigen::MatrixXd &inner_points,
                                     Eigen::VectorXd &durations)
{
  if (anchors.size() < 2)
  {
    return false;
  }

  std::vector<Eigen::Vector3d> expanded;
  expanded.reserve(anchors.size() * 2);
  expanded.push_back(anchors.front());

  const double piece_length = std::max(context.poly_piece_length, 0.2);
  for (std::size_t i = 1; i < anchors.size(); ++i)
  {
    const Eigen::Vector3d &p0 = anchors[i - 1];
    const Eigen::Vector3d &p1 = anchors[i];
    const double seg_len = (p1 - p0).norm();
    const int split_num = std::max(1, static_cast<int>(std::ceil(seg_len / piece_length)));
    for (int k = 1; k <= split_num; ++k)
    {
      const double ratio = static_cast<double>(k) / static_cast<double>(split_num);
      const Eigen::Vector3d pt = p0 + ratio * (p1 - p0);
      if ((pt - expanded.back()).norm() > 1.0e-4)
      {
        expanded.push_back(pt);
      }
    }
  }

  if (expanded.size() < 2)
  {
    return false;
  }

  if (expanded.size() == 2)
  {
    expanded.insert(expanded.begin() + 1, 0.5 * (expanded.front() + expanded.back()));
  }

  const int piece_num = static_cast<int>(expanded.size()) - 1;
  durations.resize(piece_num);
  inner_points.resize(3, std::max(0, piece_num - 1));

  for (int i = 0; i < piece_num; ++i)
  {
    const double seg_len = (expanded[static_cast<std::size_t>(i + 1)] -
                            expanded[static_cast<std::size_t>(i)])
                               .norm();
    durations(i) = std::max(seg_len / std::max(context.max_vel, 0.1), 0.2);
    if (i < piece_num - 1)
    {
      inner_points.col(i) = expanded[static_cast<std::size_t>(i + 1)];
    }
  }

  return durations.size() > 0;
}

bool buildGuideSeedImpl(const ego_planner::core::PlanningContext &context,
                        const ego_planner::core::PlanningProblem &problem,
                        ego_planner::core::SeedSpec &seed)
{
  if (problem.references.guide_path.size() < 2)
  {
    return false;
  }

  seed = ego_planner::core::SeedSpec{};
  seed.valid = true;
  seed.anchor_points = problem.references.guide_path;
  seed.kind = ego_planner::core::SeedSpec::Kind::GUIDE_PATH_INIT;
  return assembleInitialGuessFromAnchors(context,
                                         seed.anchor_points,
                                         seed.inner_points,
                                         seed.durations);
}

bool buildCorridorSeedImpl(const ego_planner::core::PlanningContext &context,
                           const ego_planner::core::PlanningProblem &problem,
                           ego_planner::core::SeedSpec &seed)
{
  const auto *corridor_set = findCorridorSet(problem);
  if (corridor_set == nullptr ||
      !problem.start_boundary.valid ||
      !problem.terminal_boundary.valid)
  {
    return false;
  }

  seed = ego_planner::core::SeedSpec{};
  seed.valid = true;
  seed.corridor_aware = true;
  seed.kind = ego_planner::core::SeedSpec::Kind::CORRIDOR_INIT;

  const double piece_length = std::max(context.poly_piece_length, 0.2);
  const double alloc_speed = std::max(0.9 * context.max_vel, 0.35);
  std::vector<Eigen::Vector3d> short_path;
  if (!spatial_map::buildCorridorInit(problem.start_boundary.position,
                                      problem.start_boundary.velocity,
                                      problem.terminal_boundary.position,
                                      corridor_set->corridor,
                                      piece_length,
                                      alloc_speed,
                                      seed.inner_points,
                                      seed.durations,
                                      nullptr,
                                      &short_path,
                                      &seed.corridor_piece_idx))
  {
    const std::vector<Eigen::Vector3d> &fallback_path =
        corridor_set->corridor_seed_path.size() >= 2
            ? corridor_set->corridor_seed_path
            : problem.references.guide_path;
    if (fallback_path.size() < 2 ||
        !assembleInitialGuessFromAnchors(context,
                                         fallback_path,
                                         seed.inner_points,
                                         seed.durations))
    {
      return false;
    }

    seed.anchor_points = fallback_path;
    seed.corridor_piece_idx = Eigen::VectorXi::Zero(static_cast<int>(corridor_set->corridor.size()));
    if (seed.corridor_piece_idx.size() == 1)
    {
      seed.corridor_piece_idx(0) = seed.durations.size();
    }
    else if (seed.corridor_piece_idx.size() > 1 && seed.durations.size() > 0)
    {
      for (int i = 0; i < seed.durations.size(); ++i)
      {
        const double ratio =
            (static_cast<double>(i) + 0.5) / static_cast<double>(seed.durations.size());
        const int poly_id =
            std::min(static_cast<int>(seed.corridor_piece_idx.size()) - 1,
                     std::max(0, static_cast<int>(std::floor(ratio * seed.corridor_piece_idx.size()))));
        seed.corridor_piece_idx(poly_id) += 1;
      }
    }
    return seed.durations.size() > 0;
  }

  if (seed.corridor_piece_idx.size() != static_cast<int>(corridor_set->corridor.size()) ||
      seed.corridor_piece_idx.sum() != seed.durations.size())
  {
    seed.corridor_piece_idx = Eigen::VectorXi::Zero(static_cast<int>(corridor_set->corridor.size()));
    if (seed.corridor_piece_idx.size() == 1)
    {
      seed.corridor_piece_idx(0) = seed.durations.size();
    }
    else if (seed.corridor_piece_idx.size() > 1 && seed.durations.size() > 0)
    {
      for (int i = 0; i < seed.durations.size(); ++i)
      {
        const double ratio =
            (static_cast<double>(i) + 0.5) / static_cast<double>(seed.durations.size());
        const int poly_id =
            std::min(static_cast<int>(seed.corridor_piece_idx.size()) - 1,
                     std::max(0, static_cast<int>(std::floor(ratio * seed.corridor_piece_idx.size()))));
        seed.corridor_piece_idx(poly_id) += 1;
      }
    }
  }

  if (short_path.size() >= 2)
  {
    seed.anchor_points = short_path;
  }
  else if (corridor_set->corridor_seed_path.size() >= 2)
  {
    seed.anchor_points = corridor_set->corridor_seed_path;
  }
  else
  {
    seed.anchor_points = problem.references.guide_path;
  }

  return seed.durations.size() > 0;
}

} // namespace

namespace ego_planner::compiler
{

bool SeedBuilder::buildGuideSeed(const core::PlanningContext &context,
                                 const core::PlanningProblem &problem,
                                 core::SeedSpec &seed) const
{
  if (!buildGuideSeedImpl(context, problem, seed))
  {
    return false;
  }

  if (problem.active_space_model == core::ActiveSpaceModel::PLAIN)
  {
    seed.kind = core::SeedSpec::Kind::PLAIN_INIT;
  }
  else if (problem.active_space_model == core::ActiveSpaceModel::ESDF)
  {
    seed.kind = core::SeedSpec::Kind::ESDF_INIT;
  }
  else if (problem.active_space_model == core::ActiveSpaceModel::VISIBLE_REGION)
  {
    seed.kind = core::SeedSpec::Kind::SEMANTIC_INIT;
  }
  return true;
}

bool SeedBuilder::buildTransitSeed(const core::PlanningContext &context,
                                   const core::PlanningProblem &problem,
                                   core::SeedSpec &seed) const
{
  switch (problem.active_space_model)
  {
  case core::ActiveSpaceModel::CORRIDOR:
    return buildCorridorSeedImpl(context, problem, seed);
  case core::ActiveSpaceModel::ESDF:
  case core::ActiveSpaceModel::PLAIN:
  default:
    return buildGuideSeed(context, problem, seed);
  }
}

bool SeedBuilder::build(const core::PlanningContext &context,
                        const core::TaskDefinition &task_definition,
                        core::PlanningProblem &problem) const
{
  problem.seed = core::SeedSpec{};

  core::SeedSpec guide_seed;
  bool seed_ok = false;
  switch (task_definition.type)
  {
  case core::TaskType::STATE_TO_STATE:
    seed_ok = buildTransitSeed(context, problem, guide_seed);
    break;
  case core::TaskType::TRACKING:
  case core::TaskType::PERCHING:
  case core::TaskType::UNKNOWN:
  default:
    seed_ok = buildGuideSeed(context, problem, guide_seed);
    break;
  }

  if (seed_ok)
  {
    problem.seed = guide_seed;
  }
  else if (task_definition.type == core::TaskType::STATE_TO_STATE)
  {
    // State-to-state seed is only an optional compile-time hint.
    problem.seed = core::SeedSpec{};
    seed_ok = true;
  }

  if (context.allow_warm_start)
  {
    core::WarmStartCache warm_start;
    if (warm_start_service_.fetch(context, warm_start))
    {
      problem.context.warm_start = warm_start;
      if (problem.seed.valid &&
          applyWarmStartTimingProfile(warm_start.durations, problem.seed.durations))
      {
        if (problem.active_space_model != core::ActiveSpaceModel::CORRIDOR)
        {
          problem.seed.kind = core::SeedSpec::Kind::WARM_START;
        }
      }
    }
  }

  problem.variable_layout.piece_num =
      problem.seed.valid ? static_cast<int>(problem.seed.durations.size()) : 0;
  problem.variable_layout.inner_point_num =
      problem.seed.valid ? static_cast<int>(problem.seed.inner_points.cols()) : 0;
  problem.variable_layout.boundary_derivative_num = MINCOTraj3D::BOUNDARY_DERIVATIVE_NUM;
  if (task_definition.type == core::TaskType::STATE_TO_STATE)
  {
    return true;
  }
  return problem.seed.valid;
}

} // namespace ego_planner::compiler
