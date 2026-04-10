#include <runtime/tracking_anchor_selector.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

Eigen::Vector2d normalizedOrDefault(const Eigen::Vector2d &v, const Eigen::Vector2d &fallback)
{
  if (v.norm() > 1.0e-6)
  {
    return v.normalized();
  }
  return fallback;
}

Eigen::Vector2d referenceHeadingAt(const cost_functional::TrackingReference &ref,
                                   const std::size_t idx,
                                   const Eigen::Vector2d &fallback)
{
  if (idx < ref.v_ref.size() && ref.v_ref[idx].head<2>().norm() > 0.1)
  {
    return ref.v_ref[idx].head<2>().normalized();
  }
  return fallback;
}

} // namespace

namespace ego_planner::runtime
{

void TrackingAnchorSelector::configure(const double tracking_d_min,
                                       const double tracking_d_max,
                                       const double side_angle_deg,
                                       const Eigen::Vector3d &preferred_relative_offset)
{
  tracking_d_min_ = std::max(0.2, tracking_d_min);
  tracking_d_max_ = std::max(tracking_d_min_ + 0.2, tracking_d_max);
  side_angle_deg_ = std::max(0.0, side_angle_deg);
  preferred_relative_offset_ = preferred_relative_offset;
  use_preferred_relative_offset_ = preferred_relative_offset_.norm() > 1.0e-3;
}

void TrackingAnchorSelector::reset()
{
  have_prev_dir_ = false;
  prev_dir_ = Eigen::Vector2d::UnitX();
}

Eigen::Vector2d TrackingAnchorSelector::rotate2D(const Eigen::Vector2d &dir,
                                                 const double angle_rad) const
{
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  return Eigen::Vector2d(c * dir.x() - s * dir.y(), s * dir.x() + c * dir.y());
}

Eigen::Vector2d TrackingAnchorSelector::chooseBaseDirection(const cost_functional::TrackingReference &target_reference,
                                                            const Eigen::Vector3d &ego_pos) const
{
  const Eigen::Vector2d fallback = have_prev_dir_ ? prev_dir_ : Eigen::Vector2d::UnitX();
  const Eigen::Vector3d target_now = target_reference.p_ref.front();

  Eigen::Vector2d minus_target_vel_xy = Eigen::Vector2d::Zero();
  if (!target_reference.v_ref.empty())
  {
    minus_target_vel_xy = (-target_reference.v_ref.front()).head<2>();
  }
  if (minus_target_vel_xy.norm() > 0.2)
  {
    return minus_target_vel_xy.normalized();
  }

  const Eigen::Vector2d ego_to_target_xy = (ego_pos - target_now).head<2>();
  if (ego_to_target_xy.norm() > 0.3)
  {
    return ego_to_target_xy.normalized();
  }

  return fallback;
}

double TrackingAnchorSelector::scoreDirection(const Eigen::Vector2d &candidate_dir,
                                              const Eigen::Vector2d &ego_to_target_dir,
                                              const Eigen::Vector2d &target_vel_dir,
                                              const double current_dist) const
{
  const double desired_dist = 0.5 * (tracking_d_min_ + tracking_d_max_);
  const double continuity_cost =
      have_prev_dir_ ? (1.0 - std::max(-1.0, std::min(1.0, candidate_dir.dot(prev_dir_)))) : 0.0;
  const double facing_cost =
      1.0 - std::max(-1.0, std::min(1.0, candidate_dir.dot(ego_to_target_dir)));
  const double against_target_vel_cost =
      std::max(0.0, candidate_dir.dot(target_vel_dir));
  const double distance_cost = std::abs(current_dist - desired_dist) / std::max(0.5, desired_dist);
  return 1.6 * continuity_cost + 1.0 * facing_cost + 0.6 * against_target_vel_cost + 0.25 * distance_cost;
}

bool TrackingAnchorSelector::buildAnchorReference(const cost_functional::TrackingReference &target_reference,
                                                  const Eigen::Vector3d &ego_pos,
                                                  const Eigen::Vector3d &ego_vel,
                                                  cost_functional::TrackingReference &planning_reference,
                                                  Eigen::Vector3d &terminal_anchor,
                                                  Eigen::Vector3d &terminal_vel)
{
  (void)ego_vel;
  if (!target_reference.valid())
  {
    return false;
  }

  const Eigen::Vector3d target_now = target_reference.p_ref.front();
  const Eigen::Vector2d ego_to_target_xy = normalizedOrDefault((ego_pos - target_now).head<2>(),
                                                               have_prev_dir_ ? prev_dir_ : Eigen::Vector2d::UnitX());

  Eigen::Vector2d target_vel_xy = Eigen::Vector2d::Zero();
  if (!target_reference.v_ref.empty())
  {
    target_vel_xy = target_reference.v_ref.front().head<2>();
  }
  const Eigen::Vector2d target_vel_dir = normalizedOrDefault(target_vel_xy, Eigen::Vector2d::Zero());

  const Eigen::Vector2d base_dir = chooseBaseDirection(target_reference, ego_pos);
  const double side_rad = side_angle_deg_ * M_PI / 180.0;
  std::vector<Eigen::Vector2d> candidates;
  candidates.reserve(3);
  candidates.push_back(base_dir);
  if (side_rad > 1.0e-3)
  {
    candidates.push_back(rotate2D(base_dir, side_rad));
    candidates.push_back(rotate2D(base_dir, -side_rad));
  }

  const double current_dist = (ego_pos - target_now).head<2>().norm();
  double best_score = std::numeric_limits<double>::infinity();
  Eigen::Vector2d best_dir = base_dir;
  for (const auto &cand : candidates)
  {
    const Eigen::Vector2d dir = normalizedOrDefault(cand, base_dir);
    const double score = scoreDirection(dir, ego_to_target_xy, target_vel_dir, current_dist);
    if (score < best_score)
    {
      best_score = score;
      best_dir = dir;
    }
  }

  const double desired_dist = 0.5 * (tracking_d_min_ + tracking_d_max_);
  planning_reference = target_reference;
  planning_reference.t_view_ref = planning_reference.t_ref;
  planning_reference.p_view_ref.resize(planning_reference.p_ref.size(), Eigen::Vector3d::Zero());
  planning_reference.v_view_ref.resize(planning_reference.p_ref.size(), Eigen::Vector3d::Zero());

  Eigen::Vector3d offset_vec(best_dir.x() * desired_dist,
                             best_dir.y() * desired_dist,
                             0.0);
  Eigen::Vector2d preferred_heading_xy = Eigen::Vector2d::UnitX();
  if (!planning_reference.v_ref.empty() &&
      planning_reference.v_ref.front().head<2>().norm() > 0.1)
  {
    preferred_heading_xy = planning_reference.v_ref.front().head<2>().normalized();
  }
  else if (have_prev_dir_ && prev_dir_.norm() > 1.0e-6)
  {
    // prev_dir_ is the target-to-drone offset direction. For rear-follow it is
    // opposite to the target heading, so invert it to recover a heading guess.
    preferred_heading_xy = -prev_dir_;
  }

  if (use_preferred_relative_offset_)
  {
    const Eigen::Vector2d lateral_xy(-preferred_heading_xy.y(), preferred_heading_xy.x());
    const Eigen::Vector2d rel_xy =
        preferred_heading_xy * preferred_relative_offset_.x() +
        lateral_xy * preferred_relative_offset_.y();
    offset_vec = Eigen::Vector3d(rel_xy.x(),
                                 rel_xy.y(),
                                 preferred_relative_offset_.z());
    if (rel_xy.norm() > 1.0e-6)
    {
      best_dir = rel_xy.normalized();
    }
  }
  for (std::size_t i = 0; i < planning_reference.p_ref.size(); ++i)
  {
    if (use_preferred_relative_offset_)
    {
      preferred_heading_xy = referenceHeadingAt(planning_reference, i, preferred_heading_xy);
      const Eigen::Vector2d lateral_xy(-preferred_heading_xy.y(), preferred_heading_xy.x());
      const Eigen::Vector2d rel_xy =
          preferred_heading_xy * preferred_relative_offset_.x() +
          lateral_xy * preferred_relative_offset_.y();
      offset_vec = Eigen::Vector3d(rel_xy.x(),
                                   rel_xy.y(),
                                   preferred_relative_offset_.z());
    }
    planning_reference.p_view_ref[i] = planning_reference.p_ref[i] + offset_vec;
    planning_reference.v_view_ref[i] =
        (i < planning_reference.v_ref.size()) ? planning_reference.v_ref[i] : Eigen::Vector3d::Zero();
  }

  planning_reference.use_view_terminal = true;
  planning_reference.has_terminal_ref = true;
  planning_reference.p_term_ref = planning_reference.p_view_ref.back();
  planning_reference.v_term_ref = planning_reference.v_view_ref.back();

  terminal_anchor = planning_reference.p_term_ref;
  terminal_vel = planning_reference.v_term_ref;
  if (!terminal_anchor.allFinite() || !terminal_vel.allFinite())
  {
    return false;
  }

  prev_dir_ = best_dir;
  have_prev_dir_ = true;
  return planning_reference.valid() && planning_reference.viewValid();
}

} // namespace ego_planner::runtime
