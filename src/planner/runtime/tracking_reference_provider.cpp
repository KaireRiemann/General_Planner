#include <runtime/tracking_reference_provider.hpp>

#include <algorithm>
#include <cmath>
#include <functional>

namespace
{

using cost_functional::TrackingReference;

bool normalizeSeries(const std::vector<double> &t_in,
                     const std::vector<Eigen::Vector3d> &p_in,
                     const std::vector<Eigen::Vector3d> &v_in,
                     const double horizon_sec,
                     std::vector<double> &t_out,
                     std::vector<Eigen::Vector3d> &p_out,
                     std::vector<Eigen::Vector3d> &v_out,
                     const std::function<bool(double, Eigen::Vector3d &, Eigen::Vector3d &)> &sampler)
{
  if (t_in.empty() || p_in.empty() || t_in.size() != p_in.size())
  {
    return false;
  }
  if (!v_in.empty() && v_in.size() != p_in.size())
  {
    return false;
  }

  t_out.clear();
  p_out.clear();
  v_out.clear();
  t_out.reserve(t_in.size() + 1);
  p_out.reserve(p_in.size() + 1);
  v_out.reserve(p_in.size() + 1);

  const double t0 = t_in.front();
  double last_t = -1.0;
  bool inserted_horizon = false;

  for (std::size_t i = 0; i < t_in.size(); ++i)
  {
    const double rel_t_raw = std::max(0.0, t_in[i] - t0);
    if (rel_t_raw > horizon_sec + 1.0e-9)
    {
      break;
    }

    double rel_t = rel_t_raw;
    if (!t_out.empty() && rel_t <= last_t)
    {
      rel_t = last_t + 1.0e-3;
    }

    t_out.push_back(rel_t);
    p_out.push_back(p_in[i]);
    if (!v_in.empty())
    {
      v_out.push_back(v_in[i]);
    }
    else
    {
      Eigen::Vector3d vel = Eigen::Vector3d::Zero();
      if (i + 1 < p_in.size())
      {
        const double dt = std::max(1.0e-3, (t_in[i + 1] - t_in[i]));
        vel = (p_in[i + 1] - p_in[i]) / dt;
      }
      else if (i > 0)
      {
        const double dt = std::max(1.0e-3, (t_in[i] - t_in[i - 1]));
        vel = (p_in[i] - p_in[i - 1]) / dt;
      }
      v_out.push_back(vel);
    }
    last_t = rel_t;

    if (std::abs(rel_t - horizon_sec) <= 1.0e-3)
    {
      inserted_horizon = true;
    }
  }

  if (!inserted_horizon && !t_out.empty() && t_out.back() < horizon_sec - 1.0e-3)
  {
    Eigen::Vector3d p_h = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_h = Eigen::Vector3d::Zero();
    if (sampler(horizon_sec, p_h, v_h))
    {
      t_out.push_back(horizon_sec);
      p_out.push_back(p_h);
      v_out.push_back(v_h);
    }
  }

  if (t_out.size() < 2)
  {
    if (t_out.empty())
    {
      return false;
    }
    const double dt = std::max(0.05, std::min(0.5, horizon_sec));
    t_out.push_back(t_out.front() + dt);
    p_out.push_back(p_out.front() + v_out.front() * dt);
    v_out.push_back(v_out.front());
  }

  return t_out.size() == p_out.size() && p_out.size() == v_out.size();
}

} // namespace

namespace ego_planner::runtime
{

void TrackingReferenceProvider::configure(const double horizon_sec,
                                          const double sample_dt,
                                          const double max_speed)
{
  horizon_sec_ = std::max(0.4, horizon_sec);
  sample_dt_ = std::max(0.05, sample_dt);
  max_speed_ = std::max(0.1, max_speed);
}

bool TrackingReferenceProvider::buildFromPath(const nav_msgs::Path &msg,
                                              const double fallback_dt,
                                              cost_functional::TrackingReference &reference) const
{
  if (msg.poses.size() < 2)
  {
    return false;
  }

  TrackingReference raw;
  const std::size_t N = msg.poses.size();
  raw.t_ref.reserve(N);
  raw.p_ref.reserve(N);
  raw.v_ref.resize(N, Eigen::Vector3d::Zero());

  const double dt_default = std::max(0.02, fallback_dt);
  const double t0_stamp = msg.poses.front().header.stamp.toSec();
  double last_t = 0.0;
  for (std::size_t i = 0; i < N; ++i)
  {
    const auto &pose = msg.poses[i].pose.position;
    raw.p_ref.emplace_back(pose.x, pose.y, pose.z);

    double ti = static_cast<double>(i) * dt_default;
    const double stamp = msg.poses[i].header.stamp.toSec();
    if (t0_stamp > 1.0e-6 && stamp > 1.0e-6)
    {
      ti = std::max(0.0, stamp - t0_stamp);
    }
    if (i > 0 && ti <= last_t)
    {
      ti = last_t + dt_default;
    }
    raw.t_ref.push_back(ti);
    last_t = ti;
  }

  for (std::size_t i = 0; i + 1 < N; ++i)
  {
    const double dt = std::max(1.0e-3, raw.t_ref[i + 1] - raw.t_ref[i]);
    raw.v_ref[i] = (raw.p_ref[i + 1] - raw.p_ref[i]) / dt;
  }
  raw.v_ref.back() = raw.v_ref[N - 2];

  return normalize(raw, reference);
}

bool TrackingReferenceProvider::buildFromTargetOdom(const Eigen::Vector3d &target_pos,
                                                    const Eigen::Vector3d &target_vel,
                                                    cost_functional::TrackingReference &reference) const
{
  TrackingReference raw;
  const double horizon = std::max(0.2, horizon_sec_);
  const double dt = std::max(0.05, sample_dt_);
  const int sample_num =
      std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);

  Eigen::Vector3d clamped_vel = target_vel;
  const double speed = clamped_vel.norm();
  if (speed > max_speed_)
  {
    clamped_vel *= max_speed_ / speed;
  }

  raw.t_ref.reserve(static_cast<std::size_t>(sample_num));
  raw.p_ref.reserve(static_cast<std::size_t>(sample_num));
  raw.v_ref.reserve(static_cast<std::size_t>(sample_num));
  for (int i = 0; i < sample_num; ++i)
  {
    const double t = std::min(horizon, static_cast<double>(i) * dt);
    raw.t_ref.push_back(t);
    raw.p_ref.push_back(target_pos + clamped_vel * t);
    raw.v_ref.push_back(clamped_vel);
  }

  return normalize(raw, reference);
}

bool TrackingReferenceProvider::normalize(const TrackingReference &raw,
                                          TrackingReference &normalized) const
{
  if (!raw.valid())
  {
    return false;
  }
  return trimAndNormalize(raw, normalized);
}

bool TrackingReferenceProvider::trimAndNormalize(const TrackingReference &raw,
                                                 TrackingReference &normalized) const
{
  normalized = TrackingReference{};
  normalized.use_view_terminal = raw.use_view_terminal;
  normalized.has_terminal_ref = raw.has_terminal_ref;
  normalized.p_term_ref = raw.p_term_ref;
  normalized.v_term_ref = raw.v_term_ref;

  const auto target_sampler = [&](const double tq, Eigen::Vector3d &p, Eigen::Vector3d &v) -> bool
  {
    return cost_functional::sampleTrackingReference(raw, tq + raw.t_ref.front(), p, v);
  };

  if (!normalizeSeries(raw.t_ref,
                       raw.p_ref,
                       raw.v_ref,
                       horizon_sec_,
                       normalized.t_ref,
                       normalized.p_ref,
                       normalized.v_ref,
                       target_sampler))
  {
    return false;
  }

  if (raw.viewValid())
  {
    const auto view_sampler = [&](const double tq, Eigen::Vector3d &p, Eigen::Vector3d &v) -> bool
    {
      return cost_functional::sampleTrackingViewReference(raw, tq + raw.t_view_ref.front(), p, v);
    };
    normalizeSeries(raw.t_view_ref,
                    raw.p_view_ref,
                    raw.v_view_ref,
                    horizon_sec_,
                    normalized.t_view_ref,
                    normalized.p_view_ref,
                    normalized.v_view_ref,
                    view_sampler);
  }

  if (!normalized.valid())
  {
    return false;
  }

  if (!normalized.viewValid())
  {
    normalized.t_view_ref = normalized.t_ref;
    normalized.p_view_ref = normalized.p_ref;
    normalized.v_view_ref = normalized.v_ref;
  }

  if (!normalized.has_terminal_ref)
  {
    Eigen::Vector3d p_term = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_term = Eigen::Vector3d::Zero();
    if (cost_functional::sampleTrackingTerminalReference(normalized, p_term, v_term))
    {
      normalized.has_terminal_ref = true;
      normalized.p_term_ref = p_term;
      normalized.v_term_ref = v_term;
    }
  }

  return normalized.valid();
}

} // namespace ego_planner::runtime
