#pragma once

#include <plan_env/grid_map.h>
#include <plan_env/raycast.h>
#include <path_searching/jps_a_star.hpp>

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ego_planner
{

class VisibleRegionGraph
{
public:
  using Ptr = std::unique_ptr<VisibleRegionGraph>;

  struct Config
  {
    double tracking_distance_min{1.5};
    double tracking_distance_max{4.0};
    double viewpoint_dt{0.6};
    int viewpoint_max_num{5};
    double viewpoint_yaw_step_deg{20.0};
    double viewpoint_connect_dist{1.5};
    double viewpoint_clearance{0.15};
    int radial_sample_num{3};
    int candidate_keep_topk{7};

    double path_timeout{0.08};
    int jps_jump_max_cells{6};
    int jps_near_obs_radius{1};

    double clearance_weight{1.15};
    double visibility_weight{1.10};
    double distance_weight{0.80};
    double sticky_weight{0.55};
    double motion_weight{0.30};
    double path_length_weight{0.16};
    double rel_align_weight{0.70};
    double los_edge_bonus{0.45};
    double non_los_edge_penalty{0.18};
  };

  struct Result
  {
    std::vector<Eigen::Vector3d> target_samples;
    std::vector<Eigen::Vector3d> viewpoint_series;
    std::vector<Eigen::Vector3d> viewpoint_target_vels;
    std::vector<double> viewpoint_times;
    std::vector<Eigen::Vector3d> guide_path;
    std::vector<Eigen::Vector3d> candidate_points;
  };

  explicit VisibleRegionGraph(const GridMap::Ptr &map = nullptr)
      : map_(map)
  {
    resetJps();
  }

  void setMap(const GridMap::Ptr &map)
  {
    map_ = map;
    resetJps();
  }

  void setConfig(const Config &cfg)
  {
    config_ = cfg;
    resetJps();
  }

  const Config &getConfig() const
  {
    return config_;
  }

  bool search(const std::vector<double> &t_ref,
              const std::vector<Eigen::Vector3d> &p_ref,
              const std::vector<Eigen::Vector3d> &v_ref,
              const Eigen::Vector3d &start_pt,
              const Eigen::Vector3d &start_vel,
              const Eigen::Vector3d *sticky_dir,
              Result &result) const
  {
    result = Result();

    if (!referenceValid(t_ref, p_ref, v_ref))
    {
      return false;
    }

    const double desired_dist = 0.5 * (config_.tracking_distance_min + config_.tracking_distance_max);
    const double horizon_end = std::max(0.0, t_ref.back());
    std::vector<double> sample_times;
    sample_times.reserve(static_cast<std::size_t>(config_.viewpoint_max_num));

    if (horizon_end < 1.0e-3)
    {
      sample_times.push_back(0.0);
    }
    else
    {
      const int desired_count =
          std::max(2,
                   std::min(config_.viewpoint_max_num,
                            static_cast<int>(std::ceil(horizon_end / std::max(config_.viewpoint_dt, 1.0e-2))) + 1));
      for (int i = 0; i < desired_count; ++i)
      {
        const double ratio =
            (desired_count <= 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(desired_count - 1);
        sample_times.push_back(ratio * horizon_end);
      }
    }

    std::sort(sample_times.begin(), sample_times.end());
    sample_times.erase(std::unique(sample_times.begin(), sample_times.end(),
                                   [](double a, double b)
                                   { return std::abs(a - b) < 1.0e-3; }),
                       sample_times.end());
    if (sample_times.empty())
    {
      return false;
    }

    Eigen::Vector3d sticky = (sticky_dir != nullptr) ? *sticky_dir : Eigen::Vector3d::UnitX();
    sticky.z() = 0.0;
    if (sticky.head<2>().norm() < 1.0e-3)
    {
      sticky = Eigen::Vector3d::UnitX();
    }
    sticky.normalize();

    Eigen::Vector3d init_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_target_vel = Eigen::Vector3d::Zero();
    sampleReference(t_ref, p_ref, v_ref, sample_times.front(), init_target, init_target_vel);

    Eigen::Vector3d init_seed_dir = start_pt - init_target;
    init_seed_dir.z() = 0.0;
    if (init_seed_dir.head<2>().norm() < 0.3)
    {
      init_seed_dir = -init_target_vel;
      init_seed_dir.z() = 0.0;
    }
    if (init_seed_dir.head<2>().norm() < 0.3)
    {
      init_seed_dir = -start_vel;
      init_seed_dir.z() = 0.0;
    }
    if (init_seed_dir.head<2>().norm() < 1.0e-3)
    {
      init_seed_dir = sticky;
    }
    init_seed_dir.normalize();

    struct Node
    {
      int id{-1};
      int layer{-1};
      int parent{-1};
      Eigen::Vector3d viewpoint{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_vel{Eigen::Vector3d::Zero()};
      double t{0.0};
      double base_score{-std::numeric_limits<double>::infinity()};
      double dp_score{-std::numeric_limits<double>::infinity()};
    };

    struct EdgeInfo
    {
      bool valid{false};
      bool los{false};
      double path_length{std::numeric_limits<double>::infinity()};
      std::vector<Eigen::Vector3d> path;
    };

    std::unordered_map<std::uint64_t, EdgeInfo> edge_cache;

    const auto edgeKey = [](int from_id, int to_id) -> std::uint64_t
    {
      return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(from_id)) << 32) |
             static_cast<std::uint32_t>(to_id);
    };

    const auto pathLength = [](const std::vector<Eigen::Vector3d> &path) -> double
    {
      double len = 0.0;
      for (std::size_t i = 1; i < path.size(); ++i)
      {
        len += (path[i] - path[i - 1]).norm();
      }
      return len;
    };

    const auto queryEdgePath = [&](int from_id,
                                   const Eigen::Vector3d &from,
                                   int to_id,
                                   const Eigen::Vector3d &to) -> const EdgeInfo &
    {
      const std::uint64_t key = edgeKey(from_id, to_id);
      auto it = edge_cache.find(key);
      if (it != edge_cache.end())
      {
        return it->second;
      }

      EdgeInfo edge;
      Eigen::Vector3d safe_from = from;
      Eigen::Vector3d safe_to = to;
      sanitizePoint(from, safe_from);
      sanitizePoint(to, safe_to);

      if ((safe_to - safe_from).norm() < 1.0e-3)
      {
        edge.valid = true;
        edge.los = true;
        edge.path = {from, to};
        edge.path_length = (to - from).norm();
        return edge_cache.emplace(key, edge).first->second;
      }

      if (!mapWindowReady() || lineOfSightFree(safe_from, safe_to))
      {
        edge.valid = true;
        edge.los = true;
        edge.path = {from, to};
        edge.path_length = (to - from).norm();
        return edge_cache.emplace(key, edge).first->second;
      }

      ensureJps();
      if (!jps_ || !jps_->search(safe_from, safe_to, edge.path) || edge.path.size() < 2)
      {
        return edge_cache.emplace(key, edge).first->second;
      }

      edge.path.front() = from;
      edge.path.back() = to;
      std::vector<Eigen::Vector3d> shortcut;
      shortcutPath(edge.path, shortcut);
      edge.path.swap(shortcut);
      edge.path_length = pathLength(edge.path);
      edge.valid = edge.path.size() >= 2;
      edge.los = false;
      return edge_cache.emplace(key, edge).first->second;
    };

    std::vector<std::vector<Node>> layers;
    layers.reserve(sample_times.size());
    int next_node_id = 1;
    Eigen::Vector3d prev_target = init_target;
    Eigen::Vector3d prev_seed_viewpoint = start_pt;
    bool have_prev_seed_viewpoint = true;

    const double yaw_step_rad =
        std::max(5.0, std::min(60.0, config_.viewpoint_yaw_step_deg)) * M_PI / 180.0;
    const int max_ring_id = std::max(4, static_cast<int>(std::ceil(M_PI / yaw_step_rad)));
    const int angle_sample_num = 2 * max_ring_id + 1;

    std::vector<double> radial_factors;
    radial_factors.reserve(static_cast<std::size_t>(std::max(1, config_.radial_sample_num)));
    if (config_.radial_sample_num <= 1)
    {
      radial_factors.push_back(1.0);
    }
    else
    {
      for (int i = 0; i < config_.radial_sample_num; ++i)
      {
        const double alpha = (config_.radial_sample_num <= 1)
                                 ? 0.5
                                 : static_cast<double>(i) / static_cast<double>(config_.radial_sample_num - 1);
        radial_factors.push_back(0.9 + 0.2 * alpha);
      }
    }

    for (std::size_t layer_idx = 0; layer_idx < sample_times.size(); ++layer_idx)
    {
      const double t_query = sample_times[layer_idx];
      Eigen::Vector3d target = Eigen::Vector3d::Zero();
      Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
      if (!sampleReference(t_ref, p_ref, v_ref, t_query, target, target_vel))
      {
        continue;
      }

      Eigen::Vector3d seed_dir =
          (layer_idx == 0 || !have_prev_seed_viewpoint) ? init_seed_dir : (prev_seed_viewpoint - target);
      seed_dir.z() = 0.0;
      if (seed_dir.head<2>().norm() < 0.3)
      {
        seed_dir = -target_vel;
        seed_dir.z() = 0.0;
      }
      if (seed_dir.head<2>().norm() < 0.3)
      {
        seed_dir = start_pt - target;
        seed_dir.z() = 0.0;
      }
      if (seed_dir.head<2>().norm() < 1.0e-3)
      {
        seed_dir = sticky;
      }
      seed_dir.normalize();

      const double seed_yaw = std::atan2(seed_dir.y(), seed_dir.x());
      std::vector<Node> layer_nodes;
      layer_nodes.reserve(static_cast<std::size_t>(angle_sample_num * radial_factors.size()));

      for (std::size_t ridx = 0; ridx < radial_factors.size(); ++ridx)
      {
        const double radius = desired_dist * radial_factors[ridx];
        for (int sample_id = 0; sample_id < angle_sample_num; ++sample_id)
        {
          const int ring_id = (sample_id == 0) ? 0 : ((sample_id + 1) / 2);
          const double yaw_offset = static_cast<double>(ring_id) * yaw_step_rad;
          const double yaw = seed_yaw + ((sample_id % 2 == 0) ? yaw_offset : -yaw_offset);

          Eigen::Vector3d candidate = target;
          candidate.x() += radius * std::cos(yaw);
          candidate.y() += radius * std::sin(yaw);
          candidate.z() = target.z();

          Eigen::Vector3d safe_candidate = candidate;
          if (!sanitizePoint(candidate, safe_candidate))
          {
            continue;
          }

          const Eigen::Vector3d rel = safe_candidate - target;
          const double radial_dist = rel.head<2>().norm();
          if (radial_dist < 0.8 * config_.tracking_distance_min ||
              radial_dist > 1.25 * config_.tracking_distance_max)
          {
            continue;
          }

          if (!lineOfSightFree(safe_candidate, target,
                               std::max(config_.tracking_distance_max + 0.5,
                                        desired_dist + 2.0 * std::max(config_.viewpoint_clearance,
                                                                      map_ ? 2.0 * map_->getResolution() : 0.2))))
          {
            continue;
          }

          bool duplicate = false;
          const double dedup_thresh = std::max(0.25, 1.5 * (map_ ? map_->getResolution() : 0.1));
          for (std::size_t j = 0; j < layer_nodes.size(); ++j)
          {
            if ((layer_nodes[j].viewpoint - safe_candidate).norm() < dedup_thresh)
            {
              duplicate = true;
              break;
            }
          }
          if (duplicate)
          {
            continue;
          }

          const double clearance =
              estimateClearance(safe_candidate,
                                std::max(config_.viewpoint_clearance,
                                         map_ ? 2.5 * map_->getResolution() : 0.2));
          const double vis_margin = rayVisibilityMargin(safe_candidate, target);
          const double radial_err = std::abs(radial_dist - desired_dist);
          const Eigen::Vector3d rel_dir = rel.normalized();
          const double sticky_align = rel_dir.dot(sticky);
          const double motion_align =
              (layer_idx == 0 || (target - prev_target).head<2>().norm() < 1.0e-3)
                  ? 0.0
                  : rel_dir.head<2>().dot((target - prev_target).head<2>().normalized());

          Node node;
          node.id = next_node_id++;
          node.layer = static_cast<int>(layers.size());
          node.viewpoint = safe_candidate;
          node.target = target;
          node.target_vel = target_vel;
          node.t = t_query;
          node.base_score =
              config_.clearance_weight * clearance +
              config_.visibility_weight * vis_margin -
              config_.distance_weight * radial_err +
              config_.sticky_weight * sticky_align +
              config_.motion_weight * motion_align;
          layer_nodes.push_back(node);
          result.candidate_points.push_back(node.viewpoint);
        }
      }

      if (!layer_nodes.empty())
      {
        std::sort(layer_nodes.begin(), layer_nodes.end(),
                  [](const Node &lhs, const Node &rhs)
                  { return lhs.base_score > rhs.base_score; });
        if (static_cast<int>(layer_nodes.size()) > config_.candidate_keep_topk)
        {
          layer_nodes.resize(static_cast<std::size_t>(config_.candidate_keep_topk));
        }
        prev_seed_viewpoint = layer_nodes.front().viewpoint;
        have_prev_seed_viewpoint = true;
        prev_target = target;
        layers.push_back(layer_nodes);
      }
    }

    if (layers.empty())
    {
      return false;
    }

    for (std::size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx)
    {
      std::vector<Node> &layer = layers[layer_idx];
      if (layer_idx == 0)
      {
        for (std::size_t i = 0; i < layer.size(); ++i)
        {
          Node &node = layer[i];
          const EdgeInfo &edge = queryEdgePath(0, start_pt, node.id, node.viewpoint);
          if (!edge.valid)
          {
            continue;
          }
          node.dp_score =
              node.base_score -
              config_.path_length_weight * edge.path_length +
              (edge.los ? config_.los_edge_bonus : -config_.non_los_edge_penalty);
        }
        continue;
      }

      const std::vector<Node> &prev_layer = layers[layer_idx - 1];
      for (std::size_t i = 0; i < layer.size(); ++i)
      {
        Node &node = layer[i];
        for (std::size_t j = 0; j < prev_layer.size(); ++j)
        {
          const Node &prev = prev_layer[j];
          if (!std::isfinite(prev.dp_score))
          {
            continue;
          }

          const EdgeInfo &edge = queryEdgePath(prev.id, prev.viewpoint, node.id, node.viewpoint);
          if (!edge.valid)
          {
            continue;
          }

          const Eigen::Vector3d prev_rel = prev.viewpoint - prev.target;
          const Eigen::Vector3d curr_rel = node.viewpoint - node.target;
          const double rel_align =
              (prev_rel.head<2>().norm() < 1.0e-3 || curr_rel.head<2>().norm() < 1.0e-3)
                  ? 0.0
                  : prev_rel.head<2>().normalized().dot(curr_rel.head<2>().normalized());

          const double transition_score =
              prev.dp_score +
              node.base_score +
              config_.rel_align_weight * rel_align -
              config_.path_length_weight * edge.path_length +
              (edge.los ? config_.los_edge_bonus : -config_.non_los_edge_penalty);

          if (transition_score > node.dp_score)
          {
            node.dp_score = transition_score;
            node.parent = static_cast<int>(j);
          }
        }
      }
    }

    int best_layer_idx = -1;
    int best_node_idx = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    for (int layer_idx = static_cast<int>(layers.size()) - 1; layer_idx >= 0; --layer_idx)
    {
      for (int i = 0; i < static_cast<int>(layers[static_cast<std::size_t>(layer_idx)].size()); ++i)
      {
        const double score = layers[static_cast<std::size_t>(layer_idx)][static_cast<std::size_t>(i)].dp_score;
        if (score > best_score)
        {
          best_score = score;
          best_layer_idx = layer_idx;
          best_node_idx = i;
        }
      }
      if (best_node_idx >= 0)
      {
        break;
      }
    }
    if (best_layer_idx < 0 || best_node_idx < 0)
    {
      return false;
    }

    std::vector<Node> sequence;
    while (best_layer_idx >= 0 && best_node_idx >= 0)
    {
      const Node &node = layers[static_cast<std::size_t>(best_layer_idx)][static_cast<std::size_t>(best_node_idx)];
      sequence.push_back(node);
      best_node_idx = node.parent;
      --best_layer_idx;
    }
    std::reverse(sequence.begin(), sequence.end());
    if (sequence.empty())
    {
      return false;
    }

    result.guide_path.clear();
    result.guide_path.push_back(start_pt);
    int prev_id = 0;
    Eigen::Vector3d prev_pt = start_pt;
    for (std::size_t i = 0; i < sequence.size(); ++i)
    {
      const Node &node = sequence[i];
      const EdgeInfo &edge = queryEdgePath(prev_id, prev_pt, node.id, node.viewpoint);
      if (!edge.valid || edge.path.size() < 2)
      {
        return false;
      }
      for (std::size_t k = 1; k < edge.path.size(); ++k)
      {
        if ((edge.path[k] - result.guide_path.back()).norm() > 1.0e-3)
        {
          result.guide_path.push_back(edge.path[k]);
        }
      }
      result.target_samples.push_back(node.target);
      result.viewpoint_series.push_back(node.viewpoint);
      result.viewpoint_target_vels.push_back(node.target_vel);
      result.viewpoint_times.push_back(node.t);
      prev_id = node.id;
      prev_pt = node.viewpoint;
    }

    std::vector<Eigen::Vector3d> shortcut;
    shortcutPath(result.guide_path, shortcut);
    result.guide_path.swap(shortcut);

    return result.viewpoint_series.size() >= 1 && result.guide_path.size() >= 2;
  }

private:
  GridMap::Ptr map_;
  mutable std::unique_ptr<JPSAStar> jps_;
  Config config_;

  void resetJps()
  {
    if (!map_)
    {
      jps_.reset();
      return;
    }
    jps_.reset(new JPSAStar(map_, 0.0));
    jps_->setTimeOut(config_.path_timeout);
    jps_->setJumpMaxCells(config_.jps_jump_max_cells);
    jps_->setJumpNearObsRadius(config_.jps_near_obs_radius);
  }

  void ensureJps() const
  {
    if (!jps_ && map_)
    {
      jps_.reset(new JPSAStar(map_, 0.0));
      jps_->setTimeOut(config_.path_timeout);
      jps_->setJumpMaxCells(config_.jps_jump_max_cells);
      jps_->setJumpNearObsRadius(config_.jps_near_obs_radius);
    }
  }

  static bool referenceValid(const std::vector<double> &t_ref,
                             const std::vector<Eigen::Vector3d> &p_ref,
                             const std::vector<Eigen::Vector3d> &v_ref)
  {
    if (t_ref.empty() || p_ref.empty() || t_ref.size() != p_ref.size())
    {
      return false;
    }
    if (!v_ref.empty() && v_ref.size() != p_ref.size())
    {
      return false;
    }
    for (std::size_t i = 0; i < t_ref.size(); ++i)
    {
      if (!std::isfinite(t_ref[i]) || !p_ref[i].allFinite())
      {
        return false;
      }
      if (!v_ref.empty() && !v_ref[i].allFinite())
      {
        return false;
      }
      if (i > 0 && t_ref[i] + 1.0e-9 < t_ref[i - 1])
      {
        return false;
      }
    }
    return true;
  }

  static bool sampleReference(const std::vector<double> &t_ref,
                              const std::vector<Eigen::Vector3d> &p_ref,
                              const std::vector<Eigen::Vector3d> &v_ref,
                              const double t_query,
                              Eigen::Vector3d &p_out,
                              Eigen::Vector3d &v_out)
  {
    if (!referenceValid(t_ref, p_ref, v_ref))
    {
      return false;
    }

    if (t_query <= t_ref.front())
    {
      p_out = p_ref.front();
      if (!v_ref.empty())
      {
        v_out = v_ref.front();
      }
      else if (t_ref.size() >= 2)
      {
        const double dt = std::max(1.0e-6, t_ref[1] - t_ref[0]);
        v_out = (p_ref[1] - p_ref[0]) / dt;
      }
      else
      {
        v_out.setZero();
      }
      return true;
    }

    if (t_query >= t_ref.back())
    {
      p_out = p_ref.back();
      if (!v_ref.empty())
      {
        v_out = v_ref.back();
      }
      else if (t_ref.size() >= 2)
      {
        const std::size_t n = t_ref.size();
        const double dt = std::max(1.0e-6, t_ref[n - 1] - t_ref[n - 2]);
        v_out = (p_ref[n - 1] - p_ref[n - 2]) / dt;
      }
      else
      {
        v_out.setZero();
      }
      return true;
    }

    const auto upper = std::upper_bound(t_ref.begin(), t_ref.end(), t_query);
    const std::size_t idx1 = static_cast<std::size_t>(std::distance(t_ref.begin(), upper));
    const std::size_t idx0 = idx1 - 1;
    const double t0 = t_ref[idx0];
    const double t1 = t_ref[idx1];
    const double dt = std::max(1.0e-6, t1 - t0);
    const double alpha = std::max(0.0, std::min(1.0, (t_query - t0) / dt));
    p_out = (1.0 - alpha) * p_ref[idx0] + alpha * p_ref[idx1];
    if (!v_ref.empty())
    {
      v_out = (1.0 - alpha) * v_ref[idx0] + alpha * v_ref[idx1];
    }
    else
    {
      v_out = (p_ref[idx1] - p_ref[idx0]) / dt;
    }
    return true;
  }

  bool mapWindowReady() const
  {
    if (!map_)
    {
      return false;
    }
    const Eigen::Vector3d low = map_->getUpdatedBoxLow();
    const Eigen::Vector3d high = map_->getUpdatedBoxHigh();
    if (!low.allFinite() || !high.allFinite())
    {
      return false;
    }
    const double res = std::max(map_->getResolution(), 1.0e-3);
    return ((high - low).array() > 6.0 * res).all();
  }

  bool isFree(const Eigen::Vector3d &pt) const
  {
    if (!map_ || !mapWindowReady())
    {
      return true;
    }
    return map_->getInflateOccupancy(pt) == 0;
  }

  Eigen::Vector3d clampToMap(const Eigen::Vector3d &pt) const
  {
    if (!mapWindowReady())
    {
      return pt;
    }
    const double res = std::max(map_->getResolution(), 1.0e-3);
    const Eigen::Vector3d low = map_->getUpdatedBoxLow() + Eigen::Vector3d::Constant(2.0 * res);
    const Eigen::Vector3d high = map_->getUpdatedBoxHigh() - Eigen::Vector3d::Constant(2.0 * res);
    return pt.cwiseMax(low).cwiseMin(high);
  }

  bool sanitizePoint(const Eigen::Vector3d &raw, Eigen::Vector3d &safe) const
  {
    safe = clampToMap(raw);
    if (!map_ || !mapWindowReady() || isFree(safe))
    {
      return true;
    }

    const double res = std::max(map_->getResolution(), 1.0e-3);
    const Eigen::Vector3d low = map_->getUpdatedBoxLow() + Eigen::Vector3d::Constant(2.0 * res);
    const Eigen::Vector3d high = map_->getUpdatedBoxHigh() - Eigen::Vector3d::Constant(2.0 * res);
    const int max_step = std::max(4, static_cast<int>(std::ceil(std::max(config_.tracking_distance_max, 1.5) / res)));
    for (int ring = 1; ring <= max_step; ++ring)
    {
      bool found = false;
      double best_score = -std::numeric_limits<double>::infinity();
      Eigen::Vector3d best_pt = safe;
      for (int dx = -ring; dx <= ring; ++dx)
      {
        for (int dy = -ring; dy <= ring; ++dy)
        {
          for (int dz = -ring; dz <= ring; ++dz)
          {
            if (std::max(std::max(std::abs(dx), std::abs(dy)), std::abs(dz)) != ring)
            {
              continue;
            }

            const Eigen::Vector3d cand =
                (safe + Eigen::Vector3d(dx, dy, dz) * res).cwiseMax(low).cwiseMin(high);
            if (!isFree(cand))
            {
              continue;
            }

            const double clearance = estimateClearance(cand, std::max(config_.viewpoint_clearance, 3.0 * res));
            const double score = clearance - 0.1 * (cand - raw).norm();
            if (!found || score > best_score)
            {
              found = true;
              best_score = score;
              best_pt = cand;
            }
          }
        }
      }

      if (found)
      {
        safe = best_pt;
        return true;
      }
    }

    return false;
  }

  double estimateClearance(const Eigen::Vector3d &pt, const double search_radius) const
  {
    if (!map_ || !mapWindowReady())
    {
      return search_radius;
    }

    if (map_->esdfEnabled())
    {
      return std::max(0.0, map_->getDistance(pt));
    }

    static const std::vector<Eigen::Vector3d> dirs = []()
    {
      std::vector<Eigen::Vector3d> out;
      out.reserve(26);
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dz = -1; dz <= 1; ++dz)
          {
            if (dx == 0 && dy == 0 && dz == 0)
            {
              continue;
            }
            out.push_back(Eigen::Vector3d(dx, dy, dz).normalized());
          }
        }
      }
      return out;
    }();

    const double res = std::max(map_->getResolution(), 1.0e-3);
    if (map_->getInflateOccupancy(pt) != 0)
    {
      return 0.0;
    }

    double clearance = std::max(search_radius, res);
    for (double radius = res; radius <= search_radius + 1.0e-6; radius += res)
    {
      for (std::size_t i = 0; i < dirs.size(); ++i)
      {
        if (map_->getInflateOccupancy(pt + dirs[i] * radius) != 0)
        {
          return std::max(0.0, radius - res);
        }
      }
    }
    return clearance;
  }

  bool lineOfSightFree(const Eigen::Vector3d &from,
                       const Eigen::Vector3d &to,
                       const double max_dist = -1.0) const
  {
    if (!map_ || !mapWindowReady())
    {
      return true;
    }

    const double dist = (to - from).norm();
    if (max_dist > 0.0 && dist > max_dist)
    {
      return false;
    }

    const Eigen::Vector3d low = map_->getUpdatedBoxLow();
    const Eigen::Vector3d high = map_->getUpdatedBoxHigh();
    if ((from.array() < low.array()).any() || (from.array() > high.array()).any() ||
        (to.array() < low.array()).any() || (to.array() > high.array()).any())
    {
      return false;
    }

    if (dist < 1.0e-6)
    {
      return isFree(from);
    }

    const double res = std::max(map_->getResolution(), 1.0e-3);
    RayCaster ray_caster;
    if (!ray_caster.setInput((from - low) / res, (to - low) / res))
    {
      return isFree(from) && isFree(to);
    }

    Eigen::Vector3d ray_idx;
    while (ray_caster.step(ray_idx))
    {
      const Eigen::Vector3d world_pt = low + (ray_idx.array() + 0.5).matrix() * res;
      if (map_->getInflateOccupancy(world_pt) != 0)
      {
        return false;
      }
    }
    return isFree(to);
  }

  double rayVisibilityMargin(const Eigen::Vector3d &from,
                             const Eigen::Vector3d &to) const
  {
    if (!map_ || !mapWindowReady())
    {
      return config_.viewpoint_clearance;
    }

    const double dist = (to - from).norm();
    if (dist < 1.0e-6)
    {
      return estimateClearance(from, std::max(config_.viewpoint_clearance, 0.1));
    }

    const double res = std::max(map_->getResolution(), 1.0e-3);
    const int sample_num = std::max(2, static_cast<int>(std::ceil(dist / res)));
    double min_margin = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= sample_num; ++i)
    {
      const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
      const Eigen::Vector3d pt = from + alpha * (to - from);
      const double margin = map_->esdfEnabled()
                                ? std::max(0.0, map_->getDistance(pt))
                                : estimateClearance(pt, std::max(config_.viewpoint_clearance, 2.0 * res));
      min_margin = std::min(min_margin, margin);
    }
    if (!std::isfinite(min_margin))
    {
      return 0.0;
    }
    return min_margin;
  }

  void shortcutPath(const std::vector<Eigen::Vector3d> &raw_path,
                    std::vector<Eigen::Vector3d> &shortcut_path) const
  {
    shortcut_path.clear();
    if (raw_path.empty())
    {
      return;
    }
    shortcut_path.push_back(raw_path.front());
    std::size_t anchor_idx = 0;
    while (anchor_idx + 1 < raw_path.size())
    {
      std::size_t next_idx = anchor_idx + 1;
      for (std::size_t cand = raw_path.size(); cand-- > anchor_idx + 1;)
      {
        if (lineOfSightFree(raw_path[anchor_idx], raw_path[cand]))
        {
          next_idx = cand;
          break;
        }
      }
      if ((raw_path[next_idx] - shortcut_path.back()).norm() > 1.0e-3)
      {
        shortcut_path.push_back(raw_path[next_idx]);
      }
      anchor_idx = next_idx;
    }
  }
};

} // namespace ego_planner
