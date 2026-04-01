#pragma once

#include <plan_env/grid_map.h>
#include <Eigen/Eigen>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

namespace ego_planner
{

class JPSAStar
{
public:
  using Ptr = std::unique_ptr<JPSAStar>;

  explicit JPSAStar(const GridMap::Ptr &map, double safe_margin = 0.0)
      : map_(map), safe_margin_(safe_margin)
  {
  }

  void setTimeOut(double timeout_sec)
  {
    timeout_ms_ = static_cast<int>(std::max(0.01, timeout_sec) * 1000.0);
  }

  void setJumpMaxCells(int jump_max_cells)
  {
    jump_max_cells_ = std::max(1, jump_max_cells);
  }

  void setJumpNearObsRadius(int radius_cells)
  {
    jump_near_obs_radius_ = std::max(1, radius_cells);
  }

  bool search(const Eigen::Vector3d &start,
              const Eigen::Vector3d &goal,
              std::vector<Eigen::Vector3d> &path_out)
  {
    path_out.clear();
    if (!map_)
    {
      return false;
    }

    SearchContext ctx;
    ctx.low = map_->getUpdatedBoxLow();
    ctx.high = map_->getUpdatedBoxHigh();
    ctx.res = std::max(map_->getResolution(), 1.0e-3);
    ctx.size = ((ctx.high - ctx.low) / ctx.res).array().floor().cast<int>().matrix();
    ctx.size += Eigen::Vector3i::Ones();

    auto inside = [&](const Eigen::Vector3d &p) -> bool
    {
      return (p.array() >= ctx.low.array()).all() && (p.array() <= ctx.high.array()).all();
    };
    auto free_point = [&](const Eigen::Vector3d &p) -> bool
    {
      return inside(p) && map_->getInflateOccupancy(p) == 0;
    };

    if (!free_point(start) || !free_point(goal))
    {
      return false;
    }

    const Eigen::Vector3i start_idx = posToIndex(start, ctx.low, ctx.res);
    const Eigen::Vector3i goal_idx = posToIndex(goal, ctx.low, ctx.res);
    if (!inBounds(start_idx, ctx) || !inBounds(goal_idx, ctx))
    {
      return false;
    }

    if (!lineInCollision(start, goal))
    {
      path_out = {start, goal};
      return true;
    }

    const auto tic = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector3i> path_idx;

    if (!searchJumpAStar(start_idx, goal_idx, ctx, tic, path_idx))
    {
      if (!searchGridAStar(start_idx, goal_idx, ctx, tic, path_idx))
      {
        return false;
      }
    }

    std::vector<Eigen::Vector3d> raw_path;
    raw_path.reserve(path_idx.size());
    for (const auto &idx : path_idx)
    {
      raw_path.push_back(indexToPos(idx, ctx.low, ctx.res));
    }

    if (!raw_path.empty())
    {
      raw_path.front() = start;
      raw_path.back() = goal;
    }

    path_out = shortenPath(raw_path);
    return path_out.size() >= 2;
  }

private:
  struct SearchContext
  {
    Eigen::Vector3d low;
    Eigen::Vector3d high;
    Eigen::Vector3i size{Eigen::Vector3i::Zero()};
    double res{0.1};
  };

  struct Node
  {
    Eigen::Vector3i idx{Eigen::Vector3i::Zero()};
    double g{0.0};
    double f{0.0};
    int parent{-1};
  };

  struct PQNode
  {
    double f{0.0};
    int id{-1};
    bool operator<(const PQNode &other) const
    {
      return f > other.f;
    }
  };

  GridMap::Ptr map_;
  double safe_margin_{0.0};
  int timeout_ms_{200};
  int jump_max_cells_{6};
  int jump_near_obs_radius_{1};

  static std::vector<Eigen::Vector3i> &dirs26()
  {
    static std::vector<Eigen::Vector3i> dirs = []()
    {
      std::vector<Eigen::Vector3i> out;
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
            out.emplace_back(dx, dy, dz);
          }
        }
      }
      return out;
    }();
    return dirs;
  }

  static Eigen::Vector3i posToIndex(const Eigen::Vector3d &p,
                                    const Eigen::Vector3d &low,
                                    double res)
  {
    return ((p - low) / res).array().floor().cast<int>();
  }

  static Eigen::Vector3d indexToPos(const Eigen::Vector3i &idx,
                                    const Eigen::Vector3d &low,
                                    double res)
  {
    return low + (idx.cast<double>().array() + 0.5).matrix() * res;
  }

  static bool inBounds(const Eigen::Vector3i &idx, const SearchContext &ctx)
  {
    return (idx.array() >= 0).all() &&
           idx.x() < ctx.size.x() &&
           idx.y() < ctx.size.y() &&
           idx.z() < ctx.size.z();
  }

  std::int64_t encodeKey(const Eigen::Vector3i &idx) const
  {
    constexpr std::int64_t kMask = (1LL << 21) - 1;
    return ((static_cast<std::int64_t>(idx.x()) & kMask) << 42) |
           ((static_cast<std::int64_t>(idx.y()) & kMask) << 21) |
           (static_cast<std::int64_t>(idx.z()) & kMask);
  }

  bool isFreeIndex(const Eigen::Vector3i &idx, const SearchContext &ctx) const
  {
    if (!inBounds(idx, ctx))
    {
      return false;
    }
    const Eigen::Vector3d p = indexToPos(idx, ctx.low, ctx.res);
    return map_->getInflateOccupancy(p) == 0;
  }

  bool nearObstacle(const Eigen::Vector3i &idx, const SearchContext &ctx) const
  {
    for (int dx = -jump_near_obs_radius_; dx <= jump_near_obs_radius_; ++dx)
    {
      for (int dy = -jump_near_obs_radius_; dy <= jump_near_obs_radius_; ++dy)
      {
        for (int dz = -jump_near_obs_radius_; dz <= jump_near_obs_radius_; ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0)
          {
            continue;
          }
          const Eigen::Vector3i nb = idx + Eigen::Vector3i(dx, dy, dz);
          if (!inBounds(nb, ctx) || !isFreeIndex(nb, ctx))
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  bool timedOut(const std::chrono::steady_clock::time_point &tic) const
  {
    const auto now = std::chrono::steady_clock::now();
    const auto dt_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - tic).count();
    return dt_ms > timeout_ms_;
  }

  bool reconstructPath(const std::vector<Node> &nodes,
                       int goal_id,
                       std::vector<Eigen::Vector3i> &path_idx) const
  {
    path_idx.clear();
    if (goal_id < 0 || goal_id >= static_cast<int>(nodes.size()))
    {
      return false;
    }

    int cur = goal_id;
    while (cur >= 0)
    {
      path_idx.push_back(nodes[static_cast<std::size_t>(cur)].idx);
      cur = nodes[static_cast<std::size_t>(cur)].parent;
    }
    std::reverse(path_idx.begin(), path_idx.end());
    return path_idx.size() >= 2;
  }

  bool jumpAlongDir(const Eigen::Vector3i &cur_idx,
                    const Eigen::Vector3i &dir,
                    const Eigen::Vector3i &goal_idx,
                    const SearchContext &ctx,
                    Eigen::Vector3i &next_idx,
                    double &step_cost) const
  {
    Eigen::Vector3i idx = cur_idx;
    int step_cnt = 0;

    while (step_cnt < jump_max_cells_)
    {
      idx += dir;
      ++step_cnt;

      if (!isFreeIndex(idx, ctx))
      {
        break;
      }

      if (idx == goal_idx || nearObstacle(idx, ctx))
      {
        next_idx = idx;
        step_cost = static_cast<double>(step_cnt) * dir.cast<double>().norm() * ctx.res;
        return true;
      }
    }

    if (step_cnt > 0)
    {
      // Even without obstacle-triggered jump point, keep the furthest free point in
      // this direction to preserve long-range expansion in open space.
      const Eigen::Vector3i candidate = cur_idx + dir * std::max(1, step_cnt - (isFreeIndex(idx, ctx) ? 0 : 1));
      if (isFreeIndex(candidate, ctx))
      {
        next_idx = candidate;
        const int real_step = (candidate - cur_idx).cwiseAbs().maxCoeff();
        step_cost = static_cast<double>(real_step) * dir.cast<double>().norm() * ctx.res;
        return real_step > 0;
      }
    }
    return false;
  }

  bool searchJumpAStar(const Eigen::Vector3i &start_idx,
                       const Eigen::Vector3i &goal_idx,
                       const SearchContext &ctx,
                       const std::chrono::steady_clock::time_point &tic,
                       std::vector<Eigen::Vector3i> &path_idx) const
  {
    path_idx.clear();
    std::vector<Node> nodes;
    nodes.reserve(12000);
    std::priority_queue<PQNode> open_set;
    std::unordered_map<std::int64_t, double> best_g;

    auto heuristic = [&](const Eigen::Vector3i &a, const Eigen::Vector3i &b) -> double
    {
      return (a.cast<double>() - b.cast<double>()).norm() * ctx.res;
    };

    nodes.push_back(Node{start_idx, 0.0, heuristic(start_idx, goal_idx), -1});
    open_set.push(PQNode{nodes.back().f, 0});
    best_g.emplace(encodeKey(start_idx), 0.0);

    int goal_id = -1;
    while (!open_set.empty())
    {
      if (timedOut(tic))
      {
        return false;
      }

      const int cur_id = open_set.top().id;
      open_set.pop();
      const Node cur = nodes[static_cast<std::size_t>(cur_id)];

      const auto cur_key = encodeKey(cur.idx);
      const auto best_it = best_g.find(cur_key);
      if (best_it == best_g.end() || cur.g > best_it->second + 1.0e-9)
      {
        continue;
      }

      if (cur.idx == goal_idx)
      {
        goal_id = cur_id;
        break;
      }

      for (const auto &dir : dirs26())
      {
        Eigen::Vector3i succ_idx;
        double step_cost = 0.0;
        if (!jumpAlongDir(cur.idx, dir, goal_idx, ctx, succ_idx, step_cost))
        {
          continue;
        }

        const double tentative_g = cur.g + step_cost;
        const auto succ_key = encodeKey(succ_idx);
        auto succ_best = best_g.find(succ_key);
        if (succ_best == best_g.end() || tentative_g + 1.0e-9 < succ_best->second)
        {
          const double f = tentative_g + heuristic(succ_idx, goal_idx);
          const int nid = static_cast<int>(nodes.size());
          nodes.push_back(Node{succ_idx, tentative_g, f, cur_id});
          best_g[succ_key] = tentative_g;
          open_set.push(PQNode{f, nid});
        }
      }
    }

    if (goal_id < 0)
    {
      return false;
    }
    return reconstructPath(nodes, goal_id, path_idx);
  }

  bool searchGridAStar(const Eigen::Vector3i &start_idx,
                       const Eigen::Vector3i &goal_idx,
                       const SearchContext &ctx,
                       const std::chrono::steady_clock::time_point &tic,
                       std::vector<Eigen::Vector3i> &path_idx) const
  {
    path_idx.clear();
    std::vector<Node> nodes;
    nodes.reserve(20000);
    std::priority_queue<PQNode> open_set;
    std::unordered_map<std::int64_t, double> best_g;

    auto heuristic = [&](const Eigen::Vector3i &a, const Eigen::Vector3i &b) -> double
    {
      return (a.cast<double>() - b.cast<double>()).norm() * ctx.res;
    };

    nodes.push_back(Node{start_idx, 0.0, heuristic(start_idx, goal_idx), -1});
    open_set.push(PQNode{nodes.back().f, 0});
    best_g.emplace(encodeKey(start_idx), 0.0);

    int goal_id = -1;
    while (!open_set.empty())
    {
      if (timedOut(tic))
      {
        return false;
      }

      const int cur_id = open_set.top().id;
      open_set.pop();
      const Node cur = nodes[static_cast<std::size_t>(cur_id)];

      const auto cur_key = encodeKey(cur.idx);
      const auto best_it = best_g.find(cur_key);
      if (best_it == best_g.end() || cur.g > best_it->second + 1.0e-9)
      {
        continue;
      }

      if (cur.idx == goal_idx)
      {
        goal_id = cur_id;
        break;
      }

      for (const auto &d : dirs26())
      {
        const Eigen::Vector3i nxt_idx = cur.idx + d;
        if (!isFreeIndex(nxt_idx, ctx))
        {
          continue;
        }

        const double step_cost = d.cast<double>().norm() * ctx.res;
        const double tentative_g = cur.g + step_cost;
        const auto nxt_key = encodeKey(nxt_idx);

        auto it = best_g.find(nxt_key);
        if (it == best_g.end() || tentative_g + 1.0e-9 < it->second)
        {
          const double f = tentative_g + heuristic(nxt_idx, goal_idx);
          const int nid = static_cast<int>(nodes.size());
          nodes.push_back(Node{nxt_idx, tentative_g, f, cur_id});
          best_g[nxt_key] = tentative_g;
          open_set.push(PQNode{f, nid});
        }
      }
    }

    if (goal_id < 0)
    {
      return false;
    }
    return reconstructPath(nodes, goal_id, path_idx);
  }

  std::vector<Eigen::Vector3d> shortenPath(const std::vector<Eigen::Vector3d> &path) const
  {
    if (path.size() < 3 || !map_)
    {
      return path;
    }

    std::vector<Eigen::Vector3d> out;
    out.reserve(path.size());
    out.push_back(path.front());

    std::size_t i = 0;
    while (i + 1 < path.size())
    {
      std::size_t best = i + 1;
      for (std::size_t j = path.size() - 1; j > i + 1; --j)
      {
        if (!lineInCollision(path[i], path[j]))
        {
          best = j;
          break;
        }
      }
      out.push_back(path[best]);
      i = best;
    }

    return out;
  }

  bool lineInCollision(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
  {
    const double dist = (a - b).norm();
    const double step = std::max(map_->getResolution() * 0.5, 0.05);
    const int n = std::max(2, static_cast<int>(std::ceil(dist / step)));
    for (int i = 0; i <= n; ++i)
    {
      const double s = static_cast<double>(i) / static_cast<double>(n);
      const Eigen::Vector3d p = (1.0 - s) * a + s * b;
      if (map_->getInflateOccupancy(p) != 0)
      {
        return true;
      }
    }
    return false;
  }
};

} // namespace ego_planner
