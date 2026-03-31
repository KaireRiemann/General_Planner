#pragma once

#include <plan_env/grid_map.h>
#include <Eigen/Eigen>
#include <queue>
#include <unordered_map>
#include <vector>
#include <memory>
#include <chrono>
#include <limits>
#include <cmath>
#include <algorithm>

namespace ego_planner {

class SimpleAStar {
public:
    using Ptr = std::unique_ptr<SimpleAStar>;

  explicit SimpleAStar(const GridMap::Ptr &map, double safe_margin = 0.0)
      : map_(map), safe_margin_(safe_margin) {}

  void setTimeOut(double timeout_sec) {
    timeout_ms_ = static_cast<int>(std::max(0.01, timeout_sec) * 1000.0);
  }

  bool search(const Eigen::Vector3d &start,
              const Eigen::Vector3d &goal,
              std::vector<Eigen::Vector3d> &path_out) {
    path_out.clear();
    if (!map_) return false;

    const Eigen::Vector3d low = map_->getUpdatedBoxLow();
    const Eigen::Vector3d high = map_->getUpdatedBoxHigh();
    const double res = std::max(map_->getResolution(), 1.0e-3);

    auto inside = [&](const Eigen::Vector3d &p) -> bool {
      return (p.array() >= low.array()).all() && (p.array() <= high.array()).all();
    };

    auto freePoint = [&](const Eigen::Vector3d &p) -> bool {
      if (!inside(p)) return false;
      return map_->getInflateOccupancy(p) == 0;
    };

    if (!freePoint(start) || !freePoint(goal)) {
      return false;
    }

    const Eigen::Vector3i start_idx = posToIndex(start, low, res);
    const Eigen::Vector3i goal_idx = posToIndex(goal, low, res);

    struct Node {
      Eigen::Vector3i idx;
      double g = 0.0;
      double f = 0.0;
      int parent = -1;
    };

    struct KeyHash {
      std::size_t operator()(const Eigen::Vector3i &k) const {
        std::size_t h1 = std::hash<int>()(k.x());
        std::size_t h2 = std::hash<int>()(k.y());
        std::size_t h3 = std::hash<int>()(k.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
      }
    };

    struct PQNode {
      double f;
      int id;
      bool operator<(const PQNode &other) const { return f > other.f; }
    };

    auto heuristic = [&](const Eigen::Vector3i &a, const Eigen::Vector3i &b) -> double {
      return (a.cast<double>() - b.cast<double>()).norm();
    };

    const auto tic = std::chrono::steady_clock::now();

    std::vector<Node> nodes;
    nodes.reserve(50000);

    std::priority_queue<PQNode> open_set;
    std::unordered_map<Eigen::Vector3i, int, KeyHash> best_id;
    std::unordered_map<Eigen::Vector3i, double, KeyHash> best_g;
    std::unordered_map<Eigen::Vector3i, bool, KeyHash> closed;

    nodes.push_back(Node{start_idx, 0.0, heuristic(start_idx, goal_idx), -1});
    open_set.push(PQNode{nodes.back().f, 0});
    best_id[start_idx] = 0;
    best_g[start_idx] = 0.0;

    std::vector<Eigen::Vector3i> neighbors;
    neighbors.reserve(26);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          neighbors.emplace_back(dx, dy, dz);
        }
      }
    }

    int goal_id = -1;

    while (!open_set.empty()) {
      const auto now = std::chrono::steady_clock::now();
      const auto dt_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - tic).count();
      if (dt_ms > timeout_ms_) {
        return false;
      }

      const int cur_id = open_set.top().id;
      open_set.pop();

      const Node &cur = nodes[cur_id];
      if (closed[cur.idx]) continue;
      closed[cur.idx] = true;

      if ((cur.idx - goal_idx).cwiseAbs().maxCoeff() <= 0) {
        goal_id = cur_id;
        break;
      }

      for (const auto &d : neighbors) {
        const Eigen::Vector3i nxt_idx = cur.idx + d;
        const Eigen::Vector3d nxt_pos = indexToPos(nxt_idx, low, res);

        if (!freePoint(nxt_pos)) continue;
        if (closed[nxt_idx]) continue;

        const double step_cost = d.cast<double>().norm() * res;
        const double tentative_g = cur.g + step_cost;

        auto it = best_g.find(nxt_idx);
        if (it == best_g.end() || tentative_g < it->second) {
          const double h = heuristic(nxt_idx, goal_idx) * res;
          const int new_id = static_cast<int>(nodes.size());
          nodes.push_back(Node{nxt_idx, tentative_g, tentative_g + h, cur_id});
          best_g[nxt_idx] = tentative_g;
          best_id[nxt_idx] = new_id;
          open_set.push(PQNode{nodes.back().f, new_id});
        }
      }
    }

    if (goal_id < 0) {
      return false;
    }

    std::vector<Eigen::Vector3d> path_rev;
    int cur = goal_id;
    while (cur >= 0) {
      path_rev.push_back(indexToPos(nodes[cur].idx, low, res));
      cur = nodes[cur].parent;
    }
    std::reverse(path_rev.begin(), path_rev.end());

    if (!path_rev.empty()) {
      path_rev.front() = start;
      path_rev.back() = goal;
    }

    path_out = shortenPath(path_rev);
    return path_out.size() >= 2;
  }

private:
  GridMap::Ptr map_;
  double safe_margin_{0.0};
  int timeout_ms_{200};

  static Eigen::Vector3i posToIndex(const Eigen::Vector3d &p,
                                    const Eigen::Vector3d &low,
                                    double res) {
    return ((p - low) / res).array().floor().cast<int>();
  }

  static Eigen::Vector3d indexToPos(const Eigen::Vector3i &idx,
                                    const Eigen::Vector3d &low,
                                    double res) {
    return low + (idx.cast<double>().array() + 0.5).matrix() * res;
  }

  std::vector<Eigen::Vector3d> shortenPath(const std::vector<Eigen::Vector3d> &path) const {
    if (path.size() < 3 || !map_) return path;

    std::vector<Eigen::Vector3d> out;
    out.push_back(path.front());

    std::size_t i = 0;
    while (i + 1 < path.size()) {
      std::size_t best = i + 1;
      for (std::size_t j = path.size() - 1; j > i + 1; --j) {
        if (!lineInCollision(path[i], path[j])) {
          best = j;
          break;
        }
      }
      out.push_back(path[best]);
      i = best;
    }

    return out;
  }

  bool lineInCollision(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {
    const double dist = (a - b).norm();
    const double step = std::max(map_->getResolution() * 0.5, 0.05);
    const int n = std::max(2, static_cast<int>(std::ceil(dist / step)));
    for (int i = 0; i <= n; ++i) {
      const double s = static_cast<double>(i) / static_cast<double>(n);
      const Eigen::Vector3d p = (1.0 - s) * a + s * b;
      if (map_->getInflateOccupancy(p) != 0) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace ego_planner