/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

#include <deque>
#include <memory>
#include <queue>
#include <unordered_map>
#include <limits>
#include <Eigen/Eigen>
#include <ros/ros.h>

namespace sfc_gen
{

    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        struct Node
        {
            Eigen::Vector3i idx;
            double g{std::numeric_limits<double>::infinity()};
            double f{std::numeric_limits<double>::infinity()};
            int parent{-1};
            bool closed{false};
        };

        struct QueueEntry
        {
            int node_id;
            double f;
            bool operator<(const QueueEntry &other) const
            {
                return f > other.f;
            }
        };

        const double resolution = std::max(mapPtr->getResolution(), 1.0e-3);
        const Eigen::Vector3d extent = (hb - lb).cwiseMax(Eigen::Vector3d::Constant(resolution));
        const Eigen::Array3i dims = (extent.array() / resolution).ceil().cast<int>() + 1;

        auto clampIdx = [&](const Eigen::Vector3i &idx) -> Eigen::Vector3i
        {
            return idx.cwiseMax(Eigen::Vector3i::Zero()).cwiseMin((dims - 1).matrix());
        };

        auto posToIdx = [&](const Eigen::Vector3d &pos) -> Eigen::Vector3i
        {
            const Eigen::Array3d local = ((pos - lb) / resolution).array().round();
            return clampIdx(local.cast<int>().matrix());
        };

        auto idxToPos = [&](const Eigen::Vector3i &idx) -> Eigen::Vector3d
        {
            return lb + idx.cast<double>() * resolution;
        };

        auto encode = [&](const Eigen::Vector3i &idx) -> std::int64_t
        {
            return (static_cast<std::int64_t>(idx.x()) << 42) ^
                   (static_cast<std::int64_t>(idx.y()) << 21) ^
                   static_cast<std::int64_t>(idx.z());
        };

        auto heuristic = [&](const Eigen::Vector3i &lhs, const Eigen::Vector3i &rhs) -> double
        {
            return (lhs.cast<double>() - rhs.cast<double>()).norm();
        };

        const Eigen::Vector3i start_idx = posToIdx(s);
        const Eigen::Vector3i goal_idx = posToIdx(g);
        if (mapPtr->query(idxToPos(start_idx)) != 0 || mapPtr->query(idxToPos(goal_idx)) != 0)
        {
            return INFINITY;
        }

        ros::Time start_time = ros::Time::now();
        std::vector<Node> nodes;
        nodes.reserve(4096);
        std::unordered_map<std::int64_t, int> node_ids;
        std::priority_queue<QueueEntry> open_set;

        nodes.push_back(Node{start_idx, 0.0, heuristic(start_idx, goal_idx), -1, false});
        node_ids.emplace(encode(start_idx), 0);
        open_set.push(QueueEntry{0, nodes.front().f});

        int goal_node_id = -1;
        while (!open_set.empty())
        {
            if ((ros::Time::now() - start_time).toSec() > timeout)
            {
                break;
            }

            const int current_id = open_set.top().node_id;
            open_set.pop();

            Node &current = nodes[static_cast<std::size_t>(current_id)];
            if (current.closed)
            {
                continue;
            }
            current.closed = true;

            if (current.idx == goal_idx)
            {
                goal_node_id = current_id;
                break;
            }

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

                        const Eigen::Vector3i next_idx = current.idx + Eigen::Vector3i(dx, dy, dz);
                        if ((next_idx.array() < 0).any() || (next_idx.array() >= dims).any())
                        {
                            continue;
                        }

                        const Eigen::Vector3d next_pos = idxToPos(next_idx);
                        if (mapPtr->query(next_pos) != 0)
                        {
                            continue;
                        }

                        const std::int64_t key = encode(next_idx);
                        auto iter = node_ids.find(key);
                        int next_id = -1;
                        if (iter == node_ids.end())
                        {
                            next_id = static_cast<int>(nodes.size());
                            node_ids.emplace(key, next_id);
                            nodes.push_back(Node{next_idx});
                        }
                        else
                        {
                            next_id = iter->second;
                        }

                        Node &next = nodes[static_cast<std::size_t>(next_id)];
                        if (next.closed)
                        {
                            continue;
                        }

                        const double step_cost = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
                        const double tentative_g = current.g + step_cost;
                        if (tentative_g + 1.0e-9 >= next.g)
                        {
                            continue;
                        }

                        next.g = tentative_g;
                        next.f = tentative_g + heuristic(next_idx, goal_idx);
                        next.parent = current_id;
                        open_set.push(QueueEntry{next_id, next.f});
                    }
                }
            }
        }

        if (goal_node_id < 0)
        {
            return INFINITY;
        }

        p.clear();
        for (int node_id = goal_node_id; node_id >= 0; node_id = nodes[static_cast<std::size_t>(node_id)].parent)
        {
            p.emplace_back(idxToPos(nodes[static_cast<std::size_t>(node_id)].idx));
        }
        std::reverse(p.begin(), p.end());
        if (!p.empty())
        {
            p.front() = s;
            p.back() = g;
        }

        return nodes[static_cast<std::size_t>(goal_node_id)].g * resolution;
    }

    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        if (path.size() < 2)
        {
            return;
        }

        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }

            if (valid_pc.empty())
            {
                hp = bd;
            }
            else
            {
                Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());
                firi::firi(bd, pc, a, b, hp);
            }

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    if (valid_pc.empty())
                    {
                        gap = bd;
                    }
                    else
                    {
                        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());
                        firi::firi(bd, pc, a, a, gap, 1);
                    }
                    hpolys.emplace_back(gap);
                }
            }

            if (hp.size() == 0)
            {
                hpolys.emplace_back(Eigen::MatrixX4d(bd));
            }
            else
            {
                hpolys.emplace_back(hp);
            }
        }
    }

    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            std::vector<Eigen::Vector3d> *corridor_path,
                            const double eps = 1.0e-6)
    {
        convexCover(path, points, lowCorner, highCorner, progress, range, hpolys, eps);

        if (corridor_path == nullptr)
        {
            return;
        }

        corridor_path->clear();
        if (path.empty())
        {
            return;
        }

        corridor_path->push_back(path.front());
        for (const auto &poly : hpolys)
        {
            Eigen::Vector3d inner;
            if (geo_utils::findInterior(poly, inner))
            {
                if ((inner - corridor_path->back()).norm() > eps)
                {
                    corridor_path->push_back(inner);
                }
            }
        }

        if ((path.back() - corridor_path->back()).norm() > eps)
        {
            corridor_path->push_back(path.back());
        }
    }

    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        if (hpolys.empty())
        {
            return;
        }

        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        const int M = htemp.size();
        bool overlap = false;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
