#ifndef CORRIDOR_INIT_HPP
#define CORRIDOR_INIT_HPP

#include "SpatialMap/SFCCommonTypes.hpp"
#include "SFCGenerator/geo_utils.hpp"
#include "optimizer/lbfgs.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace spatial_map
{

inline PolyhedronV makeVertexPoly(const Eigen::Matrix3Xd &vertices)
{
  PolyhedronV mapped;
  if (vertices.cols() <= 0)
  {
    return mapped;
  }

  mapped.resize(3, vertices.cols());
  mapped.col(0) = vertices.col(0);
  if (vertices.cols() > 1)
  {
    mapped.rightCols(vertices.cols() - 1) =
        vertices.rightCols(vertices.cols() - 1).colwise() - vertices.col(0);
  }
  return mapped;
}

inline bool buildOverlapVertexPolys(const PolyhedraH &corridor_hpolys,
                                    PolyhedraV &overlap_vpolys)
{
  overlap_vpolys.clear();
  if (corridor_hpolys.empty())
  {
    return false;
  }
  if (corridor_hpolys.size() == 1)
  {
    return true;
  }

  overlap_vpolys.reserve(corridor_hpolys.size() - 1);
  for (std::size_t i = 0; i + 1 < corridor_hpolys.size(); ++i)
  {
    Eigen::MatrixX4d overlap_h(corridor_hpolys[i].rows() + corridor_hpolys[i + 1].rows(), 4);
    overlap_h.topRows(corridor_hpolys[i].rows()) = corridor_hpolys[i];
    overlap_h.bottomRows(corridor_hpolys[i + 1].rows()) = corridor_hpolys[i + 1];

    Eigen::Matrix3Xd overlap_vertices;
    if (!geo_utils::enumerateVs(overlap_h, overlap_vertices) || overlap_vertices.cols() <= 0)
    {
      return false;
    }
    overlap_vpolys.emplace_back(makeVertexPoly(overlap_vertices));
  }

  return true;
}

inline Eigen::Vector3d overlapPointFromXi(const PolyhedronV &poly,
                                          const Eigen::VectorXd &xi)
{
  if (poly.cols() <= 0)
  {
    return Eigen::Vector3d::Zero();
  }
  if (poly.cols() == 1)
  {
    return poly.col(0);
  }

  const double norm = xi.norm();
  if (norm < 1.0e-12)
  {
    return poly.col(0);
  }

  const Eigen::VectorXd unit_xi = xi / norm;
  const Eigen::VectorXd r = unit_xi.head(poly.cols() - 1);
  return poly.rightCols(poly.cols() - 1) * r.cwiseProduct(r) + poly.col(0);
}

inline Eigen::VectorXd backwardGradByOverlapPoint(const PolyhedronV &poly,
                                                  const Eigen::VectorXd &xi,
                                                  const Eigen::Vector3d &grad_p)
{
  Eigen::VectorXd grad_xi = Eigen::VectorXd::Zero(poly.cols());
  if (poly.cols() <= 1)
  {
    return grad_xi;
  }

  const double norm = xi.norm();
  if (norm < 1.0e-12)
  {
    return grad_xi;
  }

  const double inv_norm = 1.0 / norm;
  const Eigen::VectorXd unit_xi = xi * inv_norm;

  Eigen::VectorXd grad_q(poly.cols());
  grad_q.head(poly.cols() - 1) =
      (poly.rightCols(poly.cols() - 1).transpose() * grad_p).array() *
      unit_xi.head(poly.cols() - 1).array() * 2.0;
  grad_q(poly.cols() - 1) = 0.0;

  grad_xi = (grad_q - unit_xi * unit_xi.dot(grad_q)) * inv_norm;
  return grad_xi;
}

inline void addNormPenalty(const Eigen::VectorXd &xi,
                           double &cost,
                           Eigen::Ref<Eigen::VectorXd> grad_xi)
{
  const double sqr_norm_violation = xi.squaredNorm() - 1.0;
  if (sqr_norm_violation <= 0.0)
  {
    return;
  }

  double c = sqr_norm_violation * sqr_norm_violation;
  cost += c * sqr_norm_violation;
  grad_xi += (3.0 * c) * 2.0 * xi;
}

struct ShortestPathProblem
{
  const PolyhedraV *overlap_vpolys{nullptr};
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  double smooth_eps{1.0e-3};
};

inline void buildDefaultOverlapCenters(const PolyhedraV &overlap_vpolys,
                                       std::vector<Eigen::Vector3d> &points)
{
  points.clear();
  points.reserve(overlap_vpolys.size());

  for (const auto &poly : overlap_vpolys)
  {
    if (poly.cols() <= 0)
    {
      points.emplace_back(Eigen::Vector3d::Zero());
      continue;
    }

    if (poly.cols() == 1)
    {
      points.emplace_back(poly.col(0));
      continue;
    }

    Eigen::VectorXd xi = Eigen::VectorXd::Constant(
        poly.cols(),
        std::sqrt(1.0 / static_cast<double>(poly.cols())));

    points.emplace_back(overlapPointFromXi(poly, xi));
  }
}

inline double shortestPathDistanceEval(void *instance,
                                       const double *x,
                                       double *g,
                                       const int n)
{
  auto *prob = reinterpret_cast<ShortestPathProblem *>(instance);
  std::fill(g, g + n, 0.0);

  if (prob == nullptr || prob->overlap_vpolys == nullptr)
  {
    return 0.0;
  }

  const int overlap_num = static_cast<int>(prob->overlap_vpolys->size());
  if (overlap_num <= 0)
  {
    return 0.0;
  }

  Eigen::Matrix3Xd points(3, overlap_num);
  Eigen::Matrix3Xd grad_points = Eigen::Matrix3Xd::Zero(3, overlap_num);

  int offset = 0;
  for (int i = 0; i < overlap_num; ++i)
  {
    const PolyhedronV &poly = (*prob->overlap_vpolys)[i];
    const int dof = poly.cols();
    if (offset + dof > n)
    {
      return 0.0;
    }

    const Eigen::Map<const Eigen::VectorXd> xi(x + offset, dof);
    points.col(i) = overlapPointFromXi(poly, xi);
    offset += dof;
  }

  double cost = 0.0;
  for (int i = 0; i <= overlap_num; ++i)
  {
    const Eigen::Vector3d a = (i == 0) ? prob->start : points.col(i - 1);
    const Eigen::Vector3d b = (i < overlap_num) ? points.col(i) : prob->goal;
    const Eigen::Vector3d d = b - a;

    const double smoothed_distance =
        std::sqrt(d.squaredNorm() + prob->smooth_eps * prob->smooth_eps);
    cost += smoothed_distance;

    const Eigen::Vector3d grad_segment = d / smoothed_distance;
    if (i < overlap_num)
    {
      grad_points.col(i) += grad_segment;
    }
    if (i > 0)
    {
      grad_points.col(i - 1) -= grad_segment;
    }
  }

  offset = 0;
  for (int i = 0; i < overlap_num; ++i)
  {
    const PolyhedronV &poly = (*prob->overlap_vpolys)[i];
    const int dof = poly.cols();
    const Eigen::Map<const Eigen::VectorXd> xi(x + offset, dof);
    Eigen::Map<Eigen::VectorXd> grad_xi(g + offset, dof);

    grad_xi = backwardGradByOverlapPoint(poly, xi, grad_points.col(i));
    addNormPenalty(xi, cost, grad_xi);
    offset += dof;
  }

  return cost;
}

inline bool solveShortestPathInCorridor(const Eigen::Vector3d &start_pt,
                                        const Eigen::Vector3d &goal_pt,
                                        const PolyhedraV &overlap_vpolys,
                                        std::vector<Eigen::Vector3d> &opt_points,
                                        const double smooth_eps = 1.0e-3)
{
  opt_points.clear();

  const int overlap_num = static_cast<int>(overlap_vpolys.size());
  if (overlap_num == 0)
  {
    return true;
  }

  int total_dim = 0;
  for (const auto &poly : overlap_vpolys)
  {
    total_dim += poly.cols();
  }
  if (total_dim <= 0)
  {
    return false;
  }

  std::vector<double> x(total_dim, 0.0);
  int offset = 0;
  for (const auto &poly : overlap_vpolys)
  {
    const int dof = poly.cols();
    const double init_value = std::sqrt(1.0 / static_cast<double>(std::max(dof, 1)));
    std::fill(x.begin() + offset, x.begin() + offset + dof, init_value);
    offset += dof;
  }

  ShortestPathProblem prob;
  prob.overlap_vpolys = &overlap_vpolys;
  prob.start = start_pt;
  prob.goal = goal_pt;
  prob.smooth_eps = smooth_eps;

  lbfgs::lbfgs_parameter_t param;
  lbfgs::lbfgs_load_default_parameters(&param);
  param.mem_size = 16;
  param.max_iterations = 64;
  param.max_linesearch = 40;
  param.g_epsilon = 1.0e-5;
  param.min_step = 1.0e-20;
  param.max_step = 1.0;
  param.past = 3;
  param.delta = 1.0e-3;

  double fx = 0.0;
  const int ret = lbfgs::lbfgs_optimize(
      total_dim,
      x.data(),
      &fx,
      shortestPathDistanceEval,
      nullptr,
      nullptr,
      &prob,
      &param);

  (void)fx;

  if (ret < 0)
  {
    return false;
  }

  opt_points.resize(overlap_num);
  offset = 0;
  for (int i = 0; i < overlap_num; ++i)
  {
    const PolyhedronV &poly = overlap_vpolys[i];
    const int dof = poly.cols();
    const Eigen::Map<const Eigen::VectorXd> xi(x.data() + offset, dof);
    opt_points[i] = overlapPointFromXi(poly, xi);
    offset += dof;

    if (!opt_points[i].allFinite())
    {
      return false;
    }
  }

  return true;
}

inline bool setInitialFromShortPath(const std::vector<Eigen::Vector3d> &short_path,
                                    const double piece_length,
                                    const double alloc_speed,
                                    Eigen::MatrixXd &inner_pts,
                                    Eigen::VectorXd &durations,
                                    Eigen::VectorXi *piece_idx_out = nullptr)
{
  if (short_path.size() < 2)
  {
    return false;
  }

  const int seg_num = static_cast<int>(short_path.size()) - 1;
  std::vector<int> piece_idx(seg_num, 1);

  int piece_num = 0;
  for (int i = 0; i < seg_num; ++i)
  {
    const double seg_len = (short_path[i + 1] - short_path[i]).norm();
    piece_idx[i] = std::max(1, static_cast<int>(std::floor(seg_len / piece_length)) + 1);
    piece_num += piece_idx[i];
  }

  durations.resize(piece_num);
  inner_pts.resize(3, std::max(0, piece_num - 1));

  int k = 0;
  int inner_col = 0;
  for (int i = 0; i < seg_num; ++i)
  {
    const Eigen::Vector3d a = short_path[i];
    const Eigen::Vector3d b = short_path[i + 1];
    const Eigen::Vector3d step = (b - a) / static_cast<double>(piece_idx[i]);
    const double dt = std::max(step.norm() / std::max(alloc_speed, 1.0e-3), 0.03);

    for (int j = 0; j < piece_idx[i]; ++j)
    {
      durations(k) = dt;

      // Keep the same layout as GCOPTER::setInitial:
      // for each corridor segment, insert sample "a + j * step",
      // except the global start point (segment 0, j = 0).
      if (!(i == 0 && j == 0) && inner_col < piece_num - 1)
      {
        inner_pts.col(inner_col++) = a + static_cast<double>(j) * step;
      }
      ++k;
    }
  }

  if (inner_col != piece_num - 1)
  {
    return false;
  }

  if (piece_idx_out != nullptr)
  {
    piece_idx_out->resize(seg_num);
    for (int i = 0; i < seg_num; ++i)
    {
      (*piece_idx_out)(i) = piece_idx[static_cast<std::size_t>(i)];
    }
  }

  return true;
}

inline bool buildCorridorInit(const Eigen::Vector3d &start_pt,
                                          const Eigen::Vector3d &goal_pt,
                                          const PolyhedraH &corridor_hpolys,
                                          const double piece_length,
                                          const double alloc_speed,
                                          Eigen::MatrixXd &inner_pts,
                                          Eigen::VectorXd &durations,
                                          std::vector<Eigen::Vector3d> *transition_points = nullptr,
                                          std::vector<Eigen::Vector3d> *short_path_out = nullptr,
                                          Eigen::VectorXi *piece_idx_out = nullptr)
{
  inner_pts.resize(3, 0);
  durations.resize(0);
  if (transition_points != nullptr)
  {
    transition_points->clear();
  }
  if (short_path_out != nullptr)
  {
    short_path_out->clear();
  }
  if (piece_idx_out != nullptr)
  {
    piece_idx_out->resize(0);
  }

  if (corridor_hpolys.empty())
  {
    return false;
  }

  if (corridor_hpolys.size() == 1)
  {
    const std::vector<Eigen::Vector3d> short_path{start_pt, goal_pt};
    if (short_path_out != nullptr)
    {
      *short_path_out = short_path;
    }
    return setInitialFromShortPath(short_path, piece_length, alloc_speed, inner_pts, durations, piece_idx_out);
  }

  PolyhedraV overlap_vpolys;
  if (!buildOverlapVertexPolys(corridor_hpolys, overlap_vpolys))
  {
    return false;
  }

  std::vector<Eigen::Vector3d> opt_points;
  if (!solveShortestPathInCorridor(start_pt, goal_pt, overlap_vpolys, opt_points))
  {
    buildDefaultOverlapCenters(overlap_vpolys, opt_points);
  }

  std::vector<Eigen::Vector3d> short_path;
  short_path.reserve(opt_points.size() + 2);
  short_path.push_back(start_pt);
  short_path.insert(short_path.end(), opt_points.begin(), opt_points.end());
  short_path.push_back(goal_pt);

  if (transition_points != nullptr)
  {
    *transition_points = opt_points;
  }
  if (short_path_out != nullptr)
  {
    *short_path_out = short_path;
  }

  return setInitialFromShortPath(short_path, piece_length, alloc_speed, inner_pts, durations, piece_idx_out);
}

// inline double shortestPathDistanceEval(void *instance,
//                                        const double *x,
//                                        double *g,
//                                        const int n)
// {
//   auto *prob = reinterpret_cast<ShortestPathProblem *>(instance);
//   std::fill(g, g + n, 0.0);

//   if (prob == nullptr || prob->overlap_vpolys == nullptr)
//   {
//     return 0.0;
//   }

//   const int overlap_num = static_cast<int>(prob->overlap_vpolys->size());
//   if (overlap_num <= 0)
//   {
//     return 0.0;
//   }

//   Eigen::Matrix3Xd points(3, overlap_num);
//   Eigen::Matrix3Xd grad_points = Eigen::Matrix3Xd::Zero(3, overlap_num);

//   int offset = 0;
//   for (int i = 0; i < overlap_num; ++i)
//   {
//     const PolyhedronV &poly = (*prob->overlap_vpolys)[i];
//     const int dof = poly.cols();
//     if (offset + dof > n)
//     {
//       return 0.0;
//     }

//     const Eigen::Map<const Eigen::VectorXd> xi(x + offset, dof);
//     points.col(i) = overlapPointFromXi(poly, xi);
//     offset += dof;
//   }

//   double cost = 0.0;
//   for (int i = 0; i <= overlap_num; ++i)
//   {
//     const Eigen::Vector3d a = (i == 0) ? prob->start : points.col(i - 1);
//     const Eigen::Vector3d b = (i < overlap_num) ? points.col(i) : prob->goal;
//     const Eigen::Vector3d d = b - a;
//     const double smoothed_distance = std::sqrt(d.squaredNorm() + prob->smooth_eps);
//     cost += smoothed_distance;

//     const Eigen::Vector3d grad_segment = d / smoothed_distance;
//     if (i < overlap_num)
//     {
//       grad_points.col(i) += grad_segment;
//     }
//     if (i > 0)
//     {
//       grad_points.col(i - 1) -= grad_segment;
//     }
//   }

//   offset = 0;
//   for (int i = 0; i < overlap_num; ++i)
//   {
//     const PolyhedronV &poly = (*prob->overlap_vpolys)[i];
//     const int dof = poly.cols();
//     const Eigen::Map<const Eigen::VectorXd> xi(x + offset, dof);
//     Eigen::Map<Eigen::VectorXd> grad_xi(g + offset, dof);

//     grad_xi = backwardGradByOverlapPoint(poly, xi, grad_points.col(i));
//     addNormPenalty(xi, cost, grad_xi);
//     offset += dof;
//   }

//   return cost;
// }

// inline bool solveShortestPathInCorridor(const Eigen::Vector3d &start_pt,
//                                         const Eigen::Vector3d &goal_pt,
//                                         const PolyhedraV &overlap_vpolys,
//                                         std::vector<Eigen::Vector3d> &opt_points,
//                                         const double smooth_eps = 1.0e-3)
// {
//   opt_points.clear();

//   const int overlap_num = static_cast<int>(overlap_vpolys.size());
//   if (overlap_num == 0)
//   {
//     return true;
//   }

//   int total_dim = 0;
//   for (const auto &poly : overlap_vpolys)
//   {
//     total_dim += poly.cols();
//   }
//   if (total_dim <= 0)
//   {
//     return false;
//   }

//   std::vector<double> x(total_dim, 0.0);
//   int offset = 0;
//   for (const auto &poly : overlap_vpolys)
//   {
//     const int dof = poly.cols();
//     const double init_value = std::sqrt(1.0 / static_cast<double>(std::max(dof, 1)));
//     std::fill(x.begin() + offset, x.begin() + offset + dof, init_value);
//     offset += dof;
//   }

//   ShortestPathProblem prob;
//   prob.overlap_vpolys = &overlap_vpolys;
//   prob.start = start_pt;
//   prob.goal = goal_pt;
//   prob.smooth_eps = smooth_eps;

//   lbfgs::lbfgs_parameter_t param;
//   lbfgs::lbfgs_load_default_parameters(&param);
//   param.mem_size = 16;
//   param.max_iterations = 64;
//   param.max_linesearch = 40;
//   param.g_epsilon = 1.0e-5;
//   param.min_step = 1.0e-20;
//   param.max_step = 1.0;
//   param.past = 3;
//   param.delta = 1.0e-3;

//   double fx = 0.0;
//   const int ret = lbfgs::lbfgs_optimize(
//       total_dim,
//       x.data(),
//       &fx,
//       shortestPathDistanceEval,
//       nullptr,
//       nullptr,
//       &prob,
//       &param);

//   (void)fx;

//   opt_points.resize(overlap_num);
//   offset = 0;
//   for (int i = 0; i < overlap_num; ++i)
//   {
//     const PolyhedronV &poly = overlap_vpolys[i];
//     const int dof = poly.cols();
//     const Eigen::Map<const Eigen::VectorXd> xi(x.data() + offset, dof);
//     opt_points[i] = overlapPointFromXi(poly, xi);
//     offset += dof;
//   }

//   return ret >= 0;
// }



// inline bool buildCorridorInit(const Eigen::Vector3d &start_pt,
//                                           const Eigen::Vector3d &goal_pt,
//                                           const PolyhedraH &corridor_hpolys,
//                                           const double piece_length,
//                                           const double alloc_speed,
//                                           Eigen::MatrixXd &inner_pts,
//                                           Eigen::VectorXd &durations,
//                                           std::vector<Eigen::Vector3d> *transition_points = nullptr,
//                                           std::vector<Eigen::Vector3d> *short_path_out = nullptr)
// {
//   inner_pts.resize(3, 0);
//   durations.resize(0);
//   if (transition_points != nullptr)
//   {
//     transition_points->clear();
//   }
//   if (short_path_out != nullptr)
//   {
//     short_path_out->clear();
//   }

//   if (corridor_hpolys.empty())
//   {
//     return false;
//   }

//   if (corridor_hpolys.size() == 1)
//   {
//     const std::vector<Eigen::Vector3d> short_path{start_pt, goal_pt};
//     if (short_path_out != nullptr)
//     {
//       *short_path_out = short_path;
//     }
//     return setInitialFromShortPath(short_path, piece_length, alloc_speed, inner_pts, durations);
//   }

//   PolyhedraV overlap_vpolys;
//   if (!buildOverlapVertexPolys(corridor_hpolys, overlap_vpolys))
//   {
//     return false;
//   }

//   std::vector<Eigen::Vector3d> opt_points;
//   solveShortestPathInCorridor(start_pt, goal_pt, overlap_vpolys, opt_points);

//   std::vector<Eigen::Vector3d> short_path;
//   short_path.reserve(opt_points.size() + 2);
//   short_path.push_back(start_pt);
//   short_path.insert(short_path.end(), opt_points.begin(), opt_points.end());
//   short_path.push_back(goal_pt);

//   if (transition_points != nullptr)
//   {
//     *transition_points = opt_points;
//   }
//   if (short_path_out != nullptr)
//   {
//     *short_path_out = short_path;
//   }

//   return setInitialFromShortPath(short_path, piece_length, alloc_speed, inner_pts, durations);
// }

} // namespace spatial_map

#endif
