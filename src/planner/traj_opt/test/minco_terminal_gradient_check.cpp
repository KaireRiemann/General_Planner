#include "MINCOTrajectory/MINCOOptimizer.hpp"
#include "MINCOTrajectory/MINCOTrajectory.hpp"
#include "MINCOTrajectory/terminal_mapping.hpp"
#include "SpatialMap/IdentityMap.hpp"
#include "TemporalMap/IdentityTimeMap.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{

struct ZeroTimeCost
{
  double operator()(const std::vector<double> &T, Eigen::VectorXd &gdT) const
  {
    gdT.setZero(static_cast<Eigen::Index>(T.size()));
    return 0.0;
  }
};

struct ZeroCostManager
{
  using Optimizer = minco::MINCOOptimizer<3, 3, temporal_map::IdentityTimeMap, spatial_map::IdentitySpatialMap<3>>;

  double evaluateIntegral(int,
                          double,
                          double,
                          int,
                          int,
                          const Eigen::Vector3d &,
                          const Eigen::Vector3d &,
                          const Eigen::Vector3d &,
                          const Eigen::Vector3d &,
                          Eigen::Vector3d &gp,
                          Eigen::Vector3d &gv,
                          Eigen::Vector3d &ga,
                          Eigen::Vector3d &gj,
                          double &gt) const
  {
    gp.setZero();
    gv.setZero();
    ga.setZero();
    gj.setZero();
    gt = 0.0;
    return 0.0;
  }

  double evaluateSample(const Optimizer::SampleBuffer &samples,
                        Eigen::Matrix<double, 3, Eigen::Dynamic> &sample_grad_p,
                        Eigen::VectorXd &sample_grad_t_global) const
  {
    sample_grad_p.setZero(3, static_cast<Eigen::Index>(samples.size()));
    sample_grad_t_global.setZero(static_cast<Eigen::Index>(samples.size()));
    return 0.0;
  }
};

bool approxEqual(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, const double tol)
{
  return a.rows() == b.rows() &&
         a.cols() == b.cols() &&
         (a - b).cwiseAbs().maxCoeff() <= tol;
}

} // namespace

int main()
{
  using Traj = minco::MINCOTrajectory<3, 3>;
  using Optimizer = minco::MINCOOptimizer<3, 3, temporal_map::IdentityTimeMap, spatial_map::IdentitySpatialMap<3>>;

  Traj::BoundaryState head = Traj::BoundaryState::Zero();
  Traj::BoundaryState tail = Traj::BoundaryState::Zero();
  head.col(0) << 0.0, 0.1, 0.2;
  head.col(1) << 0.2, 0.0, 0.0;
  head.col(2) << 0.0, 0.0, 0.0;
  tail.col(0) << 2.0, 0.5, 1.0;
  tail.col(1) << 0.1, -0.1, 0.0;
  tail.col(2) << 0.0, 0.0, 0.0;

  Traj::InnerPointsMat inner(3, 1);
  inner.col(0) << 0.8, 0.2, 0.7;
  Eigen::VectorXd T(2);
  T << 1.1, 0.9;

  Traj traj;
  if (!traj.generate(inner, head, tail, T))
  {
    std::cerr << "Failed to generate MINCO trajectory." << std::endl;
    return 1;
  }

  Traj::CoeffMat gdC = Traj::CoeffMat::Random(traj.getPieceNum() * Traj::COEFF_NUM, 3);
  Eigen::VectorXd gdT = Eigen::VectorXd::Random(traj.getPieceNum());

  Traj::InnerPointsMat grad_points_old;
  Eigen::VectorXd grad_times_old;
  traj.propagateGrad(gdC, gdT, grad_points_old, grad_times_old);

  Traj::InnerPointsMat grad_points_full;
  Eigen::VectorXd grad_times_full;
  Traj::BoundaryState grad_head = Traj::BoundaryState::Zero();
  Traj::BoundaryState grad_tail = Traj::BoundaryState::Zero();
  Traj::CoeffMat adjoint;
  traj.propagateGradFull(gdC,
                         gdT,
                         grad_points_full,
                         grad_times_full,
                         grad_head,
                         grad_tail,
                         &adjoint);

  if (!approxEqual(grad_points_old, grad_points_full, 1.0e-12) ||
      !approxEqual(grad_times_old, grad_times_full, 1.0e-12))
  {
    std::cerr << "Full gradient changed legacy point/time gradients." << std::endl;
    return 2;
  }

  for (int r = 0; r < Traj::S_ORDER; ++r)
  {
    if ((grad_head.col(r) - adjoint.row(r).transpose()).norm() > 1.0e-12)
    {
      std::cerr << "Head-state gradient does not match adjoint row " << r << "." << std::endl;
      return 3;
    }
    const int tail_row = traj.getPieceNum() * Traj::COEFF_NUM - Traj::S_ORDER + r;
    if ((grad_tail.col(r) - adjoint.row(tail_row).transpose()).norm() > 1.0e-12)
    {
      std::cerr << "Tail-state gradient does not match adjoint row " << tail_row << "." << std::endl;
      return 4;
    }
  }

  Optimizer opt;
  opt.setEnergyWeight(1.0);
  opt.setSamplesPerPiece(3);
  Optimizer::WaypointsType waypoints(3, 3);
  waypoints.row(0) = head.col(0).transpose();
  waypoints.row(1) = inner.col(0).transpose();
  waypoints.row(2) = tail.col(0).transpose();
  if (!opt.setInitState(std::vector<double>{1.1, 0.9}, waypoints, head, tail))
  {
    std::cerr << "Failed to set optimizer init state." << std::endl;
    return 5;
  }

  Eigen::VectorXd x = opt.generateInitialGuess();
  Eigen::VectorXd grad_fixed = Eigen::VectorXd::Zero(x.size());
  Eigen::VectorXd grad_noop = Eigen::VectorXd::Zero(x.size());
  ZeroTimeCost time_cost;
  ZeroCostManager cost_manager;
  minco::FixedTerminalMapping<3, 3> fixed_mapping;

  const double cost_fixed = opt.evaluate(x, grad_fixed, time_cost, cost_manager);
  const double cost_noop = opt.evaluateWithTerminalMapping(x, grad_noop, time_cost, cost_manager, &fixed_mapping);
  if (std::abs(cost_fixed - cost_noop) > 1.0e-12 ||
      (grad_fixed - grad_noop).cwiseAbs().maxCoeff() > 1.0e-12)
  {
    std::cerr << "FixedTerminalMapping changed optimizer output." << std::endl;
    return 6;
  }

  std::cout << "MINCO terminal-gradient self-check passed." << std::endl;
  return 0;
}
