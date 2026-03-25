#pragma once

#include "TrajectoryOptAdapters/EgoPlanningTypesAdapter.hpp"

namespace traj_opt_adapters
{
class EgoVarianceSampleCostAdapter
{
public:
  using Types = EgoPlanningTypesAdapter;
  using SampleBuffer = ego_planner::SplineOpt::IntegralSampleBuffer;
  using SampleGradMatrix = ego_planner::SplineOpt::SampleGradMatrix;

  const Types::ConstraintPoints *cps{nullptr};
  int cps_per_piece{0};
  double weight{0.0};

  static void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                Eigen::MatrixXd &gdp,
                                                double &var,
                                                double weight)
  {
    const int point_num = ps.cols();
    const int edge_num = point_num - 1;

    gdp.resize(3, std::max(point_num, 0));
    gdp.setZero();
    var = 0.0;

    if (weight <= 0.0 || edge_num <= 0)
    {
      return;
    }

    const Eigen::MatrixXd dps = ps.rightCols(edge_num) - ps.leftCols(edge_num);
    const Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    const double dquarsum = dsqrs.squaredNorm();
    const double dquarmean = dquarsum / edge_num;
    var = weight * dquarmean;

    for (int i = 0; i <= edge_num; ++i)
    {
      if (i != 0)
        gdp.col(i) += weight * (4.0 * dsqrs(i - 1) / edge_num * dps.col(i - 1));
      if (i != edge_num)
        gdp.col(i) += weight * (-4.0 * dsqrs(i) / edge_num * dps.col(i));
    }
  }

  double operator()(const SampleBuffer &samples,
                    SampleGradMatrix &grad_p,
                    Eigen::VectorXd &grad_t_global) const
  {
    grad_p.resize(3, samples.size());
    grad_p.setZero();
    grad_t_global.resize(samples.size());
    grad_t_global.setZero();

    if (cps == nullptr || cps->cp_size <= 1 || samples.empty() || weight <= 0.0)
    {
      return 0.0;
    }

    Eigen::MatrixXd sampled_points(3, cps->cp_size);
    sampled_points.setZero();
    for (const auto &sample : samples)
    {
      const int control_point_index = sample.seg_idx * cps_per_piece + sample.step_in_seg;
      if (control_point_index >= 0 && control_point_index < cps->cp_size)
      {
        sampled_points.col(control_point_index) = sample.p;
      }
    }

    Eigen::MatrixXd gdp;
    double var_cost = 0.0;
    distanceSqrVarianceWithGradCost2p(sampled_points, gdp, var_cost, weight);

    for (Eigen::Index sample_idx = 0; sample_idx < static_cast<Eigen::Index>(samples.size()); ++sample_idx)
    {
      const auto &sample = samples[sample_idx];
      const int control_point_index = sample.seg_idx * cps_per_piece + sample.step_in_seg;
      if (control_point_index >= 0 && control_point_index < gdp.cols())
      {
        grad_p.col(sample_idx) = sample.trap_weight * gdp.col(control_point_index);
      }
    }

    return var_cost;
  }
};
} // namespace traj_opt_adapters
