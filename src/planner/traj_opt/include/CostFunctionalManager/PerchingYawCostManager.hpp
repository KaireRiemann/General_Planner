#ifndef PERCHING_YAW_COST_MANAGER_HPP
#define PERCHING_YAW_COST_MANAGER_HPP

#include "CostFunctionalManager/PerchingYawProjectionCost.hpp"
#include "traj_utils/minco_types.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace cost_functional
{

class PerchingYawCostFunctionalManager
{
public:
  using YawVec = Eigen::Matrix<double, 1, 1>;

  const ego_planner::SnapTraj3D *position_traj{nullptr};
  const minco::PerchingSemanticConfig *semantic_config{nullptr};
  PerchingProjectionCamera camera;
  PerchingYawProjectionConfig projection;

  double wei_yaw_rate{0.0};
  double wei_yaw_acc{0.0};
  double max_yaw_rate{1.2};
  double max_yaw_acc{3.0};
  double smooth_eps{0.03};
  mutable Eigen::VectorXd accumulated_costs;

  PerchingYawCostFunctionalManager()
  {
    accumulated_costs.resize(3);
    accumulated_costs.setZero();
  }

  void resetAccumulation() const
  {
    accumulated_costs.setZero();
  }

  bool enabled() const
  {
    return position_traj != nullptr && semantic_config != nullptr;
  }

  double evaluateIntegral(const int /*cp_idx*/,
                          const double /*t_local*/,
                          const double t_global,
                          const int /*seg_idx*/,
                          const int /*step_in_seg*/,
                          const YawVec &yaw,
                          const YawVec &yaw_rate,
                          const YawVec &yaw_acc,
                          const YawVec &yaw_jerk,
                          YawVec &grad_yaw,
                          YawVec &grad_yaw_rate,
                          YawVec &grad_yaw_acc,
                          YawVec &grad_yaw_jerk,
                          double &grad_time) const
  {
    (void)yaw_jerk;
    (void)grad_yaw_jerk;

    double cost_projection = 0.0;
    if (enabled())
    {
      const double total_t = position_traj->getTotalDuration();
      const double st = std::min(std::max(0.0, t_global), total_t);
      const Eigen::Vector3d position = position_traj->evaluate(st, 0);
      const Eigen::Vector3d velocity = position_traj->evaluate(st, 1);
      PerchingYawProjectionResult projection_result;
      cost_projection = evaluatePerchingYawProjectionCost(
          *semantic_config,
          camera,
          projection,
          st,
          position,
          velocity,
          yaw(0),
          projection_result);
      grad_yaw(0) += projection_result.grad_yaw;
      grad_time += projection_result.grad_time;
    }

    double cost_rate = 0.0;
    if (wei_yaw_rate > 0.0 && max_yaw_rate > 0.0)
    {
      cost_rate += accumulateAbsLimitCost(
          yaw_rate(0),
          max_yaw_rate,
          wei_yaw_rate,
          grad_yaw_rate(0));
    }

    double cost_acc = 0.0;
    if (wei_yaw_acc > 0.0 && max_yaw_acc > 0.0)
    {
      cost_acc += accumulateAbsLimitCost(
          yaw_acc(0),
          max_yaw_acc,
          wei_yaw_acc,
          grad_yaw_acc(0));
    }

    accumulated_costs(0) += cost_projection;
    accumulated_costs(1) += cost_rate;
    accumulated_costs(2) += cost_acc;
    return cost_projection + cost_rate + cost_acc;
  }

  template <typename SamplesType>
  double evaluateSample(const SamplesType &samples,
                        Eigen::Matrix<double, 1, Eigen::Dynamic> &grad_p,
                        Eigen::VectorXd &grad_t_global) const
  {
    grad_p.setZero(1, samples.size());
    grad_t_global.setZero(samples.size());
    return 0.0;
  }

private:
  double accumulateAbsLimitCost(const double value,
                                const double limit,
                                const double weight,
                                double &grad_value) const
  {
    const double abs_value = std::abs(value);
    const double violation = abs_value - limit;
    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!smoothedL1(violation, std::max(1.0e-6, smooth_eps), penalty, penalty_grad))
    {
      return 0.0;
    }
    const double sign = (value >= 0.0) ? 1.0 : -1.0;
    grad_value += weight * penalty_grad * sign;
    return weight * penalty;
  }
};

} // namespace cost_functional

#endif
