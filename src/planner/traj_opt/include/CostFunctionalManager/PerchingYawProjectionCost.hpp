#ifndef PERCHING_YAW_PROJECTION_COST_HPP
#define PERCHING_YAW_PROJECTION_COST_HPP

#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"
#include "MINCOTrajectory/terminal_mapping.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace cost_functional
{

struct PerchingProjectionCamera
{
  double fx{387.0};
  double fy{387.0};
  double cx{320.0};
  double cy{240.0};
  double width{640.0};
  double height{480.0};
  Eigen::Matrix3d R_bc{defaultForwardOpticalRbc()};
  Eigen::Vector3d p_bc{0.05, 0.0, 0.0};

  static Eigen::Matrix3d defaultForwardOpticalRbc()
  {
    Eigen::Matrix3d R;
    // Camera optical frame: z forward, x right, y down. Body frame: x forward,
    // y left, z up. R_bc maps camera coordinates into body coordinates.
    R << 0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
         0.0, -1.0, 0.0;
    return R;
  }

  bool valid() const
  {
    return fx > 1.0 && fy > 1.0 && width > 1.0 && height > 1.0 &&
           R_bc.allFinite() && p_bc.allFinite();
  }
};

struct PerchingYawProjectionConfig
{
  double weight{0.0};
  double center_weight_ratio{1.0};
  double boundary_weight_ratio{2.0};
  double depth_weight_ratio{4.0};
  double image_margin_ratio{0.08};
  double min_depth{0.20};
  double smooth_eps{0.03};
  double target_surface_offset{0.0};
};

struct PerchingYawProjectionResult
{
  double cost{0.0};
  double grad_yaw{0.0};
  double grad_time{0.0};
  Eigen::Vector3d grad_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d camera_point{Eigen::Vector3d::Zero()};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  bool projected{false};
};

inline Eigen::Matrix3d yawToWorldFromBody(const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d R;
  R << c, -s, 0.0,
       s,  c, 0.0,
       0.0, 0.0, 1.0;
  return R;
}

inline Eigen::Matrix3d dWorldFromBodyTransposeDYaw(const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d dRt;
  dRt << -s,  c, 0.0,
         -c, -s, 0.0,
          0.0, 0.0, 0.0;
  return dRt;
}

inline Eigen::Vector3d normalizedOrFallback(const Eigen::Vector3d &v,
                                            const Eigen::Vector3d &fallback)
{
  return (v.allFinite() && v.norm() > 1.0e-6) ? v.normalized() : fallback;
}

inline double accumulateSignedSmoothedPenalty(const double violation,
                                              const double smooth_eps,
                                              const double weight,
                                              const double sign,
                                              double &grad_scalar)
{
  if (weight <= 0.0)
  {
    return 0.0;
  }
  double penalty = 0.0;
  double penalty_grad = 0.0;
  if (!smoothedL1(violation, std::max(1.0e-6, smooth_eps), penalty, penalty_grad))
  {
    return 0.0;
  }
  grad_scalar += weight * penalty_grad * sign;
  return weight * penalty;
}

inline double evaluatePerchingYawProjectionCost(
    const minco::PerchingSemanticConfig &semantic,
    const PerchingProjectionCamera &camera,
    const PerchingYawProjectionConfig &config,
    const double t_global,
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &position_velocity,
    const double yaw,
    PerchingYawProjectionResult &result)
{
  result = PerchingYawProjectionResult{};
  if (config.weight <= 0.0 || !camera.valid() || !position.allFinite())
  {
    return 0.0;
  }

  const Eigen::Vector3d surface_z =
      normalizedOrFallback(semantic.surface_z, Eigen::Vector3d::UnitZ());
  const Eigen::Vector3d target =
      semantic.plate_position +
      semantic.plate_velocity * (t_global - semantic.reference_time) +
      config.target_surface_offset * surface_z;

  const Eigen::Matrix3d R_wb = yawToWorldFromBody(yaw);
  const Eigen::Matrix3d R_bw = R_wb.transpose();
  const Eigen::Matrix3d R_cb = camera.R_bc.transpose();
  const Eigen::Vector3d rel_w = target - position;
  const Eigen::Vector3d rel_b = R_bw * rel_w;
  const Eigen::Vector3d p_c = R_cb * (rel_b - camera.p_bc);

  result.target_position = target;
  result.camera_point = p_c;

  if (!p_c.allFinite())
  {
    return 0.0;
  }

  const double half_w = std::max(1.0, 0.5 * camera.width);
  const double half_h = std::max(1.0, 0.5 * camera.height);
  const double margin =
      std::min(0.90, std::max(0.0, config.image_margin_ratio));
  const double inside_limit = std::max(0.05, 1.0 - margin);
  const double smooth_eps = std::max(1.0e-6, config.smooth_eps);
  const double z_safe = std::max(p_c.z(), std::max(1.0e-3, config.min_depth));

  double cost = 0.0;
  Eigen::Vector3d grad_pc = Eigen::Vector3d::Zero();

  double penalty = 0.0;
  double penalty_grad = 0.0;
  if (smoothedL1(config.min_depth - p_c.z(), smooth_eps, penalty, penalty_grad))
  {
    const double w_depth = config.weight * std::max(0.0, config.depth_weight_ratio);
    cost += w_depth * penalty;
    grad_pc.z() -= w_depth * penalty_grad;
  }

  if (p_c.z() > std::max(1.0e-3, config.min_depth))
  {
    const double u = camera.fx * p_c.x() / z_safe + camera.cx;
    const double v = camera.fy * p_c.y() / z_safe + camera.cy;
    const double e_u = (u - camera.cx) / half_w;
    const double e_v = (v - camera.cy) / half_h;
    result.pixel = Eigen::Vector2d(u, v);
    result.projected = true;

    const Eigen::Vector3d du_dpc(camera.fx / z_safe,
                                 0.0,
                                 -camera.fx * p_c.x() / (z_safe * z_safe));
    const Eigen::Vector3d dv_dpc(0.0,
                                 camera.fy / z_safe,
                                 -camera.fy * p_c.y() / (z_safe * z_safe));
    const Eigen::Vector3d de_u_dpc = du_dpc / half_w;
    const Eigen::Vector3d de_v_dpc = dv_dpc / half_h;

    const double w_center = config.weight * std::max(0.0, config.center_weight_ratio);
    if (w_center > 0.0)
    {
      cost += 0.5 * w_center * (e_u * e_u + e_v * e_v);
      grad_pc += w_center * (e_u * de_u_dpc + e_v * de_v_dpc);
    }

    const double w_boundary = config.weight * std::max(0.0, config.boundary_weight_ratio);
    double grad_e_u = 0.0;
    cost += accumulateSignedSmoothedPenalty(
        std::abs(e_u) - inside_limit,
        smooth_eps,
        w_boundary,
        (e_u >= 0.0) ? 1.0 : -1.0,
        grad_e_u);
    double grad_e_v = 0.0;
    cost += accumulateSignedSmoothedPenalty(
        std::abs(e_v) - inside_limit,
        smooth_eps,
        w_boundary,
        (e_v >= 0.0) ? 1.0 : -1.0,
        grad_e_v);
    grad_pc += grad_e_u * de_u_dpc + grad_e_v * de_v_dpc;
  }

  const Eigen::Matrix3d target_to_camera = R_cb * R_bw;
  const Eigen::Matrix3d camera_to_world = target_to_camera.transpose();
  const Eigen::Vector3d grad_target = camera_to_world * grad_pc;
  result.grad_position = -grad_target;
  result.grad_time =
      grad_target.dot(semantic.plate_velocity) +
      result.grad_position.dot(position_velocity);

  const Eigen::Vector3d dpc_dyaw =
      R_cb * dWorldFromBodyTransposeDYaw(yaw) * rel_w;
  result.grad_yaw = grad_pc.dot(dpc_dyaw);
  result.cost = cost;
  return cost;
}

} // namespace cost_functional

#endif
