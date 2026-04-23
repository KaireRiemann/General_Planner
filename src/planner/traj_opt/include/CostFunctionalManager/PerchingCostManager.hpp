#ifndef PERCHING_COST_MANAGER_HPP
#define PERCHING_COST_MANAGER_HPP

#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "CostFunctionalManager/CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/CorridorObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/DistanceFieldObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoFeasibilityPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/EgoSwarmPenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/GuidePointObstaclePenalty.hpp"
#include "CostFunctionalManager/CostFunctional/SpatialCosts/VarianceSampleCost.hpp"
#include "MINCOTrajectory/terminal_mapping.hpp"
#include "SpatialMap/SFCCommonTypes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cost_functional
{

class PerchingCostFunctionalManager
{
public:
    using Types = cost_functional::PlanningTypesAdapter;
    using PolyhedraH = spatial_map::PolyhedraH;
    using ReferencePoints = Eigen::Matrix<double, 3, Eigen::Dynamic>;
    using PerchingSemanticConfig = minco::PerchingSemanticConfig;

    enum SpatialMode
    {
        SPATIAL_PLAIN = 0,
        SPATIAL_ESDF,
        SPATIAL_CORRIDOR
    };

    Types::GridMapPtr grid_map;
    Types::ConstraintPoints *cps;
    Types::SwarmTrajData *swarm_traj;
    const PolyhedraH *h_polys;
    const Eigen::VectorXi *segment_poly_idx;
    const ReferencePoints *reference_points;
    const PerchingSemanticConfig *semantic_config;

    SpatialMode spatial_mode;
    double wei_obs, wei_obs_soft, wei_dist, wei_corridor, wei_corridor_ref;
    double wei_swarm, wei_feas, wei_sqrvar;
    double wei_perch_floor, wei_perch_thrust, wei_perch_omega, wei_perch_collision, wei_perch_height;
    double obs_clearance, obs_clearance_soft, safe_margin;
    double corridor_clearance, corridor_smoothing, swarm_clearance;
    double max_vel, max_acc, max_jer;
    double thrust_min, thrust_max, omega_max;
    double robot_radius, platform_radius, floor_height;
    double relative_height_min, relative_height_max;
    bool mask_platform_from_esdf;
    int drone_id;
    double t_now;
    bool touch_goal;
    mutable std::vector<double> *min_ellip_dist2_ptr;
    mutable Eigen::VectorXd accumulated_costs;

    PerchingCostFunctionalManager()
        : grid_map(nullptr), cps(nullptr), swarm_traj(nullptr),
          h_polys(nullptr), segment_poly_idx(nullptr), reference_points(nullptr), semantic_config(nullptr),
          spatial_mode(SPATIAL_PLAIN),
          wei_obs(0.0), wei_obs_soft(0.0), wei_dist(0.0), wei_corridor(0.0), wei_corridor_ref(0.0),
          wei_swarm(0.0), wei_feas(0.0), wei_sqrvar(0.0),
          wei_perch_floor(0.0), wei_perch_thrust(0.0), wei_perch_omega(0.0), wei_perch_collision(0.0), wei_perch_height(0.0),
          obs_clearance(0.0), obs_clearance_soft(0.0), safe_margin(0.0),
          corridor_clearance(0.0), corridor_smoothing(0.05), swarm_clearance(0.0),
          max_vel(0.0), max_acc(0.0), max_jer(0.0),
          thrust_min(0.0), thrust_max(0.0), omega_max(0.0),
          robot_radius(0.15), platform_radius(0.50), floor_height(0.1),
          relative_height_min(0.05), relative_height_max(3.0), mask_platform_from_esdf(true),
          drone_id(-1), t_now(0.0), touch_goal(false), min_ellip_dist2_ptr(nullptr)
    {
        accumulated_costs.resize(9);
        accumulated_costs.setZero();
    }

    void setSpatialMode(const SpatialMode mode)
    {
        spatial_mode = mode;
    }

    void setCorridor(const PolyhedraH *polys, const Eigen::VectorXi *indices)
    {
        h_polys = polys;
        segment_poly_idx = indices;
    }

    void setReferencePoints(const ReferencePoints *refs, double weight)
    {
        reference_points = refs;
        wei_corridor_ref = std::max(0.0, weight);
    }

    void setPerchingSemanticConfig(const PerchingSemanticConfig *config)
    {
        semantic_config = config;
    }

    void resetAccumulation() const
    {
        accumulated_costs.setZero();
    }

    double evaluateIntegral(const int cp_idx,
                            const double /*t_local*/,
                            const double t_global,
                            const int seg_idx,
                            const int /*step_in_seg*/,
                            const Types::Vec3 &position,
                            const Types::Vec3 &velocity,
                            const Types::Vec3 &acceleration,
                            const Types::Vec3 &jerk,
                            Types::Vec3 &grad_position,
                            Types::Vec3 &grad_velocity,
                            Types::Vec3 &grad_acceleration,
                            Types::Vec3 &grad_jerk,
                            double &grad_time) const
    {
        const double cost_feasibility =
            cost_functional::accumulateFeasibilityPenalty(
                velocity, acceleration, jerk,
                max_vel, max_acc, max_jer,
                wei_feas, grad_velocity, grad_acceleration, grad_jerk);

        double cost_space = 0.0;
        switch (spatial_mode)
        {
        case SPATIAL_CORRIDOR:
            cost_space += accumulateCorridorPenalty(seg_idx, position, grad_position);
            break;
        case SPATIAL_ESDF:
            if (grid_map != nullptr && grid_map->esdfEnabled())
            {
                if (!shouldMaskPerchingPlatformObstacle(t_global, position))
                {
                    Types::Vec3 sdf_grad = Types::Vec3::Zero();
                    const double sdf_value = grid_map->getDistWithGradTrilinear(position, sdf_grad);
                    cost_space += cost_functional::accumulateDistanceFieldObstaclePenalty(
                        position, sdf_value, sdf_grad, safe_margin, wei_dist, grad_position);
                }
            }
            break;
        case SPATIAL_PLAIN:
        default:
            cost_space += cost_functional::accumulateObstaclePenalty(
                cp_idx, cps, touch_goal, position,
                obs_clearance, obs_clearance_soft,
                wei_obs, wei_obs_soft, grad_position);
            break;
        }

        const double cost_swarm =
            cost_functional::accumulateSwarmPenalty(
                cp_idx, cps, touch_goal, swarm_traj, drone_id,
                t_now, t_global, swarm_clearance, wei_swarm,
                position, grad_position, grad_time, min_ellip_dist2_ptr);

        Types::Vec3 gp = Types::Vec3::Zero();
        Types::Vec3 ga = Types::Vec3::Zero();
        Types::Vec3 gj = Types::Vec3::Zero();
        double gt = 0.0;
        const double cost_perching =
            accumulatePerchingRunningCost(t_global,
                                          position,
                                          acceleration,
                                          jerk,
                                          gp,
                                          ga,
                                          gj,
                                          gt);

        grad_position += gp;
        grad_acceleration += ga;
        grad_jerk += gj;
        grad_time += gt;

        accumulated_costs(0) += cost_feasibility;
        accumulated_costs(1) += cost_space;
        accumulated_costs(2) += cost_swarm;
        accumulated_costs(3) += cost_perching;

        return cost_feasibility + cost_space + cost_swarm + cost_perching;
    }

    template <typename SamplesType>
    double evaluateSample(const SamplesType &samples,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_p,
                          Eigen::VectorXd &grad_t_global) const
    {
        grad_p.setZero(3, samples.size());
        grad_t_global.setZero(samples.size());

        if ((wei_sqrvar <= 0.0 && wei_corridor_ref <= 0.0) || samples.empty())
        {
            return 0.0;
        }

        const int point_num = samples.back().logical_idx + 1;
        if (point_num < 1)
        {
            return 0.0;
        }

        Eigen::MatrixXd points(point_num, 3);
        points.setZero();
        std::vector<int> canonical_sample(point_num, -1);
        for (int i = 0; i < static_cast<int>(samples.size()); ++i)
        {
            const int logical_idx = samples[i].logical_idx;
            points.row(logical_idx) = samples[i].p.transpose();
            if (canonical_sample[logical_idx] < 0)
            {
                canonical_sample[logical_idx] = i;
            }
        }

        double cost_total = 0.0;
        if (wei_sqrvar > 0.0 && point_num >= 2)
        {
            Eigen::MatrixXd grad_points = Eigen::MatrixXd::Zero(point_num, 3);
            const double cost_smooth =
                cost_functional::accumulateVarianceSampleCost(points, wei_sqrvar, grad_points);
            cost_total += cost_smooth;
            accumulated_costs(4) += cost_smooth;

            for (int logical_idx = 0; logical_idx < point_num; ++logical_idx)
            {
                const int sample_idx = canonical_sample[logical_idx];
                if (sample_idx >= 0)
                {
                    grad_p.col(sample_idx) += grad_points.row(logical_idx).transpose();
                }
            }
        }

        const int ref_cols = (reference_points != nullptr) ? static_cast<int>(reference_points->cols()) : 0;
        double cost_ref = 0.0;
        if (wei_corridor_ref > 0.0 && ref_cols > 0)
        {
            for (int logical_idx = 0; logical_idx < point_num; ++logical_idx)
            {
                const int sample_idx = canonical_sample[logical_idx];
                if (sample_idx < 0 || logical_idx >= ref_cols)
                {
                    continue;
                }
                const Types::Vec3 ref_pt = reference_points->col(logical_idx);
                const Types::Vec3 err = samples[sample_idx].p - ref_pt;
                grad_p.col(sample_idx) += wei_corridor_ref * err;
                cost_ref += 0.5 * wei_corridor_ref * err.squaredNorm();
            }
        }

        accumulated_costs(5) += cost_ref;
        return cost_total + cost_ref;
    }

private:
    static Eigen::Vector3d gravity()
    {
        return Eigen::Vector3d(0.0, 0.0, -9.81);
    }

    static Eigen::Vector3d normalizedOrFallback(const Eigen::Vector3d &vec,
                                                const Eigen::Vector3d &fallback)
    {
        if (!vec.allFinite() || vec.norm() < 1.0e-6)
        {
            return fallback;
        }
        return vec.normalized();
    }

    static double smoothed01(const double x,
                             double &grad)
    {
        static constexpr double mu = 0.01;
        static constexpr double mu4 = mu * mu * mu * mu;
        static constexpr double mu4_inv = 1.0 / mu4;
        if (x < -mu)
        {
            grad = 0.0;
            return 0.0;
        }
        if (x < 0.0)
        {
            const double y = x + mu;
            const double y2 = y * y;
            grad = y2 * (mu - 2.0 * x) * mu4_inv;
            return 0.5 * y2 * y * (mu - x) * mu4_inv;
        }
        if (x < mu)
        {
            const double y = x - mu;
            const double y2 = y * y;
            grad = y2 * (mu + 2.0 * x) * mu4_inv;
            return 0.5 * y2 * y * (mu + x) * mu4_inv + 1.0;
        }
        grad = 0.0;
        return 1.0;
    }

    static Eigen::Matrix3d f_DN(const Eigen::Vector3d &x)
    {
        const double x_norm_2 = std::max(1.0e-9, x.squaredNorm());
        return (Eigen::Matrix3d::Identity() - x * x.transpose() / x_norm_2) / std::sqrt(x_norm_2);
    }

    static Eigen::Matrix3d f_D2N_times_vec(const Eigen::Vector3d &x,
                                           const Eigen::Vector3d &y)
    {
        const double x_norm_2 = std::max(1.0e-9, x.squaredNorm());
        const double x_norm_3 = x_norm_2 * std::sqrt(x_norm_2);
        const Eigen::Matrix3d A =
            (3.0 * x * x.transpose() / x_norm_2 - Eigen::Matrix3d::Identity());
        return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::Matrix3d::Identity()) / x_norm_3;
    }

    double accumulatePolyPenalty(const spatial_map::PolyhedronH &poly,
                                 const Types::Vec3 &position,
                                 Types::Vec3 &grad_position) const
    {
        const double smooth_eps = std::max(corridor_smoothing, 1.0e-6);

        double cost = 0.0;
        for (int i = 0; i < poly.rows(); ++i)
        {
            const Types::Vec3 outer_normal = poly.row(i).head<3>().transpose();
            if (outer_normal.squaredNorm() <= 1.0e-12)
            {
                continue;
            }

            const double violation =
                outer_normal.dot(position) + poly(i, 3) + corridor_clearance;

            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (!cost_functional::smoothedL1(violation, smooth_eps, penalty, penalty_grad))
            {
                continue;
            }

            grad_position += wei_corridor * penalty_grad * outer_normal;
            cost += wei_corridor * penalty;
        }
        return cost;
    }

    int resolvePolyId(const int seg_idx) const
    {
        if (!h_polys || h_polys->empty())
        {
            return -1;
        }

        if (!segment_poly_idx || segment_poly_idx->size() <= 0)
        {
            return std::min(std::max(seg_idx, 0), static_cast<int>(h_polys->size()) - 1);
        }

        const int idx_count = static_cast<int>(segment_poly_idx->size());
        const int idx_pos = std::min(std::max(seg_idx, 0), idx_count - 1);
        const int poly_id = (*segment_poly_idx)(idx_pos);
        return std::min(std::max(poly_id, 0), static_cast<int>(h_polys->size()) - 1);
    }

    double accumulateCorridorPenalty(const int seg_idx,
                                     const Types::Vec3 &position,
                                     Types::Vec3 &grad_position) const
    {
        if (wei_corridor <= 0.0)
        {
            return 0.0;
        }

        const int poly_id = resolvePolyId(seg_idx);
        if (poly_id < 0)
        {
            return 0.0;
        }

        return accumulatePolyPenalty((*h_polys)[poly_id], position, grad_position);
    }

    bool perchingEnabled() const
    {
        return semantic_config != nullptr;
    }

    Eigen::Vector3d platePositionAtTime(const double t_global) const
    {
        if (!perchingEnabled())
        {
            return Eigen::Vector3d::Zero();
        }
        const double dt = t_global - semantic_config->reference_time;
        return semantic_config->plate_position + semantic_config->plate_velocity * dt;
    }

    bool shouldMaskPerchingPlatformObstacle(const double t_global,
                                            const Eigen::Vector3d &position) const
    {
        if (!perchingEnabled() || !mask_platform_from_esdf)
        {
            return false;
        }

        const Eigen::Vector3d plate_pos = platePositionAtTime(t_global);
        const Eigen::Vector3d surface_z =
            normalizedOrFallback(semantic_config->surface_z, Eigen::Vector3d::UnitZ());
        const Eigen::Vector3d rel = position - plate_pos;
        const double normal_offset = rel.dot(surface_z);
        const Eigen::Vector3d tangential = rel - normal_offset * surface_z;
        const double mask_radius =
            std::max(0.05, platform_radius + robot_radius + 0.20);
        const double mask_height =
            std::max(0.05, semantic_config->robot_l + robot_radius + 0.20);

        return tangential.norm() <= mask_radius &&
               std::abs(normal_offset) <= mask_height;
    }

    double accumulateFloorCost(const Eigen::Vector3d &position,
                               Eigen::Vector3d &grad_position) const
    {
        if (wei_perch_floor <= 0.0)
        {
            return 0.0;
        }
        const double pen = floor_height - position.z();
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!cost_functional::smoothedL1(pen, 0.01, penalty, penalty_grad))
        {
            return 0.0;
        }
        grad_position.z() -= wei_perch_floor * penalty_grad;
        return wei_perch_floor * penalty;
    }

    double accumulateRelativeHeightCost(const double t_global,
                                        const Eigen::Vector3d &position,
                                        Eigen::Vector3d &grad_position,
                                        double &grad_time) const
    {
        if (!perchingEnabled() || wei_perch_height <= 0.0)
        {
            return 0.0;
        }

        const Eigen::Vector3d plate_pos = platePositionAtTime(t_global);
        const Eigen::Vector3d world_z = Eigen::Vector3d::UnitZ();
        const double delta_z = world_z.dot(position - plate_pos);

        double cost = 0.0;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (cost_functional::smoothedL1(relative_height_min - delta_z, 0.01, penalty, penalty_grad))
        {
            cost += wei_perch_height * penalty;
            grad_position -= wei_perch_height * penalty_grad * world_z;
            grad_time += wei_perch_height * penalty_grad * world_z.dot(semantic_config->plate_velocity);
        }

        penalty = 0.0;
        penalty_grad = 0.0;
        if (cost_functional::smoothedL1(delta_z - relative_height_max, 0.01, penalty, penalty_grad))
        {
            cost += wei_perch_height * penalty;
            grad_position += wei_perch_height * penalty_grad * world_z;
            grad_time -= wei_perch_height * penalty_grad * world_z.dot(semantic_config->plate_velocity);
        }
        return cost;
    }

    double accumulateThrustCost(const Eigen::Vector3d &acceleration,
                                Eigen::Vector3d &grad_acceleration) const
    {
        if (wei_perch_thrust <= 0.0 || thrust_max <= 0.0)
        {
            return 0.0;
        }

        double cost = 0.0;
        const Eigen::Vector3d thrust_f = acceleration - gravity();
        const double thrust_sq = thrust_f.squaredNorm();

        const double max_pen = thrust_sq - thrust_max * thrust_max;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (cost_functional::smoothedL1(max_pen, 0.01, penalty, penalty_grad))
        {
            cost += wei_perch_thrust * penalty;
            grad_acceleration += wei_perch_thrust * 2.0 * penalty_grad * thrust_f;
        }

        if (thrust_min > 0.0)
        {
            const double min_pen = thrust_min * thrust_min - thrust_sq;
            penalty = 0.0;
            penalty_grad = 0.0;
            if (cost_functional::smoothedL1(min_pen, 0.01, penalty, penalty_grad))
            {
                cost += wei_perch_thrust * penalty;
                grad_acceleration -= wei_perch_thrust * 2.0 * penalty_grad * thrust_f;
            }
        }
        return cost;
    }

    double accumulateOmegaCost(const Eigen::Vector3d &acceleration,
                               const Eigen::Vector3d &jerk,
                               Eigen::Vector3d &grad_acceleration,
                               Eigen::Vector3d &grad_jerk) const
    {
        if (wei_perch_omega <= 0.0 || omega_max <= 0.0)
        {
            return 0.0;
        }

        const Eigen::Vector3d thrust_f = acceleration - gravity();
        const Eigen::Vector3d zb_dot = f_DN(thrust_f) * jerk;
        const double omega12_sq = zb_dot.squaredNorm();
        const double pen = omega12_sq - omega_max * omega_max;
        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!cost_functional::smoothedL1(pen, 0.01, penalty, penalty_grad))
        {
            return 0.0;
        }

        const Eigen::Vector3d grad_zb_dot = wei_perch_omega * 2.0 * penalty_grad * zb_dot;
        grad_jerk += f_DN(thrust_f).transpose() * grad_zb_dot;
        grad_acceleration += f_D2N_times_vec(thrust_f, jerk).transpose() * grad_zb_dot;
        return wei_perch_omega * penalty;
    }

    double accumulatePerchingCollisionCost(const double t_global,
                                           const Eigen::Vector3d &position,
                                           const Eigen::Vector3d &acceleration,
                                           Eigen::Vector3d &grad_position,
                                           Eigen::Vector3d &grad_acceleration,
                                           double &grad_time) const
    {
        if (!perchingEnabled() || wei_perch_collision <= 0.0)
        {
            return 0.0;
        }

        const Eigen::Vector3d plate_pos = platePositionAtTime(t_global);
        const Eigen::Vector3d surface_z =
            normalizedOrFallback(semantic_config->surface_z, Eigen::Vector3d::UnitZ());

        const double safe_r = std::max(1.0e-3, platform_radius + robot_radius);
        const double safe_r_sqr = safe_r * safe_r;
        const double dist_sqr = (position - plate_pos).squaredNorm();
        const double pen_dist = (safe_r_sqr - dist_sqr) / safe_r_sqr;
        double grad_dist = 0.0;
        const double var01 = smoothed01(pen_dist, grad_dist);
        if (var01 <= 0.0)
        {
            return 0.0;
        }

        const Eigen::Vector3d grad_pos_dist = grad_dist * 2.0 * (plate_pos - position);
        const Eigen::Vector3d grad_plate_dist = -grad_pos_dist;

        const Eigen::Vector3d a_i = -surface_z;
        const double b_i = a_i.dot(plate_pos);
        const Eigen::Vector3d thrust_f = acceleration - gravity();
        const Eigen::Vector3d zb = normalizedOrFallback(thrust_f, surface_z);

        const double a = zb.x();
        const double b = zb.y();
        const double c = zb.z();
        const double c_1 = 1.0 / std::max(1.0e-6, 1.0 + c);

        Eigen::Matrix<double, 2, 3> BTRT;
        BTRT(0, 0) = 1.0 - a * a * c_1;
        BTRT(0, 1) = -a * b * c_1;
        BTRT(0, 2) = -a;
        BTRT(1, 0) = -a * b * c_1;
        BTRT(1, 1) = 1.0 - b * b * c_1;
        BTRT(1, 2) = -b;

        const Eigen::Vector2d v2 = BTRT * a_i;
        const double v2_norm = std::sqrt(v2.squaredNorm() + 1.0e-6);
        const double pen =
            a_i.dot(position) -
            std::max(0.0, semantic_config->robot_l - 0.005) * a_i.dot(zb) -
            b_i +
            robot_radius * v2_norm;

        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!cost_functional::smoothedL1(pen, 0.01, penalty, penalty_grad))
        {
            return 0.0;
        }

        Eigen::Vector3d grad_pos = a_i;
        Eigen::Vector3d grad_plate = -a_i;
        const Eigen::Vector2d grad_v2 = robot_radius * v2 / v2_norm;

        const double c2_1 = c_1 * c_1;
        Eigen::Matrix<double, 2, 3> pM_pa, pM_pb, pM_pc;
        pM_pa << -2.0 * a * c_1, -b * c_1, -1.0,
                 -b * c_1, 0.0, 0.0;
        pM_pb << 0.0, -a * c_1, 0.0,
                 -a * c_1, -2.0 * b * c_1, -1.0;
        pM_pc << a * a * c2_1, a * b * c2_1, 0.0,
                 a * b * c2_1, b * b * c2_1, 0.0;

        Eigen::Matrix<double, 2, 3> pv2_pzb;
        pv2_pzb.col(0) = pM_pa * a_i;
        pv2_pzb.col(1) = pM_pb * a_i;
        pv2_pzb.col(2) = pM_pc * a_i;

        const Eigen::Vector3d grad_zb =
            pv2_pzb.transpose() * grad_v2 - std::max(0.0, semantic_config->robot_l) * a_i;
        Eigen::Vector3d grad_acc = f_DN(thrust_f).transpose() * grad_zb;

        const double penalty_raw = penalty;
        grad_pos = penalty_grad * var01 * grad_pos + grad_pos_dist * penalty_raw;
        grad_plate = penalty_grad * var01 * grad_plate + grad_plate_dist * penalty_raw;
        grad_acc *= penalty_grad * var01;
        penalty = penalty_raw * var01;

        grad_position += wei_perch_collision * grad_pos;
        grad_acceleration += wei_perch_collision * grad_acc;
        grad_time += wei_perch_collision * grad_plate.dot(semantic_config->plate_velocity);
        return wei_perch_collision * penalty;
    }

    double accumulatePerchingRunningCost(const double t_global,
                                         const Eigen::Vector3d &position,
                                         const Eigen::Vector3d &acceleration,
                                         const Eigen::Vector3d &jerk,
                                         Eigen::Vector3d &grad_position,
                                         Eigen::Vector3d &grad_acceleration,
                                         Eigen::Vector3d &grad_jerk,
                                         double &grad_time) const
    {
        if (!perchingEnabled())
        {
            return 0.0;
        }

        double cost = 0.0;
        cost += accumulateFloorCost(position, grad_position);
        cost += accumulateRelativeHeightCost(t_global, position, grad_position, grad_time);
        cost += accumulateThrustCost(acceleration, grad_acceleration);
        cost += accumulateOmegaCost(acceleration, jerk, grad_acceleration, grad_jerk);
        cost += accumulatePerchingCollisionCost(t_global,
                                                position,
                                                acceleration,
                                                grad_position,
                                                grad_acceleration,
                                                grad_time);
        return cost;
    }
};

} // namespace cost_functional

#endif
