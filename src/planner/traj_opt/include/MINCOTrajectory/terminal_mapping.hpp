#ifndef MINCO_TERMINAL_MAPPING_HPP
#define MINCO_TERMINAL_MAPPING_HPP

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace minco
{

/**
 * @brief Task-layer terminal-state mapping hook.
 *
 * Fixed-boundary MINCO eliminates polynomial coefficients with
 * M(T)c = b(head, inner, tail). State-to-state and current tracking tasks keep
 * head/tail fixed, so no chain rule beyond MINCO's own variables is required.
 *
 * Perching-style tasks need a higher-level terminal model such as
 * tail_state = F(T, Xi(T), nu, tau_f, ...). In that case the optimizer first
 * exposes dJ/d(head_state) and dJ/d(tail_state), then the task mapping adds
 * chain-rule terms to outer decision variables and to physical segment-time
 * gradients.
 */
template <int DIM, int S>
class TerminalMappingBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BoundaryState = Eigen::Matrix<double, DIM, S>;

  virtual ~TerminalMappingBase() = default;

  virtual bool enabled() const = 0;

  /**
   * @brief Number of extra unconstrained variables appended after [tau, xi].
   *
   * Fixed-terminal tasks keep this zero. Dynamic terminal tasks such as
   * perching can expose variables like tangential closing velocity or thrust
   * phase here.
   */
  virtual int extraVariableDim() const
  {
    return 0;
  }

  /**
   * @brief Fill the initial guess of extra terminal variables.
   */
  virtual void setInitialExtraVariables(Eigen::Ref<Eigen::VectorXd> extra_vars) const
  {
    extra_vars.setZero();
  }

  /**
   * @brief Forward terminal mapping hook.
   *
   * MINCOOptimizer first decodes [tau, xi] into physical segment times and
   * inner points. Dynamic terminal tasks then map
   *   head/tail = F(cache_T, extra_vars, nominal_head, nominal_tail)
   * before generating the trajectory coefficients.
   */
  virtual void mapBoundaryStates(const BoundaryState &nominal_head_state,
                                 const BoundaryState &nominal_tail_state,
                                 const Eigen::VectorXd &cache_T,
                                 const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                 BoundaryState &mapped_head_state,
                                 BoundaryState &mapped_tail_state) const
  {
    mapped_head_state = nominal_head_state;
    mapped_tail_state = nominal_tail_state;
    (void)cache_T;
    (void)extra_vars;
  }

  /**
   * @brief Optional regularization on extra terminal variables.
   *
   * This cost is accumulated directly in the unconstrained decision space.
   */
  virtual double addExtraVariableCost(const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                      Eigen::Ref<Eigen::VectorXd> grad_extra) const
  {
    grad_extra.setZero();
    (void)extra_vars;
    return 0.0;
  }

  /**
   * @brief Backpropagate terminal gradient to extra outer variables.
   *
   * grad_out is the full unconstrained decision-gradient vector supplied by
   * the caller. MINCO writes its own [tau, xi] entries; task-specific terminal
   * variables, if present, should be written by the mapping implementation.
   */
  virtual void backwardTerminalGradient(const BoundaryState &grad_head_state,
                                        const BoundaryState &grad_tail_state,
                                        const Eigen::VectorXd &cache_T,
                                        const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                        Eigen::Ref<Eigen::VectorXd> grad_out) const = 0;

  /**
   * @brief Add physical-time chain-rule terms before TimeMap::backward.
   *
   * Fixed-boundary MINCO produces dJ/dT | fixed_tail. If a task has
   * tail_state = F(T, ...), it must add
   * (d tail_state / dT)^T * (dJ / d tail_state)
   * to grad_by_times here. MINCOOptimizer then maps that physical dJ/dT to
   * the unconstrained tau gradient through the active TimeMap.
   */
  virtual void backwardTerminalTimeGradient(const BoundaryState &grad_head_state,
                                            const BoundaryState &grad_tail_state,
                                            const Eigen::VectorXd &cache_T,
                                            const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                            Eigen::Ref<Eigen::VectorXd> grad_by_times) const = 0;
};

template <int DIM, int S>
class FixedTerminalMapping final : public TerminalMappingBase<DIM, S>
{
public:
  using BoundaryState = typename TerminalMappingBase<DIM, S>::BoundaryState;

  bool enabled() const override
  {
    return false;
  }

  void backwardTerminalGradient(const BoundaryState &,
                                const BoundaryState &,
                                const Eigen::VectorXd &,
                                const Eigen::Ref<const Eigen::VectorXd> &,
                                Eigen::Ref<Eigen::VectorXd>) const override
  {
  }

  void backwardTerminalTimeGradient(const BoundaryState &,
                                    const BoundaryState &,
                                    const Eigen::VectorXd &,
                                    const Eigen::Ref<const Eigen::VectorXd> &,
                                    Eigen::Ref<Eigen::VectorXd>) const override
  {
  }
};

/**
 * @brief Simplified Fast-Perching-style dynamic terminal mapping.
 *
 * This mapping follows the paper semantics in a lightweight way:
 *   tail_pos(T) = Xi(T) + l * z_s
 *   tail_vel(T) = Xi_dot + nu_x * x_s + nu_y * y_s - v_plus * z_s
 *   tail_acc(T) = (tau_m + tau_r * sin(tau_f)) * z_s + g
 *   tail_jerk(T) = 0                              (when S >= 4)
 *
 * Here Xi(T) is predicted with a constant-velocity landing-plate model and
 * the surface frame {x_s, y_s, z_s} is assumed constant over one planning
 * cycle. That keeps the current perching pipeline compatible with the generic
 * MINCO backend while exposing the correct chain rule:
 *
 *   dJ / d extra = (d tail_state / d extra)^T * dJ / d tail_state
 *   dJ / d T_i  += (d tail_state / d T)^T * dJ / d tail_state
 *
 * The same mapping works for S=3 (minimum jerk, P/V/A tail constraints) and
 * for S=4 (minimum snap, P/V/A/J tail constraints). When S=4, terminal jerk
 * is explicitly forced to zero as in the paper. This repository currently
 * executes the runtime trajectory through the S=3 path; the S=4 mapping/alias
 * is provided so the backend is perching-ready without introducing a fake
 * half-wired runtime path.
 */
template <int DIM, int S>
class PerchingTerminalMapping final : public TerminalMappingBase<DIM, S>
{
public:
  static_assert(DIM == 3, "PerchingTerminalMapping currently assumes 3D position trajectories.");
  static_assert(S >= 3, "PerchingTerminalMapping requires boundary derivatives up to acceleration.");

  using BoundaryState = typename TerminalMappingBase<DIM, S>::BoundaryState;

  enum ExtraIndex
  {
    IDX_NU_X = 0,
    IDX_NU_Y = 1,
    IDX_TAU_F = 2,
    EXTRA_DIM = 3
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PerchingTerminalMapping() = default;

  void configure(const Eigen::Vector3d &plate_position,
                 const Eigen::Vector3d &plate_velocity,
                 const Eigen::Vector3d &surface_x,
                 const Eigen::Vector3d &surface_y,
                 const Eigen::Vector3d &surface_z,
                 const double robot_l,
                 const double v_plus,
                 const double thrust_nominal,
                 const double thrust_range,
                 const bool use_dynamics_terminal_accel,
                 const Eigen::Vector2d &nu_seed = Eigen::Vector2d::Zero(),
                 const double tau_f_seed = 0.0,
                 const double weight_nu = 1.0e-2,
                 const double weight_tau_f = 1.0e-3)
  {
    plate_position_ = plate_position;
    plate_velocity_ = plate_velocity;
    surface_x_ = normalizedOr(surface_x, Eigen::Vector3d::UnitX());
    surface_y_ = normalizedOr(surface_y, Eigen::Vector3d::UnitY());
    surface_z_ = normalizedOr(surface_z, Eigen::Vector3d::UnitZ());

    // Keep the tangent frame right-handed and orthonormal enough for chain
    // rule projections.
    surface_y_ = normalizedOr(surface_z_.cross(surface_x_), Eigen::Vector3d::UnitY());
    surface_x_ = normalizedOr(surface_y_.cross(surface_z_), Eigen::Vector3d::UnitX());

    robot_l_ = std::max(0.0, robot_l);
    v_plus_ = std::max(0.0, v_plus);
    thrust_nominal_ = thrust_nominal;
    thrust_range_ = std::max(0.0, thrust_range);
    use_dynamics_terminal_accel_ = use_dynamics_terminal_accel;
    nu_seed_ = nu_seed;
    tau_f_seed_ = tau_f_seed;
    weight_nu_ = std::max(0.0, weight_nu);
    weight_tau_f_ = std::max(0.0, weight_tau_f);
    configured_ = true;
  }

  bool enabled() const override
  {
    return configured_;
  }

  int extraVariableDim() const override
  {
    return EXTRA_DIM;
  }

  void setInitialExtraVariables(Eigen::Ref<Eigen::VectorXd> extra_vars) const override
  {
    if (extra_vars.size() != EXTRA_DIM)
    {
      return;
    }
    extra_vars.setZero();
    extra_vars(IDX_NU_X) = nu_seed_.x();
    extra_vars(IDX_NU_Y) = nu_seed_.y();
    extra_vars(IDX_TAU_F) = tau_f_seed_;
  }

  void mapBoundaryStates(const BoundaryState &nominal_head_state,
                         const BoundaryState &nominal_tail_state,
                         const Eigen::VectorXd &cache_T,
                         const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                         BoundaryState &mapped_head_state,
                         BoundaryState &mapped_tail_state) const override
  {
    mapped_head_state = nominal_head_state;
    mapped_tail_state = nominal_tail_state;
    if (!configured_)
    {
      return;
    }

    const double total_T = cache_T.size() > 0 ? cache_T.sum() : 0.0;
    const double nu_x = extra_vars.size() > IDX_NU_X ? extra_vars(IDX_NU_X) : 0.0;
    const double nu_y = extra_vars.size() > IDX_NU_Y ? extra_vars(IDX_NU_Y) : 0.0;
    const double tau_f = extra_vars.size() > IDX_TAU_F ? extra_vars(IDX_TAU_F) : 0.0;

    mapped_tail_state.col(0) = plate_position_ + plate_velocity_ * total_T + robot_l_ * surface_z_;
    mapped_tail_state.col(1) = plate_velocity_ +
                               nu_x * surface_x_ +
                               nu_y * surface_y_ -
                               v_plus_ * surface_z_;
    if (use_dynamics_terminal_accel_)
    {
      mapped_tail_state.col(2) =
          nominal_tail_state.col(2) + thrust_range_ * std::sin(tau_f) * surface_z_;
    }

    for (int d = 3; d < S; ++d)
    {
      mapped_tail_state.col(d).setZero();
    }
  }

  double addExtraVariableCost(const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                              Eigen::Ref<Eigen::VectorXd> grad_extra) const override
  {
    grad_extra.setZero();
    if (!configured_ || extra_vars.size() != EXTRA_DIM)
    {
      return 0.0;
    }

    const double nu_x = extra_vars(IDX_NU_X);
    const double nu_y = extra_vars(IDX_NU_Y);
    const double tau_f = extra_vars(IDX_TAU_F);

    double cost = 0.0;
    cost += weight_nu_ * (nu_x * nu_x + nu_y * nu_y);
    if (use_dynamics_terminal_accel_)
    {
      cost += weight_tau_f_ * tau_f * tau_f;
    }

    grad_extra(IDX_NU_X) = 2.0 * weight_nu_ * nu_x;
    grad_extra(IDX_NU_Y) = 2.0 * weight_nu_ * nu_y;
    grad_extra(IDX_TAU_F) = use_dynamics_terminal_accel_ ? 2.0 * weight_tau_f_ * tau_f : 0.0;
    return cost;
  }

  void backwardTerminalGradient(const BoundaryState &,
                                const BoundaryState &grad_tail_state,
                                const Eigen::VectorXd &,
                                const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
                                Eigen::Ref<Eigen::VectorXd> grad_out) const override
  {
    if (!configured_ || grad_out.size() < EXTRA_DIM || extra_vars.size() != EXTRA_DIM)
    {
      return;
    }

    const Eigen::Index offset = grad_out.size() - EXTRA_DIM;
    const double tau_f = extra_vars(IDX_TAU_F);

    grad_out(offset + IDX_NU_X) += surface_x_.dot(grad_tail_state.col(1));
    grad_out(offset + IDX_NU_Y) += surface_y_.dot(grad_tail_state.col(1));
    if (use_dynamics_terminal_accel_)
    {
      grad_out(offset + IDX_TAU_F) +=
          thrust_range_ * std::cos(tau_f) * surface_z_.dot(grad_tail_state.col(2));
    }
  }

  void backwardTerminalTimeGradient(const BoundaryState &,
                                    const BoundaryState &grad_tail_state,
                                    const Eigen::VectorXd &cache_T,
                                    const Eigen::Ref<const Eigen::VectorXd> &,
                                    Eigen::Ref<Eigen::VectorXd> grad_by_times) const override
  {
    if (!configured_ || cache_T.size() == 0 || grad_by_times.size() != cache_T.size())
    {
      return;
    }

    const double time_chain = plate_velocity_.dot(grad_tail_state.col(0));
    grad_by_times.array() += time_chain;
  }

private:
  static Eigen::Vector3d normalizedOr(const Eigen::Vector3d &v,
                                      const Eigen::Vector3d &fallback)
  {
    if (!v.allFinite() || v.norm() < 1.0e-6)
    {
      return fallback;
    }
    return v.normalized();
  }

private:
  bool configured_{false};
  Eigen::Vector3d plate_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d plate_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d surface_x_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d surface_y_{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d surface_z_{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.81)};
  double robot_l_{0.0};
  double v_plus_{0.0};
  double thrust_nominal_{9.81};
  double thrust_range_{0.0};
  bool use_dynamics_terminal_accel_{false};
  Eigen::Vector2d nu_seed_{Eigen::Vector2d::Zero()};
  double tau_f_seed_{0.0};
  double weight_nu_{1.0e-2};
  double weight_tau_f_{1.0e-3};
};

} // namespace minco

#endif // MINCO_TERMINAL_MAPPING_HPP
