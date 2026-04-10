#ifndef MINCO_TERMINAL_MAPPING_HPP
#define MINCO_TERMINAL_MAPPING_HPP

#include <Eigen/Core>

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
   * @brief Backpropagate terminal gradient to extra outer variables.
   *
   * grad_out is the full unconstrained decision-gradient vector supplied by
   * the caller. MINCO writes its own [tau, xi] entries; task-specific terminal
   * variables, if present, should be written by the mapping implementation.
   */
  virtual void backwardTerminalGradient(const BoundaryState &grad_head_state,
                                        const BoundaryState &grad_tail_state,
                                        const Eigen::VectorXd &cache_T,
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
                                Eigen::Ref<Eigen::VectorXd>) const override
  {
  }

  void backwardTerminalTimeGradient(const BoundaryState &,
                                    const BoundaryState &,
                                    const Eigen::VectorXd &,
                                    Eigen::Ref<Eigen::VectorXd>) const override
  {
  }
};

/**
 * @brief Skeleton for future Fast-Perching-style terminal optimization.
 *
 * This intentionally does not implement a fake perching optimizer. A real
 * implementation must be provided by the task layer with all terminal-model
 * Jacobians:
 *   tail_state = F(T, Xi(T), surface_frame(T), nu, tau_f, ...)
 * and then fill:
 *   d tail_state / d nu
 *   d tail_state / d tau_f
 *   d tail_state / d T
 * plus any target-prediction/surface-frame derivatives.
 */
template <int DIM, int S>
class PerchingTerminalMappingSkeleton : public TerminalMappingBase<DIM, S>
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
                                Eigen::Ref<Eigen::VectorXd>) const override
  {
    // TODO(perching): inject terminal gradients into nu/tau_f/surface params.
  }

  void backwardTerminalTimeGradient(const BoundaryState &,
                                    const BoundaryState &,
                                    const Eigen::VectorXd &,
                                    Eigen::Ref<Eigen::VectorXd>) const override
  {
    // TODO(perching): add (d tail_state / dT)^T * dJ/dtail_state.
  }
};

} // namespace minco

#endif // MINCO_TERMINAL_MAPPING_HPP
