#ifndef MINCO_OPTIMIZER_HPP
#define MINCO_OPTIMIZER_HPP

#include "MINCOTrajectory/MINCOTrajectory.hpp"
#include "MINCOTrajectory/terminal_mapping.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace minco
{

namespace optimizer_traits
{
  template <typename...>
  using void_t = void;

  template <typename T, typename = void>
  struct HasTimeMapInterface : std::false_type
  {
  };

  template <typename T>
  struct HasTimeMapInterface<T, void_t<
                                   decltype(static_cast<double>(std::declval<T>().toTime(std::declval<double>()))),
                                   decltype(static_cast<double>(std::declval<T>().toTau(std::declval<double>()))),
                                   decltype(static_cast<double>(std::declval<T>().backward(std::declval<double>(), std::declval<double>(), std::declval<double>())))>> : std::true_type
  {
  };

  template <typename T, int DIM, typename = void>
  struct HasSpatialMapInterface : std::false_type
  {
  };

  template <typename T, int DIM>
  struct HasSpatialMapInterface<T, DIM, void_t<
                                           decltype(static_cast<int>(std::declval<T>().getUnconstrainedDim(std::declval<int>()))),
                                           decltype(std::declval<T>().toPhysical(std::declval<Eigen::VectorXd>(), std::declval<int>())),
                                           decltype(std::declval<T>().toUnconstrained(std::declval<Eigen::Matrix<double, DIM, 1>>(), std::declval<int>())),
                                           decltype(std::declval<T>().backwardGrad(std::declval<Eigen::VectorXd>(),
                                                                                   std::declval<Eigen::Matrix<double, DIM, 1>>(),
                                                                                   std::declval<int>())),
                                           decltype(std::declval<T>().addNormPenalty(std::declval<Eigen::VectorXd>(), std::declval<double &>(), std::declval<Eigen::VectorXd &>()))>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasTimeCostInterface : std::false_type
  {
  };

  template <typename T>
  struct HasTimeCostInterface<T, void_t<
                                     decltype(static_cast<double>(std::declval<T>()(
                                         std::declval<const std::vector<double> &>(),
                                         std::declval<Eigen::VectorXd &>())))>> : std::true_type
  {
  };
} // namespace optimizer_traits

template <int DIM, int S, typename TimeMap, typename SpatialMap>
class MINCOOptimizer
{
  static_assert(optimizer_traits::HasTimeMapInterface<TimeMap>::value,
                "TimeMap does not satisfy the required interface.");
  static_assert(optimizer_traits::HasSpatialMapInterface<SpatialMap, DIM>::value,
                "SpatialMap does not satisfy the required interface.");

public:
  using TrajType = MINCOTrajectory<DIM, S>;
  using VectorType = Eigen::Matrix<double, DIM, 1>;
  using BoundaryState = typename TrajType::BoundaryState;
  using WaypointsType = Eigen::Matrix<double, Eigen::Dynamic, DIM>;
  using InnerPointsMat = Eigen::Matrix<double, DIM, Eigen::Dynamic>;
  using CoeffMat = Eigen::Matrix<double, Eigen::Dynamic, DIM>;

  struct Sample
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int seg_idx{0};
    int step_in_seg{0};
    int logical_idx{0};
    double alpha{0.0};
    double t_local{0.0};
    double t_global{0.0};
    double trap_weight{0.0};
    double dt{0.0};
    typename TrajType::BasisRow b_p;
    VectorType p{VectorType::Zero()};
    VectorType v{VectorType::Zero()};
  };

  using SampleBuffer = std::vector<Sample, Eigen::aligned_allocator<Sample>>;

  struct Workspace
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd cache_T;
    InnerPointsMat cache_P_inner;
    BoundaryState head_state;
    BoundaryState tail_state;

    CoeffMat gdC_energy;
    Eigen::VectorXd gdT_energy;

    CoeffMat gdC_integral;
    Eigen::VectorXd gdT_integral;

    CoeffMat gdC_sample;
    Eigen::VectorXd gdT_sample;

    Eigen::VectorXd gdT_time;

    InnerPointsMat grad_by_points;
    Eigen::VectorXd grad_by_times;
    BoundaryState grad_by_head_state;
    BoundaryState grad_by_tail_state;

    SampleBuffer samples;
    Eigen::Matrix<double, DIM, Eigen::Dynamic> sample_grad_p;
    Eigen::VectorXd sample_grad_t_global;

    void resize(int piece_num)
    {
      cache_T.resize(piece_num);
      cache_P_inner.resize(DIM, std::max(0, piece_num - 1));
      gdC_energy.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_energy.resize(piece_num);
      gdC_integral.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_integral.resize(piece_num);
      gdC_sample.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_sample.resize(piece_num);
      gdT_time.resize(piece_num);
      grad_by_points.resize(DIM, std::max(0, piece_num - 1));
      grad_by_times.resize(piece_num);
      grad_by_head_state.setZero();
      grad_by_tail_state.setZero();
    }
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MINCOOptimizer()
      : active_time_map_(&default_time_map_),
        active_spatial_map_(&default_spatial_map_)
  {
    workspace_ = std::make_unique<Workspace>();
  }

  void setTimeMap(const TimeMap *time_map)
  {
    active_time_map_ = time_map != nullptr ? time_map : &default_time_map_;
  }

  void setSpatialMap(const SpatialMap *spatial_map)
  {
    active_spatial_map_ = spatial_map != nullptr ? spatial_map : &default_spatial_map_;
  }

  void setEnergyWeight(double rho_energy)
  {
    rho_energy_ = rho_energy;
  }

  void setSamplesPerPiece(int samples_per_piece)
  {
    samples_per_piece_ = std::max(1, samples_per_piece);
  }

  bool setInitState(const std::vector<double> &time_segments,
                    const WaypointsType &waypoints,
                    const BoundaryState &boundary_head,
                    const BoundaryState &boundary_tail)
  {
    piece_num_ = static_cast<int>(time_segments.size());
    ref_times_ = time_segments;
    ref_waypoints_ = waypoints;

    workspace_->resize(piece_num_);
    nominal_head_state_ = boundary_head;
    nominal_tail_state_ = boundary_tail;
    workspace_->head_state = nominal_head_state_;
    workspace_->tail_state = nominal_tail_state_;
    return piece_num_ > 0;
  }

  void setWarmStartGuess(const Eigen::Ref<const Eigen::VectorXd> &x)
  {
    warm_start_guess_ = x;
    has_warm_start_guess_ = (x.size() > 0);
  }

  void clearWarmStartGuess()
  {
    warm_start_guess_.resize(0);
    has_warm_start_guess_ = false;
  }

  bool hasWarmStartGuess() const
  {
    return has_warm_start_guess_;
  }

  const Eigen::VectorXd &warmStartGuess() const
  {
    return warm_start_guess_;
  }

  Eigen::VectorXd encodeDecisionVector(
      const std::vector<double> &physical_times,
      const WaypointsType &physical_waypoints,
      const TerminalMappingBase<DIM, S> *terminal_mapping = nullptr,
      const Eigen::VectorXd *extra_vars = nullptr) const
  {
    if (static_cast<int>(physical_times.size()) != piece_num_ ||
        physical_waypoints.rows() != piece_num_ + 1 ||
        physical_waypoints.cols() != DIM)
    {
      return Eigen::VectorXd{};
    }

    const int dim_T = piece_num_;
    const int extra_dim =
        (terminal_mapping != nullptr && terminal_mapping->enabled())
            ? terminal_mapping->extraVariableDim()
            : 0;
    const int total_dim = getCoreDecisionDim() + extra_dim;

    Eigen::VectorXd x(total_dim);
    for (int i = 0; i < piece_num_; ++i)
    {
      if (!std::isfinite(physical_times[static_cast<std::size_t>(i)]) ||
          physical_times[static_cast<std::size_t>(i)] <= 0.0)
      {
        return Eigen::VectorXd{};
      }
      x(i) = active_time_map_->toTau(physical_times[static_cast<std::size_t>(i)]);
    }

    int offset = dim_T;
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = active_spatial_map_->getUnconstrainedDim(i);
      const auto waypoint = physical_waypoints.row(i).transpose();
      if (!waypoint.allFinite())
      {
        return Eigen::VectorXd{};
      }
      x.segment(offset, dof) =
          active_spatial_map_->toUnconstrained(waypoint, i);
      offset += dof;
    }

    if (extra_dim > 0)
    {
      Eigen::Ref<Eigen::VectorXd> extra_segment =
          x.segment(offset, extra_dim);
      if (extra_vars != nullptr &&
          extra_vars->size() == extra_dim &&
          extra_vars->allFinite())
      {
        extra_segment = *extra_vars;
      }
      else if (terminal_mapping != nullptr)
      {
        terminal_mapping->setInitialExtraVariables(extra_segment);
      }
      else
      {
        extra_segment.setZero();
      }
    }

    return x;
  }

  Eigen::VectorXd generateInitialGuess() const
  {
    return generateInitialGuess(nullptr);
  }

  Eigen::VectorXd generateInitialGuess(const TerminalMappingBase<DIM, S> *terminal_mapping) const
  {
    const int total_dim =
        getCoreDecisionDim() +
        ((terminal_mapping != nullptr && terminal_mapping->enabled())
             ? terminal_mapping->extraVariableDim()
             : 0);

    if (has_warm_start_guess_ &&
        warm_start_guess_.size() == total_dim &&
        warm_start_guess_.allFinite())
    {
      return warm_start_guess_;
    }

    return encodeDecisionVector(ref_times_, ref_waypoints_, terminal_mapping, nullptr);
  }

  template <typename TimeCostFunc, typename CostManager>
  double evaluate(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Eigen::Ref<Eigen::VectorXd> grad_out,
                  TimeCostFunc &&time_cost_func,
                  CostManager &&cost_manager)
  {
    FixedTerminalMapping<DIM, S> fixed_terminal_mapping;
    return evaluateWithTerminalMapping(x,
                                       grad_out,
                                       std::forward<TimeCostFunc>(time_cost_func),
                                       std::forward<CostManager>(cost_manager),
                                       &fixed_terminal_mapping);
  }

  template <typename TimeCostFunc, typename CostManager>
  double evaluateWithTerminalMapping(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      Eigen::Ref<Eigen::VectorXd> grad_out,
      TimeCostFunc &&time_cost_func,
      CostManager &&cost_manager,
      const TerminalMappingBase<DIM, S> *terminal_mapping)
  {
    static_assert(optimizer_traits::HasTimeCostInterface<typename std::decay<TimeCostFunc>::type>::value,
                  "TimeCostFunc does not satisfy the required interface.");

    grad_out.setZero();
    double total_cost = 0.0;
    const int core_dim = getCoreDecisionDim();
    const int extra_dim =
        (terminal_mapping != nullptr && terminal_mapping->enabled())
            ? terminal_mapping->extraVariableDim()
            : 0;

    if (x.size() != core_dim + extra_dim)
    {
      return std::numeric_limits<double>::infinity();
    }

    // Constraint elimination path:
    //   x = [tau, xi] -> (T, P_inner) through active time/spatial maps
    //   -> build MINCO trajectory in physical space
    //   -> accumulate costs/partials in physical variables
    //   -> backpropagate to the unconstrained decision vector x
    decodeDecisionVariables(x, grad_out, total_cost);
    workspace_->head_state = nominal_head_state_;
    workspace_->tail_state = nominal_tail_state_;
    if (terminal_mapping != nullptr && terminal_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      terminal_mapping->mapBoundaryStates(nominal_head_state_,
                                          nominal_tail_state_,
                                          workspace_->cache_T,
                                          extra_vars,
                                          workspace_->head_state,
                                          workspace_->tail_state);
      if (extra_dim > 0)
      {
        total_cost += terminal_mapping->addExtraVariableCost(
            extra_vars, grad_out.segment(core_dim, extra_dim));
      }
    }
    traj_.generate(workspace_->cache_P_inner,
                   workspace_->head_state,
                   workspace_->tail_state,
                   workspace_->cache_T);

    double energy_cost = 0.0;
    if (rho_energy_ > 0.0)
    {
      traj_.getEnergyPartialGradByCoeffs(energy_cost, workspace_->gdC_energy);
      traj_.getEnergyPartialGradByTimes(workspace_->gdT_energy);
      total_cost += rho_energy_ * energy_cost;
    }
    else
    {
      workspace_->gdC_energy.setZero();
      workspace_->gdT_energy.setZero();
    }

    std::vector<double> T_vec(workspace_->cache_T.data(), workspace_->cache_T.data() + workspace_->cache_T.size());
    workspace_->gdT_time.setZero();
    total_cost += time_cost_func(T_vec, workspace_->gdT_time);

    double integral_cost = 0.0;
    accumulateIntegralCost(cost_manager, integral_cost);
    total_cost += integral_cost;

    double sample_cost = 0.0;
    accumulateSampleCost(cost_manager, sample_cost);
    total_cost += sample_cost;

    const CoeffMat gdC_total =
        rho_energy_ * workspace_->gdC_energy + workspace_->gdC_integral + workspace_->gdC_sample;
    const Eigen::VectorXd gdT_direct_total =
        rho_energy_ * workspace_->gdT_energy + workspace_->gdT_time + workspace_->gdT_integral + workspace_->gdT_sample;

    traj_.propagateGradFull(gdC_total,
                            gdT_direct_total,
                            workspace_->grad_by_points,
                            workspace_->grad_by_times,
                            workspace_->grad_by_head_state,
                            workspace_->grad_by_tail_state);

    // Fixed-boundary MINCO already has dJ/dT | fixed_tail. For terminal
    // mappings where tail_state = F(T, ...), add
    // (d tail_state / dT)^T * (dJ / d tail_state) to the physical time
    // gradient before mapping dJ/dT back to tau.
    if (terminal_mapping != nullptr && terminal_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      terminal_mapping->backwardTerminalTimeGradient(workspace_->grad_by_head_state,
                                                     workspace_->grad_by_tail_state,
                                                     workspace_->cache_T,
                                                     extra_vars,
                                                     workspace_->grad_by_times);
    }

    writeDecisionGradient(x, grad_out);

    if (terminal_mapping != nullptr && terminal_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      terminal_mapping->backwardTerminalGradient(workspace_->grad_by_head_state,
                                                 workspace_->grad_by_tail_state,
                                                 workspace_->cache_T,
                                                 extra_vars,
                                                 grad_out);
    }
    return total_cost;
  }

  const TrajType &getTrajectory() const
  {
    return traj_;
  }

private:
  int getCoreDecisionDim() const
  {
    int dim_P = 0;
    for (int i = 1; i < piece_num_; ++i)
    {
      dim_P += active_spatial_map_->getUnconstrainedDim(i);
    }
    return piece_num_ + dim_P;
  }

  void decodeDecisionVariables(const Eigen::Ref<const Eigen::VectorXd> &x,
                               Eigen::Ref<Eigen::VectorXd> grad_out,
                               double &total_cost)
  {
    for (int i = 0; i < piece_num_; ++i)
    {
      workspace_->cache_T(i) = active_time_map_->toTime(x(i));
    }

    int offset = piece_num_;
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = active_spatial_map_->getUnconstrainedDim(i);
      const Eigen::VectorXd xi = x.segment(offset, dof);
      workspace_->cache_P_inner.col(i - 1) = active_spatial_map_->toPhysical(xi, i);

      Eigen::VectorXd grad_xi = Eigen::VectorXd::Zero(dof);
      active_spatial_map_->addNormPenalty(xi, total_cost, grad_xi);
      grad_out.segment(offset, dof) += grad_xi;
      offset += dof;
    }
  }

  template <typename CostManager>
  void accumulateIntegralCost(CostManager &&cost_manager, double &cost)
  {
    workspace_->gdC_integral.setZero();
    workspace_->gdT_integral.setZero();
    workspace_->samples.clear();
    workspace_->samples.reserve(piece_num_ * samples_per_piece_ + 1);

    const auto &coeffs = traj_.getCoefficients();
    Eigen::VectorXd global_time_grad = Eigen::VectorXd::Zero(piece_num_);

    double seg_start_time = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      const double T = workspace_->cache_T(i);
      const double inv_K = 1.0 / static_cast<double>(samples_per_piece_);
      const double dt = T * inv_K;
      const int base_row = i * TrajType::COEFF_NUM;
      const auto coeff_block = coeffs.template block<TrajType::COEFF_NUM, DIM>(base_row, 0);

      for (int k = 0; k <= samples_per_piece_; ++k)
      {
        const double alpha = static_cast<double>(k) * inv_K;
        const double t = alpha * T;
        const double trap_weight = (k == 0 || k == samples_per_piece_) ? 0.5 : 1.0;
        const double common_weight = trap_weight * dt;
        const double t_global = seg_start_time + t;
        const int logical_idx = i * samples_per_piece_ + k;

        typename TrajType::BasisRow b_p, b_v, b_a, b_j, b_s;
        TrajType::computeBasisFunctions(t, b_p, b_v, b_a, b_j, b_s);

        VectorType p = VectorType::Zero();
        VectorType v = VectorType::Zero();
        VectorType a = VectorType::Zero();
        VectorType j = VectorType::Zero();
        VectorType s = VectorType::Zero();
        p.transpose().noalias() = b_p * coeff_block;
        v.transpose().noalias() = b_v * coeff_block;
        a.transpose().noalias() = b_a * coeff_block;
        j.transpose().noalias() = b_j * coeff_block;
        s.transpose().noalias() = b_s * coeff_block;

        VectorType gp = VectorType::Zero();
        VectorType gv = VectorType::Zero();
        VectorType ga = VectorType::Zero();
        VectorType gj = VectorType::Zero();
        double gt = 0.0;

        const double c_val = cost_manager.evaluateIntegral(
            logical_idx, t, t_global, i, k, p, v, a, j, gp, gv, ga, gj, gt);

        cost += c_val * common_weight;

        workspace_->gdC_integral.template block<TrajType::COEFF_NUM, DIM>(base_row, 0).noalias() +=
            (b_p.transpose() * gp.transpose() +
             b_v.transpose() * gv.transpose() +
             b_a.transpose() * ga.transpose() +
             b_j.transpose() * gj.transpose()) *
            common_weight;

        workspace_->gdT_integral(i) += c_val * trap_weight * inv_K;
        workspace_->gdT_integral(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j) + gj.dot(s)) * alpha * common_weight;
        workspace_->gdT_integral(i) += gt * alpha * common_weight;
        global_time_grad(i) += gt * common_weight;

        if (k > 0 || i == 0)
        {
          Sample sample;
          sample.seg_idx = i;
          sample.step_in_seg = k;
          sample.logical_idx = logical_idx;
          sample.alpha = alpha;
          sample.t_local = t;
          sample.t_global = t_global;
          sample.trap_weight = trap_weight;
          sample.dt = dt;
          sample.b_p = b_p;
          sample.p = p;
          sample.v = v;
          workspace_->samples.push_back(sample);
        }
      }

      seg_start_time += T;
    }

    double accumulator = 0.0;
    for (int i = piece_num_ - 1; i > 0; --i)
    {
      accumulator += global_time_grad(i);
      workspace_->gdT_integral(i - 1) += accumulator;
    }
  }

  template <typename CostManager>
  void accumulateSampleCost(CostManager &&cost_manager, double &cost)
  {
    workspace_->gdC_sample.setZero();
    workspace_->gdT_sample.setZero();

    const Eigen::Index sample_count = static_cast<Eigen::Index>(workspace_->samples.size());
    workspace_->sample_grad_p.resize(DIM, sample_count);
    workspace_->sample_grad_p.setZero();
    workspace_->sample_grad_t_global.resize(sample_count);
    workspace_->sample_grad_t_global.setZero();

    cost += cost_manager.evaluateSample(workspace_->samples,
                                        workspace_->sample_grad_p,
                                        workspace_->sample_grad_t_global);

    Eigen::VectorXd global_time_grad = Eigen::VectorXd::Zero(piece_num_);
    for (Eigen::Index sample_idx = 0; sample_idx < sample_count; ++sample_idx)
    {
      const auto &sample = workspace_->samples[sample_idx];
      const int base_row = sample.seg_idx * TrajType::COEFF_NUM;
      const VectorType grad_position = workspace_->sample_grad_p.col(sample_idx);

      workspace_->gdC_sample.template block<TrajType::COEFF_NUM, DIM>(base_row, 0).noalias() +=
          sample.b_p.transpose() * grad_position.transpose();
      workspace_->gdT_sample(sample.seg_idx) += grad_position.dot(sample.v) * sample.alpha;

      const double grad_time = workspace_->sample_grad_t_global(sample_idx);
      workspace_->gdT_sample(sample.seg_idx) += grad_time * sample.alpha;
      global_time_grad(sample.seg_idx) += grad_time;
    }

    double accumulator = 0.0;
    for (int i = piece_num_ - 1; i > 0; --i)
    {
      accumulator += global_time_grad(i);
      workspace_->gdT_sample(i - 1) += accumulator;
    }
  }

  void writeDecisionGradient(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::VectorXd> grad_out) const
  {
    for (int i = 0; i < piece_num_; ++i)
    {
      grad_out(i) += active_time_map_->backward(x(i), workspace_->cache_T(i), workspace_->grad_by_times(i));
    }

    int offset = piece_num_;
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = active_spatial_map_->getUnconstrainedDim(i);
      const VectorType grad_p = workspace_->grad_by_points.col(i - 1);
      grad_out.segment(offset, dof) += active_spatial_map_->backwardGrad(x.segment(offset, dof), grad_p, i);
      offset += dof;
    }
  }

private:
  TrajType traj_;
  BoundaryState nominal_head_state_{BoundaryState::Zero()};
  BoundaryState nominal_tail_state_{BoundaryState::Zero()};
  int piece_num_{0};
  int samples_per_piece_{5};
  double rho_energy_{1.0};

  std::vector<double> ref_times_;
  WaypointsType ref_waypoints_;
  bool has_warm_start_guess_{false};
  Eigen::VectorXd warm_start_guess_;

  TimeMap default_time_map_;
  SpatialMap default_spatial_map_;
  const TimeMap *active_time_map_{nullptr};
  const SpatialMap *active_spatial_map_{nullptr};

  std::unique_ptr<Workspace> workspace_;
};

} // namespace minco

#endif
