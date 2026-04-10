#ifndef MINCO_TRAJECTORY_HPP
#define MINCO_TRAJECTORY_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace minco
{

class BandedSystem
{
public:
  void create(int n, int lower_bw, int upper_bw)
  {
    destroy();
    N_ = n;
    lower_bw_ = lower_bw;
    upper_bw_ = upper_bw;
    data_.assign(static_cast<std::size_t>(N_) *
                     static_cast<std::size_t>(lower_bw_ + upper_bw_ + 1),
                 0.0);
  }

  void destroy()
  {
    N_ = 0;
    lower_bw_ = 0;
    upper_bw_ = 0;
    data_.clear();
  }

  void reset()
  {
    std::fill(data_.begin(), data_.end(), 0.0);
  }

  const double &operator()(int i, int j) const
  {
    return data_[static_cast<std::size_t>(i - j + upper_bw_) *
                     static_cast<std::size_t>(N_) +
                 static_cast<std::size_t>(j)];
  }

  double &operator()(int i, int j)
  {
    return data_[static_cast<std::size_t>(i - j + upper_bw_) *
                     static_cast<std::size_t>(N_) +
                 static_cast<std::size_t>(j)];
  }

  void factorizeLU()
  {
    int iM, jM;
    double cVl;
    for (int k = 0; k <= N_ - 2; ++k)
    {
      iM = std::min(k + lower_bw_, N_ - 1);
      cVl = operator()(k, k);

      for (int i = k + 1; i <= iM; ++i)
      {
        if (operator()(i, k) != 0.0)
        {
          operator()(i, k) /= cVl;
        }
      }

      jM = std::min(k + upper_bw_, N_ - 1);
      for (int j = k + 1; j <= jM; ++j)
      {
        cVl = operator()(k, j);
        if (cVl != 0.0)
        {
          for (int i = k + 1; i <= iM; ++i)
          {
            if (operator()(i, k) != 0.0)
            {
              operator()(i, j) -= operator()(i, k) * cVl;
            }
          }
        }
      }
    }
  }

  template <typename Derived>
  void solve(Eigen::MatrixBase<Derived> &b) const
  {
    int iM;
    for (int j = 0; j <= N_ - 1; ++j)
    {
      iM = std::min(j + lower_bw_, N_ - 1);
      for (int i = j + 1; i <= iM; ++i)
      {
        if (operator()(i, j) != 0.0)
        {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }

    for (int j = N_ - 1; j >= 0; --j)
    {
      b.row(j) /= operator()(j, j);
      iM = std::max(0, j - upper_bw_);
      for (int i = iM; i <= j - 1; ++i)
      {
        if (operator()(i, j) != 0.0)
        {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
  }

  template <typename Derived>
  void solveAdj(Eigen::MatrixBase<Derived> &b) const
  {
    int iM;
    for (int j = 0; j <= N_ - 1; ++j)
    {
      b.row(j) /= operator()(j, j);
      iM = std::min(j + upper_bw_, N_ - 1);
      for (int i = j + 1; i <= iM; ++i)
      {
        if (operator()(j, i) != 0.0)
        {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }

    for (int j = N_ - 1; j >= 0; --j)
    {
      iM = std::max(0, j - lower_bw_);
      for (int i = iM; i <= j - 1; ++i)
      {
        if (operator()(j, i) != 0.0)
        {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
  }

private:
  int N_{0};
  int lower_bw_{0};
  int upper_bw_{0};
  std::vector<double> data_;
};

template <int S>
struct MincoGenericKernel
{
  static_assert(S >= 2, "MINCO requires S >= 2.");

  static constexpr int kS = S;
  static constexpr int kOrder = 2 * S - 1;
  static constexpr int kCoeffNum = 2 * S;

  using BasisRow = Eigen::Matrix<double, 1, kCoeffNum>;
  using QMat = Eigen::Matrix<double, kCoeffNum, kCoeffNum>;

  static double fallingFactorial(int n, int r)
  {
    if (r < 0 || r > n)
    {
      return 0.0;
    }

    double out = 1.0;
    for (int i = 0; i < r; ++i)
    {
      out *= static_cast<double>(n - i);
    }
    return out;
  }

  static BasisRow derivativeBasis(int derivative_order, double t)
  {
    BasisRow row = BasisRow::Zero();
    if (derivative_order < 0)
    {
      return row;
    }

    for (int k = derivative_order; k <= kOrder; ++k)
    {
      row(k) = fallingFactorial(k, derivative_order) *
               std::pow(t, static_cast<double>(k - derivative_order));
    }
    return row;
  }

  static QMat controlCostHessian(double T)
  {
    QMat Q = QMat::Zero();
    for (int i = S; i < kCoeffNum; ++i)
    {
      const double ci = fallingFactorial(i, S);
      for (int j = S; j < kCoeffNum; ++j)
      {
        const double cj = fallingFactorial(j, S);
        const int power = i + j - 2 * S + 1;
        Q(i, j) = ci * cj * std::pow(T, static_cast<double>(power)) /
                  static_cast<double>(power);
      }
    }
    return Q;
  }

  static QMat dControlCostHessian_dT(double T)
  {
    QMat dQ = QMat::Zero();
    for (int i = S; i < kCoeffNum; ++i)
    {
      const double ci = fallingFactorial(i, S);
      for (int j = S; j < kCoeffNum; ++j)
      {
        const double cj = fallingFactorial(j, S);
        const int power = i + j - 2 * S;
        dQ(i, j) = ci * cj * std::pow(T, static_cast<double>(power));
      }
    }
    return dQ;
  }

  template <int DIM>
  static void accumulateTimeGradientFromAdjoint(
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &coeffs,
      const Eigen::VectorXd &durations,
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &adj_grad,
      Eigen::VectorXd &grad_by_times,
      int piece_num)
  {
    for (int i = 0; i < piece_num - 1; ++i)
    {
      const double T = durations(i);
      const auto coeff_block = coeffs.template block<kCoeffNum, DIM>(i * kCoeffNum, 0);
      int row = i * kCoeffNum + S;

      for (int r = S; r <= kOrder - 1; ++r, ++row)
      {
        const BasisRow basis = derivativeBasis(r + 1, T);
        const Eigen::Matrix<double, DIM, 1> value = coeff_block.transpose() * basis.transpose();
        grad_by_times(i) -= adj_grad.row(row).transpose().dot(value);
      }

      {
        const BasisRow basis = derivativeBasis(1, T);
        const Eigen::Matrix<double, DIM, 1> value = coeff_block.transpose() * basis.transpose();
        grad_by_times(i) -= adj_grad.row(row).transpose().dot(value);
        ++row;
        grad_by_times(i) -= adj_grad.row(row).transpose().dot(value);
        ++row;
      }

      for (int r = 1; r <= S - 1; ++r, ++row)
      {
        const BasisRow basis = derivativeBasis(r + 1, T);
        const Eigen::Matrix<double, DIM, 1> value = coeff_block.transpose() * basis.transpose();
        grad_by_times(i) -= adj_grad.row(row).transpose().dot(value);
      }
    }

    const int last_piece = piece_num - 1;
    const double T_last = durations(last_piece);
    const auto coeff_block = coeffs.template block<kCoeffNum, DIM>(last_piece * kCoeffNum, 0);
    const int tail_row = kCoeffNum * piece_num - S;
    for (int r = 0; r < S; ++r)
    {
      const BasisRow basis = derivativeBasis(r + 1, T_last);
      const Eigen::Matrix<double, DIM, 1> value = coeff_block.transpose() * basis.transpose();
      grad_by_times(last_piece) -= adj_grad.row(tail_row + r).transpose().dot(value);
    }
  }
};

template <int S>
struct MincoKernel : MincoGenericKernel<S>
{
};

template <>
struct MincoKernel<2>
{
  static constexpr int kS = 2;
  static constexpr int kOrder = 3;
  static constexpr int kCoeffNum = 4;

  using BasisRow = Eigen::Matrix<double, 1, kCoeffNum>;
  using QMat = Eigen::Matrix<double, kCoeffNum, kCoeffNum>;

  static BasisRow derivativeBasis(int derivative_order, double t)
  {
    const double t2 = t * t;
    BasisRow row = BasisRow::Zero();
    switch (derivative_order)
    {
    case 0:
      row << 1.0, t, t2, t2 * t;
      break;
    case 1:
      row << 0.0, 1.0, 2.0 * t, 3.0 * t2;
      break;
    case 2:
      row << 0.0, 0.0, 2.0, 6.0 * t;
      break;
    case 3:
      row << 0.0, 0.0, 0.0, 6.0;
      break;
    default:
      break;
    }
    return row;
  }

  static QMat controlCostHessian(double T)
  {
    QMat Q = QMat::Zero();
    const double T2 = T * T;
    const double T3 = T2 * T;

    Q(2, 2) = 4.0 * T;
    Q(2, 3) = 6.0 * T2;
    Q(3, 2) = Q(2, 3);
    Q(3, 3) = 12.0 * T3;
    return Q;
  }

  static QMat dControlCostHessian_dT(double T)
  {
    QMat dQ = QMat::Zero();
    const double T2 = T * T;

    dQ(2, 2) = 4.0;
    dQ(2, 3) = 12.0 * T;
    dQ(3, 2) = dQ(2, 3);
    dQ(3, 3) = 36.0 * T2;
    return dQ;
  }

  template <int DIM>
  static void accumulateTimeGradientFromAdjoint(
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &coeffs,
      const Eigen::VectorXd &durations,
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &adj_grad,
      Eigen::VectorXd &grad_by_times,
      int piece_num)
  {
    for (int i = 0; i < piece_num - 1; ++i)
    {
      const double T = durations(i);
      const auto c1 = coeffs.row(4 * i + 1);
      const auto c2 = coeffs.row(4 * i + 2);
      const auto c3 = coeffs.row(4 * i + 3);

      const auto vel = c1 + 2.0 * T * c2 + 3.0 * T * T * c3;
      const auto acc = 2.0 * c2 + 6.0 * T * c3;
      const auto jer = 6.0 * c3;

      grad_by_times(i) -= adj_grad.row(4 * i + 2).dot(jer);
      grad_by_times(i) -= adj_grad.row(4 * i + 3).dot(vel);
      grad_by_times(i) -= adj_grad.row(4 * i + 4).dot(vel);
      grad_by_times(i) -= adj_grad.row(4 * i + 5).dot(acc);
    }

    const int last = piece_num - 1;
    const double T = durations(last);
    const auto c1 = coeffs.row(4 * last + 1);
    const auto c2 = coeffs.row(4 * last + 2);
    const auto c3 = coeffs.row(4 * last + 3);

    const auto vel = c1 + 2.0 * T * c2 + 3.0 * T * T * c3;
    const auto acc = 2.0 * c2 + 6.0 * T * c3;

    grad_by_times(last) -= adj_grad.row(4 * piece_num - 2).dot(vel);
    grad_by_times(last) -= adj_grad.row(4 * piece_num - 1).dot(acc);
  }
};

template <>
struct MincoKernel<3>
{
  static constexpr int kS = 3;
  static constexpr int kOrder = 5;
  static constexpr int kCoeffNum = 6;

  using BasisRow = Eigen::Matrix<double, 1, kCoeffNum>;
  using QMat = Eigen::Matrix<double, kCoeffNum, kCoeffNum>;

  static BasisRow derivativeBasis(int derivative_order, double t)
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    BasisRow row = BasisRow::Zero();
    switch (derivative_order)
    {
    case 0:
      row << 1.0, t, t2, t3, t4, t5;
      break;
    case 1:
      row << 0.0, 1.0, 2.0 * t, 3.0 * t2, 4.0 * t3, 5.0 * t4;
      break;
    case 2:
      row << 0.0, 0.0, 2.0, 6.0 * t, 12.0 * t2, 20.0 * t3;
      break;
    case 3:
      row << 0.0, 0.0, 0.0, 6.0, 24.0 * t, 60.0 * t2;
      break;
    case 4:
      row << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t;
      break;
    case 5:
      row << 0.0, 0.0, 0.0, 0.0, 0.0, 120.0;
      break;
    default:
      break;
    }
    return row;
  }

  static QMat controlCostHessian(double T)
  {
    QMat Q = QMat::Zero();
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;

    Q(3, 3) = 36.0 * T;
    Q(3, 4) = 72.0 * T2;
    Q(4, 3) = Q(3, 4);
    Q(3, 5) = 120.0 * T3;
    Q(5, 3) = Q(3, 5);
    Q(4, 4) = 192.0 * T3;
    Q(4, 5) = 360.0 * T4;
    Q(5, 4) = Q(4, 5);
    Q(5, 5) = 720.0 * T5;
    return Q;
  }

  static QMat dControlCostHessian_dT(double T)
  {
    QMat dQ = QMat::Zero();
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;

    dQ(3, 3) = 36.0;
    dQ(3, 4) = 144.0 * T;
    dQ(4, 3) = dQ(3, 4);
    dQ(3, 5) = 360.0 * T2;
    dQ(5, 3) = dQ(3, 5);
    dQ(4, 4) = 576.0 * T2;
    dQ(4, 5) = 1440.0 * T3;
    dQ(5, 4) = dQ(4, 5);
    dQ(5, 5) = 3600.0 * T4;
    return dQ;
  }

  template <int DIM>
  static void accumulateTimeGradientFromAdjoint(
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &coeffs,
      const Eigen::VectorXd &durations,
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &adj_grad,
      Eigen::VectorXd &grad_by_times,
      int piece_num)
  {
    for (int i = 0; i < piece_num - 1; ++i)
    {
      const double T = durations(i);
      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T3 * T;

      const auto c1 = coeffs.row(6 * i + 1);
      const auto c2 = coeffs.row(6 * i + 2);
      const auto c3 = coeffs.row(6 * i + 3);
      const auto c4 = coeffs.row(6 * i + 4);
      const auto c5 = coeffs.row(6 * i + 5);

      const auto vel = c1 + 2.0 * T * c2 + 3.0 * T2 * c3 + 4.0 * T3 * c4 + 5.0 * T4 * c5;
      const auto acc = 2.0 * c2 + 6.0 * T * c3 + 12.0 * T2 * c4 + 20.0 * T3 * c5;
      const auto jer = 6.0 * c3 + 24.0 * T * c4 + 60.0 * T2 * c5;
      const auto snap = 24.0 * c4 + 120.0 * T * c5;
      const auto crackle = 120.0 * c5;

      grad_by_times(i) -= adj_grad.row(6 * i + 3).dot(snap);
      grad_by_times(i) -= adj_grad.row(6 * i + 4).dot(crackle);
      grad_by_times(i) -= adj_grad.row(6 * i + 5).dot(vel);
      grad_by_times(i) -= adj_grad.row(6 * i + 6).dot(vel);
      grad_by_times(i) -= adj_grad.row(6 * i + 7).dot(acc);
      grad_by_times(i) -= adj_grad.row(6 * i + 8).dot(jer);
    }

    const int last = piece_num - 1;
    const double T = durations(last);
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;

    const auto c1 = coeffs.row(6 * last + 1);
    const auto c2 = coeffs.row(6 * last + 2);
    const auto c3 = coeffs.row(6 * last + 3);
    const auto c4 = coeffs.row(6 * last + 4);
    const auto c5 = coeffs.row(6 * last + 5);

    const auto vel = c1 + 2.0 * T * c2 + 3.0 * T2 * c3 + 4.0 * T3 * c4 + 5.0 * T4 * c5;
    const auto acc = 2.0 * c2 + 6.0 * T * c3 + 12.0 * T2 * c4 + 20.0 * T3 * c5;
    const auto jer = 6.0 * c3 + 24.0 * T * c4 + 60.0 * T2 * c5;

    grad_by_times(last) -= adj_grad.row(6 * piece_num - 3).dot(vel);
    grad_by_times(last) -= adj_grad.row(6 * piece_num - 2).dot(acc);
    grad_by_times(last) -= adj_grad.row(6 * piece_num - 1).dot(jer);
  }
};

template <>
struct MincoKernel<4>
{
  static constexpr int kS = 4;
  static constexpr int kOrder = 7;
  static constexpr int kCoeffNum = 8;

  using BasisRow = Eigen::Matrix<double, 1, kCoeffNum>;
  using QMat = Eigen::Matrix<double, kCoeffNum, kCoeffNum>;

  static BasisRow derivativeBasis(int derivative_order, double t)
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    const double t7 = t6 * t;
    BasisRow row = BasisRow::Zero();
    switch (derivative_order)
    {
    case 0:
      row << 1.0, t, t2, t3, t4, t5, t6, t7;
      break;
    case 1:
      row << 0.0, 1.0, 2.0 * t, 3.0 * t2, 4.0 * t3, 5.0 * t4, 6.0 * t5, 7.0 * t6;
      break;
    case 2:
      row << 0.0, 0.0, 2.0, 6.0 * t, 12.0 * t2, 20.0 * t3, 30.0 * t4, 42.0 * t5;
      break;
    case 3:
      row << 0.0, 0.0, 0.0, 6.0, 24.0 * t, 60.0 * t2, 120.0 * t3, 210.0 * t4;
      break;
    case 4:
      row << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t, 360.0 * t2, 840.0 * t3;
      break;
    case 5:
      row << 0.0, 0.0, 0.0, 0.0, 0.0, 120.0, 720.0 * t, 2520.0 * t2;
      break;
    case 6:
      row << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 720.0, 5040.0 * t;
      break;
    case 7:
      row << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5040.0;
      break;
    default:
      break;
    }
    return row;
  }

  static QMat controlCostHessian(double T)
  {
    QMat Q = QMat::Zero();
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const double T6 = T5 * T;
    const double T7 = T6 * T;

    Q(4, 4) = 576.0 * T;
    Q(4, 5) = 1440.0 * T2;
    Q(5, 4) = Q(4, 5);
    Q(4, 6) = 2880.0 * T3;
    Q(6, 4) = Q(4, 6);
    Q(4, 7) = 5040.0 * T4;
    Q(7, 4) = Q(4, 7);
    Q(5, 5) = 4800.0 * T3;
    Q(5, 6) = 10800.0 * T4;
    Q(6, 5) = Q(5, 6);
    Q(5, 7) = 20160.0 * T5;
    Q(7, 5) = Q(5, 7);
    Q(6, 6) = 25920.0 * T5;
    Q(6, 7) = 50400.0 * T6;
    Q(7, 6) = Q(6, 7);
    Q(7, 7) = 100800.0 * T7;
    return Q;
  }

  static QMat dControlCostHessian_dT(double T)
  {
    QMat dQ = QMat::Zero();
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const double T6 = T5 * T;

    dQ(4, 4) = 576.0;
    dQ(4, 5) = 2880.0 * T;
    dQ(5, 4) = dQ(4, 5);
    dQ(4, 6) = 8640.0 * T2;
    dQ(6, 4) = dQ(4, 6);
    dQ(4, 7) = 20160.0 * T3;
    dQ(7, 4) = dQ(4, 7);
    dQ(5, 5) = 14400.0 * T2;
    dQ(5, 6) = 43200.0 * T3;
    dQ(6, 5) = dQ(5, 6);
    dQ(5, 7) = 100800.0 * T4;
    dQ(7, 5) = dQ(5, 7);
    dQ(6, 6) = 129600.0 * T4;
    dQ(6, 7) = 302400.0 * T5;
    dQ(7, 6) = dQ(6, 7);
    dQ(7, 7) = 705600.0 * T6;
    return dQ;
  }

  template <int DIM>
  static void accumulateTimeGradientFromAdjoint(
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &coeffs,
      const Eigen::VectorXd &durations,
      const Eigen::Matrix<double, Eigen::Dynamic, DIM> &adj_grad,
      Eigen::VectorXd &grad_by_times,
      int piece_num)
  {
    for (int i = 0; i < piece_num - 1; ++i)
    {
      const double T = durations(i);
      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T3 * T;
      const double T5 = T4 * T;
      const double T6 = T5 * T;

      const auto c1 = coeffs.row(8 * i + 1);
      const auto c2 = coeffs.row(8 * i + 2);
      const auto c3 = coeffs.row(8 * i + 3);
      const auto c4 = coeffs.row(8 * i + 4);
      const auto c5 = coeffs.row(8 * i + 5);
      const auto c6 = coeffs.row(8 * i + 6);
      const auto c7 = coeffs.row(8 * i + 7);

      const auto vel = c1 + 2.0 * T * c2 + 3.0 * T2 * c3 + 4.0 * T3 * c4 +
                       5.0 * T4 * c5 + 6.0 * T5 * c6 + 7.0 * T6 * c7;
      const auto acc = 2.0 * c2 + 6.0 * T * c3 + 12.0 * T2 * c4 + 20.0 * T3 * c5 +
                       30.0 * T4 * c6 + 42.0 * T5 * c7;
      const auto jer = 6.0 * c3 + 24.0 * T * c4 + 60.0 * T2 * c5 +
                       120.0 * T3 * c6 + 210.0 * T4 * c7;
      const auto snap = 24.0 * c4 + 120.0 * T * c5 + 360.0 * T2 * c6 + 840.0 * T3 * c7;
      const auto d5 = 120.0 * c5 + 720.0 * T * c6 + 2520.0 * T2 * c7;
      const auto d6 = 720.0 * c6 + 5040.0 * T * c7;
      const auto d7 = 5040.0 * c7;

      grad_by_times(i) -= adj_grad.row(8 * i + 4).dot(d5);
      grad_by_times(i) -= adj_grad.row(8 * i + 5).dot(d6);
      grad_by_times(i) -= adj_grad.row(8 * i + 6).dot(d7);
      grad_by_times(i) -= adj_grad.row(8 * i + 7).dot(vel);
      grad_by_times(i) -= adj_grad.row(8 * i + 8).dot(vel);
      grad_by_times(i) -= adj_grad.row(8 * i + 9).dot(acc);
      grad_by_times(i) -= adj_grad.row(8 * i + 10).dot(jer);
      grad_by_times(i) -= adj_grad.row(8 * i + 11).dot(snap);
    }

    const int last = piece_num - 1;
    const double T = durations(last);
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const double T6 = T5 * T;

    const auto c1 = coeffs.row(8 * last + 1);
    const auto c2 = coeffs.row(8 * last + 2);
    const auto c3 = coeffs.row(8 * last + 3);
    const auto c4 = coeffs.row(8 * last + 4);
    const auto c5 = coeffs.row(8 * last + 5);
    const auto c6 = coeffs.row(8 * last + 6);
    const auto c7 = coeffs.row(8 * last + 7);

    const auto vel = c1 + 2.0 * T * c2 + 3.0 * T2 * c3 + 4.0 * T3 * c4 +
                     5.0 * T4 * c5 + 6.0 * T5 * c6 + 7.0 * T6 * c7;
    const auto acc = 2.0 * c2 + 6.0 * T * c3 + 12.0 * T2 * c4 + 20.0 * T3 * c5 +
                     30.0 * T4 * c6 + 42.0 * T5 * c7;
    const auto jer = 6.0 * c3 + 24.0 * T * c4 + 60.0 * T2 * c5 +
                     120.0 * T3 * c6 + 210.0 * T4 * c7;
    const auto snap = 24.0 * c4 + 120.0 * T * c5 + 360.0 * T2 * c6 + 840.0 * T3 * c7;

    grad_by_times(last) -= adj_grad.row(8 * piece_num - 4).dot(vel);
    grad_by_times(last) -= adj_grad.row(8 * piece_num - 3).dot(acc);
    grad_by_times(last) -= adj_grad.row(8 * piece_num - 2).dot(jer);
    grad_by_times(last) -= adj_grad.row(8 * piece_num - 1).dot(snap);
  }
};

template <int DIM, int S = 3>
class MINCOTrajectory
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Traits = MincoKernel<S>;

  static constexpr int DIMENSION = DIM;
  static constexpr int S_ORDER = Traits::kS;
  static constexpr int ORDER = Traits::kOrder;
  static constexpr int COEFF_NUM = Traits::kCoeffNum;
  static constexpr int BOUNDARY_DERIVATIVE_NUM = S;

  using VectorD = Eigen::Matrix<double, DIM, 1>;
  using BoundaryState = Eigen::Matrix<double, DIM, S>;
  using InnerPointsMat = Eigen::Matrix<double, DIM, Eigen::Dynamic>;
  using CoeffMat = Eigen::Matrix<double, Eigen::Dynamic, DIM>;
  using PieceCoeffMat = Eigen::Matrix<double, DIM, COEFF_NUM>;
  using BasisRow = typename Traits::BasisRow;
  using QMat = typename Traits::QMat;

  struct MINCOGradResult
  {
    InnerPointsMat grad_by_points;
    Eigen::VectorXd grad_by_times;
    BoundaryState grad_by_head_state{BoundaryState::Zero()};
    BoundaryState grad_by_tail_state{BoundaryState::Zero()};
  };

  MINCOTrajectory() = default;
  ~MINCOTrajectory()
  {
    system_.destroy();
  }

  template <typename Derived>
  bool generate(const Eigen::MatrixBase<Derived> &inner_points_expr,
                const BoundaryState &head_state,
                const BoundaryState &tail_state,
                const Eigen::VectorXd &durations)
  {
    InnerPointsMat inner_points;
    if (inner_points_expr.size() == 0)
    {
      inner_points.resize(DIM, 0);
    }
    else
    {
      if (inner_points_expr.rows() != DIM)
      {
        return false;
      }
      inner_points = inner_points_expr;
    }

    return generate(inner_points, head_state, tail_state, durations);
  }

  void reset(const BoundaryState &head_state,
             const BoundaryState &tail_state,
             int piece_num)
  {
    piece_num_ = piece_num;
    head_state_ = head_state;
    tail_state_ = tail_state;
    system_.create(COEFF_NUM * piece_num_, COEFF_NUM, COEFF_NUM);
    coeffs_.resize(COEFF_NUM * piece_num_, DIM);
    durations_.resize(piece_num_);
  }

  bool generate(const InnerPointsMat &inner_points,
                const BoundaryState &head_state,
                const BoundaryState &tail_state,
                const Eigen::VectorXd &durations)
  {
    if (durations.size() <= 0)
    {
      return false;
    }
    if ((durations.array() <= 0.0).any())
    {
      return false;
    }

    const int expected_inner_points = std::max(0, static_cast<int>(durations.size()) - 1);
    if (inner_points.cols() != expected_inner_points)
    {
      return false;
    }

    reset(head_state, tail_state, durations.size());
    durations_ = durations;

    system_.reset();
    coeffs_.setZero();

    buildHeadRows();
    buildInternalRows(inner_points);
    buildTailRows();

    system_.factorizeLU();
    system_.solve(coeffs_);
    return true;
  }

  int getPieceNum() const { return piece_num_; }
  const Eigen::VectorXd &getDurations() const { return durations_; }
  double getTotalDuration() const { return durations_.sum(); }
  const CoeffMat &getCoefficients() const { return coeffs_; }

  static BasisRow derivativeBasis(int derivative_order, double t)
  {
    return Traits::derivativeBasis(derivative_order, t);
  }

  static void computeBasisFunctions(double t,
                                    BasisRow &b_p,
                                    BasisRow &b_v,
                                    BasisRow &b_a,
                                    BasisRow &b_j,
                                    BasisRow &b_s)
  {
    b_p = derivativeBasis(0, t);
    b_v = derivativeBasis(1, t);
    b_a = derivativeBasis(2, t);
    b_j = derivativeBasis(3, t);
    b_s = derivativeBasis(4, t);
  }

  VectorD evaluate(double t, int derivative_order = 0) const
  {
    if (piece_num_ <= 0)
    {
      return VectorD::Zero();
    }

    int piece_idx = 0;
    double local_t = 0.0;
    locatePieceAndTime(t, piece_idx, local_t);
    return evaluatePiece(piece_idx, local_t, derivative_order);
  }

  VectorD getPos(double t) const { return evaluate(t, 0); }
  VectorD getVel(double t) const { return evaluate(t, 1); }
  VectorD getAcc(double t) const { return evaluate(t, 2); }
  VectorD getJer(double t) const { return evaluate(t, 3); }
  VectorD getSnap(double t) const { return evaluate(t, 4); }

  Eigen::Matrix<double, DIM, Eigen::Dynamic> getPositions() const
  {
    Eigen::Matrix<double, DIM, Eigen::Dynamic> positions(DIM, piece_num_ + 1);
    double accum_t = 0.0;
    for (int i = 0; i <= piece_num_; ++i)
    {
      positions.col(i) = evaluate(accum_t, 0);
      if (i < piece_num_)
      {
        accum_t += durations_(i);
      }
    }
    return positions;
  }

  PieceCoeffMat getPieceCoeffMat(int piece_idx) const
  {
    return coeffs_.template block<COEFF_NUM, DIM>(piece_idx * COEFF_NUM, 0)
        .transpose()
        .rowwise()
        .reverse();
  }

  Eigen::Matrix<double, DIM, Eigen::Dynamic> getInitConstraintPoints(int samples_per_piece) const
  {
    const int K = std::max(1, samples_per_piece);
    Eigen::Matrix<double, DIM, Eigen::Dynamic> pts(DIM, piece_num_ * K + 1);
    int idx = 0;
    for (int i = 0; i < piece_num_; ++i)
    {
      const double step = durations_(i) / static_cast<double>(K);
      for (int j = 0; j <= K; ++j)
      {
        const double t = step * static_cast<double>(j);
        const auto coeff_block = coeffs_.template block<COEFF_NUM, DIM>(i * COEFF_NUM, 0);
        const BasisRow b_p = derivativeBasis(0, t);
        pts.col(idx).transpose().noalias() = b_p * coeff_block;
        if (j != K || i == piece_num_ - 1)
        {
          ++idx;
        }
      }
    }
    return pts;
  }

  double getEnergy() const
  {
    double energy = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      const QMat Q = Traits::controlCostHessian(durations_(i));
      const auto C = coeffs_.template block<COEFF_NUM, DIM>(i * COEFF_NUM, 0);
      energy += (C.transpose() * Q * C).trace();
    }
    return energy;
  }

  void getEnergyPartialGradByCoeffs(double &energy, CoeffMat &gdC) const
  {
    energy = getEnergy();
    gdC.resize(COEFF_NUM * piece_num_, DIM);
    gdC.setZero();

    for (int i = 0; i < piece_num_; ++i)
    {
      const QMat Q = Traits::controlCostHessian(durations_(i));
      const auto C = coeffs_.template block<COEFF_NUM, DIM>(i * COEFF_NUM, 0);
      gdC.template block<COEFF_NUM, DIM>(i * COEFF_NUM, 0).noalias() += 2.0 * Q * C;
    }
  }

  void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
  {
    gdT.resize(piece_num_);
    gdT.setZero();

    for (int i = 0; i < piece_num_; ++i)
    {
      const QMat dQ = Traits::dControlCostHessian_dT(durations_(i));
      const auto C = coeffs_.template block<COEFF_NUM, DIM>(i * COEFF_NUM, 0);
      gdT(i) += (C.transpose() * dQ * C).trace();
    }
  }

  void propagateGrad(const CoeffMat &partial_grad_by_coeffs,
                     const Eigen::VectorXd &partial_grad_by_times,
                     InnerPointsMat &grad_by_points,
                     Eigen::VectorXd &grad_by_times) const
  {
    BoundaryState grad_by_head_state = BoundaryState::Zero();
    BoundaryState grad_by_tail_state = BoundaryState::Zero();
    propagateGradFull(partial_grad_by_coeffs,
                      partial_grad_by_times,
                      grad_by_points,
                      grad_by_times,
                      grad_by_head_state,
                      grad_by_tail_state);
  }

  MINCOGradResult propagateGradFull(const CoeffMat &partial_grad_by_coeffs,
                                    const Eigen::VectorXd &partial_grad_by_times) const
  {
    MINCOGradResult result;
    propagateGradFull(partial_grad_by_coeffs,
                      partial_grad_by_times,
                      result.grad_by_points,
                      result.grad_by_times,
                      result.grad_by_head_state,
                      result.grad_by_tail_state);
    return result;
  }

  void propagateGradFull(const CoeffMat &partial_grad_by_coeffs,
                         const Eigen::VectorXd &partial_grad_by_times,
                         InnerPointsMat &grad_by_points,
                         Eigen::VectorXd &grad_by_times,
                         BoundaryState &grad_by_head_state,
                         BoundaryState &grad_by_tail_state,
                         CoeffMat *adjoint_out = nullptr) const
  {
    grad_by_points.resize(DIM, std::max(0, piece_num_ - 1));
    grad_by_points.setZero();
    grad_by_times.resize(piece_num_);
    grad_by_times.setZero();
    grad_by_head_state.setZero();
    grad_by_tail_state.setZero();

    if (piece_num_ <= 0)
    {
      return;
    }

    CoeffMat adj_grad = partial_grad_by_coeffs;
    system_.solveAdj(adj_grad);

    // Mathematical meaning:
    // The coefficient-elimination system is M(T)c = b(head, inner, tail).
    // Head and tail states only enter the RHS b. After solving the adjoint
    // M(T)^T lambda = dJ/dc, the RHS sensitivity is lambda. Therefore:
    //   dJ/d(head_state) = lambda rows owned by head boundary,
    //   dJ/d(tail_state) = lambda rows owned by tail boundary.
    for (int r = 0; r < S; ++r)
    {
      grad_by_head_state.col(r) = adj_grad.row(r).transpose();
    }
    const int tail_start_row = COEFF_NUM * piece_num_ - S;
    for (int r = 0; r < S; ++r)
    {
      grad_by_tail_state.col(r) = adj_grad.row(tail_start_row + r).transpose();
    }

    for (int i = 0; i < piece_num_ - 1; ++i)
    {
      const int row = i * COEFF_NUM + (COEFF_NUM - 1);
      grad_by_points.col(i) = adj_grad.row(row).transpose();
    }

    Traits::template accumulateTimeGradientFromAdjoint<DIM>(
        coeffs_, durations_, adj_grad, grad_by_times, piece_num_);

    grad_by_times += partial_grad_by_times;

    if (adjoint_out != nullptr)
    {
      *adjoint_out = adj_grad;
    }
  }

private:
  void buildHeadRows()
  {
    for (int r = 0; r < S; ++r)
    {
      addBasisToRow(r, 0, r, 0.0, 1.0);
      coeffs_.row(r) = head_state_.col(r).transpose();
    }
  }

  void buildInternalRows(const InnerPointsMat &inner_points)
  {
    for (int i = 0; i < piece_num_ - 1; ++i)
    {
      int row = i * COEFF_NUM + S;
      const double T = durations_(i);

      for (int r = S; r <= ORDER - 1; ++r, ++row)
      {
        addBasisToRow(row, i, r, T, +1.0);
        addBasisToRow(row, i + 1, r, 0.0, -1.0);
      }

      addBasisToRow(row, i, 0, T, +1.0);
      coeffs_.row(row) = inner_points.col(i).transpose();
      ++row;

      addBasisToRow(row, i, 0, T, +1.0);
      addBasisToRow(row, i + 1, 0, 0.0, -1.0);
      ++row;

      for (int r = 1; r <= S - 1; ++r, ++row)
      {
        addBasisToRow(row, i, r, T, +1.0);
        addBasisToRow(row, i + 1, r, 0.0, -1.0);
      }
    }
  }

  void buildTailRows()
  {
    const int start_row = COEFF_NUM * piece_num_ - S;
    const int last_piece = piece_num_ - 1;
    const double T_last = durations_(last_piece);

    for (int r = 0; r < S; ++r)
    {
      addBasisToRow(start_row + r, last_piece, r, T_last, 1.0);
      coeffs_.row(start_row + r) = tail_state_.col(r).transpose();
    }
  }

  void addBasisToRow(int row, int piece_idx, int derivative_order, double t, double scale)
  {
    const BasisRow basis = derivativeBasis(derivative_order, t);
    const int col0 = piece_idx * COEFF_NUM;
    for (int k = 0; k < COEFF_NUM; ++k)
    {
      if (basis(k) != 0.0)
      {
        system_(row, col0 + k) += scale * basis(k);
      }
    }
  }

  void locatePieceAndTime(double t, int &piece_idx, double &local_t) const
  {
    if (t <= 0.0)
    {
      piece_idx = 0;
      local_t = 0.0;
      return;
    }

    const double total = getTotalDuration();
    if (t >= total)
    {
      piece_idx = piece_num_ - 1;
      local_t = durations_(piece_idx);
      return;
    }

    double accum = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      if (t <= accum + durations_(i))
      {
        piece_idx = i;
        local_t = t - accum;
        return;
      }
      accum += durations_(i);
    }

    piece_idx = piece_num_ - 1;
    local_t = durations_(piece_idx);
  }

  VectorD evaluatePiece(int piece_idx, double local_t, int derivative_order) const
  {
    assert(piece_idx >= 0 && piece_idx < piece_num_);
    const BasisRow basis = derivativeBasis(derivative_order, local_t);
    const auto coeff_block = coeffs_.template block<COEFF_NUM, DIM>(piece_idx * COEFF_NUM, 0);
    return coeff_block.transpose() * basis.transpose();
  }

private:
  int piece_num_{0};
  BoundaryState head_state_{BoundaryState::Zero()};
  BoundaryState tail_state_{BoundaryState::Zero()};
  BandedSystem system_;
  CoeffMat coeffs_;
  Eigen::VectorXd durations_;
};

template <int DIM>
using MINCO_S2 = MINCOTrajectory<DIM, 2>;

template <int DIM>
using MINCO_S3 = MINCOTrajectory<DIM, 3>;

template <int DIM>
using MINCO_S4 = MINCOTrajectory<DIM, 4>;

} // namespace minco

#endif
