#ifndef MINCO_TRAJECTORY_HPP
#define MINCO_TRAJECTORY_HPP

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <vector>

namespace minco
{

template <int DIM>
class MINCOTrajectory
{
public:
  static constexpr int ORDER = 5;
  static constexpr int COEFF_NUM = 6;

  using VectorD = Eigen::Matrix<double, DIM, 1>;
  using BoundaryState = Eigen::Matrix<double, DIM, 3>;
  using InnerPointsMat = Eigen::Matrix<double, DIM, Eigen::Dynamic>;
  using CoeffMat = Eigen::Matrix<double, Eigen::Dynamic, DIM>;   // [1, t, t^2, ...] row-major by coefficient rows
  using PieceCoeffMat = Eigen::Matrix<double, DIM, COEFF_NUM>;   // [t^5 ... t^0] for external use
  using BasisRow = Eigen::Matrix<double, 1, COEFF_NUM>;

  class BandedSystem
  {
  public:
    void create(int n, int lower_bw, int upper_bw)
    {
      N_ = n;
      lower_bw_ = lower_bw;
      upper_bw_ = upper_bw;
      data_.assign(N_ * (lower_bw_ + upper_bw_ + 1), 0.0);
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
      return data_[(i - j + upper_bw_) * N_ + j];
    }

    double &operator()(int i, int j)
    {
      return data_[(i - j + upper_bw_) * N_ + j];
    }

    void factorizeLU()
    {
      int iM, jM;
      double cVl;
      for (int k = 0; k <= N_ - 2; k++)
      {
        iM = std::min(k + lower_bw_, N_ - 1);
        cVl = operator()(k, k);
        for (int i = k + 1; i <= iM; i++)
        {
          if (operator()(i, k) != 0.0)
          {
            operator()(i, k) /= cVl;
          }
        }
        jM = std::min(k + upper_bw_, N_ - 1);
        for (int j = k + 1; j <= jM; j++)
        {
          cVl = operator()(k, j);
          if (cVl != 0.0)
          {
            for (int i = k + 1; i <= iM; i++)
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
      for (int j = 0; j <= N_ - 1; j++)
      {
        iM = std::min(j + lower_bw_, N_ - 1);
        for (int i = j + 1; i <= iM; i++)
        {
          if (operator()(i, j) != 0.0)
          {
            b.row(i) -= operator()(i, j) * b.row(j);
          }
        }
      }
      for (int j = N_ - 1; j >= 0; j--)
      {
        b.row(j) /= operator()(j, j);
        iM = std::max(0, j - upper_bw_);
        for (int i = iM; i <= j - 1; i++)
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
      for (int j = 0; j <= N_ - 1; j++)
      {
        b.row(j) /= operator()(j, j);
        iM = std::min(j + upper_bw_, N_ - 1);
        for (int i = j + 1; i <= iM; i++)
        {
          if (operator()(j, i) != 0.0)
          {
            b.row(i) -= operator()(j, i) * b.row(j);
          }
        }
      }
      for (int j = N_ - 1; j >= 0; j--)
      {
        iM = std::max(0, j - lower_bw_);
        for (int i = iM; i <= j - 1; i++)
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MINCOTrajectory() = default;

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
    T2_.resize(piece_num_);
    T3_.resize(piece_num_);
    T4_.resize(piece_num_);
    T5_.resize(piece_num_);
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
    const int expected_inner_points = std::max<int>(0, static_cast<int>(durations.size()) - 1);
    if (inner_points.cols() != expected_inner_points)
    {
      return false;
    }

    reset(head_state, tail_state, durations.size());
    durations_ = durations;
    T2_ = durations_.cwiseProduct(durations_);
    T3_ = T2_.cwiseProduct(durations_);
    T4_ = T2_.cwiseProduct(T2_);
    T5_ = T4_.cwiseProduct(durations_);

    system_.reset();
    coeffs_.setZero();

    system_(0, 0) = 1.0;
    system_(1, 1) = 1.0;
    system_(2, 2) = 2.0;
    coeffs_.row(0) = head_state_.col(0).transpose();
    coeffs_.row(1) = head_state_.col(1).transpose();
    coeffs_.row(2) = head_state_.col(2).transpose();

    for (int i = 0; i < piece_num_ - 1; ++i)
    {
      system_(6 * i + 3, 6 * i + 3) = 6.0;
      system_(6 * i + 3, 6 * i + 4) = 24.0 * durations_(i);
      system_(6 * i + 3, 6 * i + 5) = 60.0 * T2_(i);
      system_(6 * i + 3, 6 * i + 9) = -6.0;

      system_(6 * i + 4, 6 * i + 4) = 24.0;
      system_(6 * i + 4, 6 * i + 5) = 120.0 * durations_(i);
      system_(6 * i + 4, 6 * i + 10) = -24.0;

      system_(6 * i + 5, 6 * i) = 1.0;
      system_(6 * i + 5, 6 * i + 1) = durations_(i);
      system_(6 * i + 5, 6 * i + 2) = T2_(i);
      system_(6 * i + 5, 6 * i + 3) = T3_(i);
      system_(6 * i + 5, 6 * i + 4) = T4_(i);
      system_(6 * i + 5, 6 * i + 5) = T5_(i);

      system_(6 * i + 6, 6 * i) = 1.0;
      system_(6 * i + 6, 6 * i + 1) = durations_(i);
      system_(6 * i + 6, 6 * i + 2) = T2_(i);
      system_(6 * i + 6, 6 * i + 3) = T3_(i);
      system_(6 * i + 6, 6 * i + 4) = T4_(i);
      system_(6 * i + 6, 6 * i + 5) = T5_(i);
      system_(6 * i + 6, 6 * i + 6) = -1.0;

      system_(6 * i + 7, 6 * i + 1) = 1.0;
      system_(6 * i + 7, 6 * i + 2) = 2.0 * durations_(i);
      system_(6 * i + 7, 6 * i + 3) = 3.0 * T2_(i);
      system_(6 * i + 7, 6 * i + 4) = 4.0 * T3_(i);
      system_(6 * i + 7, 6 * i + 5) = 5.0 * T4_(i);
      system_(6 * i + 7, 6 * i + 7) = -1.0;

      system_(6 * i + 8, 6 * i + 2) = 2.0;
      system_(6 * i + 8, 6 * i + 3) = 6.0 * durations_(i);
      system_(6 * i + 8, 6 * i + 4) = 12.0 * T2_(i);
      system_(6 * i + 8, 6 * i + 5) = 20.0 * T3_(i);
      system_(6 * i + 8, 6 * i + 8) = -2.0;

      coeffs_.row(6 * i + 5) = inner_points.col(i).transpose();
    }

    const int last = piece_num_ - 1;
    system_(6 * piece_num_ - 3, 6 * piece_num_ - 6) = 1.0;
    system_(6 * piece_num_ - 3, 6 * piece_num_ - 5) = durations_(last);
    system_(6 * piece_num_ - 3, 6 * piece_num_ - 4) = T2_(last);
    system_(6 * piece_num_ - 3, 6 * piece_num_ - 3) = T3_(last);
    system_(6 * piece_num_ - 3, 6 * piece_num_ - 2) = T4_(last);
    system_(6 * piece_num_ - 3, 6 * piece_num_ - 1) = T5_(last);

    system_(6 * piece_num_ - 2, 6 * piece_num_ - 5) = 1.0;
    system_(6 * piece_num_ - 2, 6 * piece_num_ - 4) = 2.0 * durations_(last);
    system_(6 * piece_num_ - 2, 6 * piece_num_ - 3) = 3.0 * T2_(last);
    system_(6 * piece_num_ - 2, 6 * piece_num_ - 2) = 4.0 * T3_(last);
    system_(6 * piece_num_ - 2, 6 * piece_num_ - 1) = 5.0 * T4_(last);

    system_(6 * piece_num_ - 1, 6 * piece_num_ - 4) = 2.0;
    system_(6 * piece_num_ - 1, 6 * piece_num_ - 3) = 6.0 * durations_(last);
    system_(6 * piece_num_ - 1, 6 * piece_num_ - 2) = 12.0 * T2_(last);
    system_(6 * piece_num_ - 1, 6 * piece_num_ - 1) = 20.0 * T3_(last);

    coeffs_.row(6 * piece_num_ - 3) = tail_state_.col(0).transpose();
    coeffs_.row(6 * piece_num_ - 2) = tail_state_.col(1).transpose();
    coeffs_.row(6 * piece_num_ - 1) = tail_state_.col(2).transpose();

    system_.factorizeLU();
    system_.solve(coeffs_);
    return true;
  }

  int getPieceNum() const { return piece_num_; }
  const Eigen::VectorXd &getDurations() const { return durations_; }
  double getTotalDuration() const { return durations_.sum(); }
  const CoeffMat &getCoefficients() const { return coeffs_; }

  static void computeBasisFunctions(double t,
                                    BasisRow &b_p,
                                    BasisRow &b_v,
                                    BasisRow &b_a,
                                    BasisRow &b_j,
                                    BasisRow &b_s)
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;

    b_p << 1.0, t, t2, t3, t4, t5;
    b_v << 0.0, 1.0, 2.0 * t, 3.0 * t2, 4.0 * t3, 5.0 * t4;
    b_a << 0.0, 0.0, 2.0, 6.0 * t, 12.0 * t2, 20.0 * t3;
    b_j << 0.0, 0.0, 0.0, 6.0, 24.0 * t, 60.0 * t2;
    b_s << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * t;
  }

  VectorD evaluate(double t, int derivative_order = 0) const
  {
    if (piece_num_ <= 0)
    {
      return VectorD::Zero();
    }

    int piece_idx = 0;
    double local_t = t;
    if (t <= 0.0)
    {
      piece_idx = 0;
      local_t = 0.0;
    }
    else
    {
      double accum = 0.0;
      for (int i = 0; i < piece_num_; ++i)
      {
        const double next_accum = accum + durations_(i);
        if (t <= next_accum || i == piece_num_ - 1)
        {
          piece_idx = i;
          local_t = std::min(std::max(0.0, t - accum), durations_(i));
          break;
        }
        accum = next_accum;
      }
    }

    BasisRow b_p, b_v, b_a, b_j, b_s;
    computeBasisFunctions(local_t, b_p, b_v, b_a, b_j, b_s);

    const auto coeff_block = coeffs_.template block<COEFF_NUM, DIM>(piece_idx * COEFF_NUM, 0);
    VectorD out = VectorD::Zero();
    switch (derivative_order)
    {
    case 0:
      out.transpose().noalias() = b_p * coeff_block;
      break;
    case 1:
      out.transpose().noalias() = b_v * coeff_block;
      break;
    case 2:
      out.transpose().noalias() = b_a * coeff_block;
      break;
    case 3:
      out.transpose().noalias() = b_j * coeff_block;
      break;
    default:
      out.transpose().noalias() = b_s * coeff_block;
      break;
    }
    return out;
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
    return coeffs_.template block<COEFF_NUM, DIM>(piece_idx * COEFF_NUM, 0).transpose().rowwise().reverse();
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
        BasisRow b_p, b_v, b_a, b_j, b_s;
        computeBasisFunctions(t, b_p, b_v, b_a, b_j, b_s);
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
      energy += 36.0 * coeffs_.row(6 * i + 3).squaredNorm() * durations_(i) +
                144.0 * coeffs_.row(6 * i + 4).dot(coeffs_.row(6 * i + 3)) * T2_(i) +
                192.0 * coeffs_.row(6 * i + 4).squaredNorm() * T3_(i) +
                240.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 3)) * T3_(i) +
                720.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 4)) * T4_(i) +
                720.0 * coeffs_.row(6 * i + 5).squaredNorm() * T5_(i);
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
      gdC.row(6 * i + 5) = 240.0 * coeffs_.row(6 * i + 3) * T3_(i) +
                           720.0 * coeffs_.row(6 * i + 4) * T4_(i) +
                           1440.0 * coeffs_.row(6 * i + 5) * T5_(i);
      gdC.row(6 * i + 4) = 144.0 * coeffs_.row(6 * i + 3) * T2_(i) +
                           384.0 * coeffs_.row(6 * i + 4) * T3_(i) +
                           720.0 * coeffs_.row(6 * i + 5) * T4_(i);
      gdC.row(6 * i + 3) = 72.0 * coeffs_.row(6 * i + 3) * durations_(i) +
                           144.0 * coeffs_.row(6 * i + 4) * T2_(i) +
                           240.0 * coeffs_.row(6 * i + 5) * T3_(i);
    }
  }

  void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
  {
    gdT.resize(piece_num_);
    for (int i = 0; i < piece_num_; ++i)
    {
      gdT(i) = 36.0 * coeffs_.row(6 * i + 3).squaredNorm() +
               288.0 * coeffs_.row(6 * i + 4).dot(coeffs_.row(6 * i + 3)) * durations_(i) +
               576.0 * coeffs_.row(6 * i + 4).squaredNorm() * T2_(i) +
               720.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 3)) * T2_(i) +
               2880.0 * coeffs_.row(6 * i + 5).dot(coeffs_.row(6 * i + 4)) * T3_(i) +
               3600.0 * coeffs_.row(6 * i + 5).squaredNorm() * T4_(i);
    }
  }

  void propagateGrad(const CoeffMat &partial_grad_by_coeffs,
                     const Eigen::VectorXd &partial_grad_by_times,
                     InnerPointsMat &grad_by_points,
                     Eigen::VectorXd &grad_by_times) const
  {
    grad_by_points.resize(DIM, std::max(0, piece_num_ - 1));
    grad_by_points.setZero();
    grad_by_times.resize(piece_num_);
    grad_by_times.setZero();

    CoeffMat adj_grad = partial_grad_by_coeffs;
    system_.solveAdj(adj_grad);

    for (int i = 0; i < piece_num_ - 1; ++i)
    {
      grad_by_points.col(i) = adj_grad.row(6 * i + 5).transpose();
    }

    Eigen::Matrix<double, 6, DIM> B1;
    Eigen::Matrix<double, 3, DIM> B2;
    for (int i = 0; i < piece_num_ - 1; ++i)
    {
      B1.row(2) = -(coeffs_.row(6 * i + 1) +
                    2.0 * durations_(i) * coeffs_.row(6 * i + 2) +
                    3.0 * T2_(i) * coeffs_.row(6 * i + 3) +
                    4.0 * T3_(i) * coeffs_.row(6 * i + 4) +
                    5.0 * T4_(i) * coeffs_.row(6 * i + 5));
      B1.row(3) = B1.row(2);

      B1.row(4) = -(2.0 * coeffs_.row(6 * i + 2) +
                    6.0 * durations_(i) * coeffs_.row(6 * i + 3) +
                    12.0 * T2_(i) * coeffs_.row(6 * i + 4) +
                    20.0 * T3_(i) * coeffs_.row(6 * i + 5));

      B1.row(5) = -(6.0 * coeffs_.row(6 * i + 3) +
                    24.0 * durations_(i) * coeffs_.row(6 * i + 4) +
                    60.0 * T2_(i) * coeffs_.row(6 * i + 5));

      B1.row(0) = -(24.0 * coeffs_.row(6 * i + 4) +
                    120.0 * durations_(i) * coeffs_.row(6 * i + 5));

      B1.row(1) = -120.0 * coeffs_.row(6 * i + 5);

      grad_by_times(i) = B1.cwiseProduct(adj_grad.template block<6, DIM>(6 * i + 3, 0)).sum();
    }

    B2.row(0) = -(coeffs_.row(6 * piece_num_ - 5) +
                  2.0 * durations_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 4) +
                  3.0 * T2_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 3) +
                  4.0 * T3_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 2) +
                  5.0 * T4_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 1));

    B2.row(1) = -(2.0 * coeffs_.row(6 * piece_num_ - 4) +
                  6.0 * durations_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 3) +
                  12.0 * T2_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 2) +
                  20.0 * T3_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 1));

    B2.row(2) = -(6.0 * coeffs_.row(6 * piece_num_ - 3) +
                  24.0 * durations_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 2) +
                  60.0 * T2_(piece_num_ - 1) * coeffs_.row(6 * piece_num_ - 1));

    grad_by_times(piece_num_ - 1) = B2.cwiseProduct(adj_grad.template block<3, DIM>(6 * piece_num_ - 3, 0)).sum();
    grad_by_times += partial_grad_by_times;
  }

private:
  int piece_num_{0};
  BoundaryState head_state_{BoundaryState::Zero()};
  BoundaryState tail_state_{BoundaryState::Zero()};
  BandedSystem system_;
  CoeffMat coeffs_;
  Eigen::VectorXd durations_;
  Eigen::VectorXd T2_;
  Eigen::VectorXd T3_;
  Eigen::VectorXd T4_;
  Eigen::VectorXd T5_;
};

} // namespace minco

#endif
