#include "optimizer/poly_traj_optimizer.h"
#include "TrajectoryOptAdapters/EgoVarianceSampleCostAdapter.hpp"

using namespace std;

namespace ego_planner
{
  SplineOpt::EvaluationResult PolyTrajOptimizer::evaluateCurrentDecisionVariables(
      const Eigen::VectorXd &x,
      Eigen::VectorXd &grad)
  {
    SplineOpt::EvaluateSpec<TimeCostFunction,
                            IntegralCostFunction,
                            SplineTrajectory::VoidWaypointsCost,
                            SampleCostFunction> spec;
    spec.time_cost = &time_cost_func_;
    spec.integral_cost = &integral_cost_func_;
    spec.sample_cost = &sample_cost_func_;
    spec.workspace = &spline_workspace_;
    return splineOpt_.evaluate(x, grad, spec);
  }

  // =====================================================
  //  Generate trajectory from states using QuinticSplineND
  // =====================================================
  PPoly3D PolyTrajOptimizer::generateTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &innerPts, const Eigen::VectorXd &durations)
  {
    int piece_num = durations.size();
    // Build waypoints: start + inner + end
    WaypointsMat waypoints(innerPts.cols() + 2, 3);
    waypoints.row(0) = iniState.col(0).transpose();
    for (int i = 0; i < innerPts.cols(); ++i)
      waypoints.row(i + 1) = innerPts.col(i).transpose();
    waypoints.row(innerPts.cols() + 1) = finState.col(0).transpose();

    // Build boundary conditions
    BCs bc;
    bc.start_velocity = iniState.col(1);
    bc.start_acceleration = iniState.col(2);
    bc.end_velocity = finState.col(1);
    bc.end_acceleration = finState.col(2);

    // Build time segments
    std::vector<double> time_segs(piece_num);
    for (int i = 0; i < piece_num; ++i)
      time_segs[i] = durations(i);

    // Create and update spline
    SplineTraj spline;
    spline.update(time_segs, waypoints, 0.0, bc);

    return spline.getTrajectoryCopy();
  }

  // =====================================================
  //  Get initial constraint points from a PPoly3D trajectory
  // =====================================================
  Eigen::MatrixXd PolyTrajOptimizer::getInitConstraintPoints(
      const PPoly3D &traj,
      const Eigen::VectorXd &durations,
      int K) const
  {
    int N = durations.size();
    int total_pts = N * K + 1;
    Eigen::MatrixXd cstr_pts(3, total_pts);
    int idx = 0;
    double t_accum = traj.getStartTime();

    for (int i = 0; i < N; ++i)
    {
      double dur = durations(i);
      double step = dur / K;
      for (int j = 0; j <= K; ++j)
      {
        double t = t_accum + step * j;
        cstr_pts.col(idx) = traj.evaluate(t, SplineTrajectory::Deriv::Pos);
        if (j != K || (j == K && i == N - 1))
          ++idx;
      }
      t_accum += dur;
    }

    return cstr_pts;
  }

  // =====================================================
  //  LBFGS cost function callback
  // =====================================================
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    fill(opt->min_ellip_dist2_.begin(), opt->min_ellip_dist2_.end(), std::numeric_limits<double>::max());

    Eigen::VectorXd x_vec = Eigen::Map<const Eigen::VectorXd>(x, n);
    Eigen::VectorXd grad_vec = Eigen::VectorXd::Zero(n);

    opt->integral_cost_func_.resetAccumulation();

    const auto eval_result = opt->evaluateCurrentDecisionVariables(x_vec, grad_vec);
    if (!eval_result)
    {
      ROS_ERROR_STREAM("SplineOptimizer evaluate failed: " << eval_result.message);
      opt->force_stop_type_ = STOP_FOR_ERROR;
      Eigen::Map<Eigen::VectorXd>(grad, n).setZero();
      return std::numeric_limits<double>::infinity();
    }

    const double total_cost = eval_result.cost;

    opt->updateConstraintPointsFromSamples();

    // Copy gradients back to raw pointer
    Eigen::Map<Eigen::VectorXd>(grad, n) = grad_vec;

    // Check for rebound
    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints();
    }

    opt->iter_num_ += 1;
    return total_cost;
  }

  void PolyTrajOptimizer::updateConstraintPointsFromSamples()
  {
    if (cps_.cp_size <= 0)
    {
      return;
    }

    if (cps_.points.rows() != 3 || cps_.points.cols() != cps_.cp_size)
    {
      cps_.points.resize(3, cps_.cp_size);
    }

    const auto &samples = splineOpt_.getRecordedIntegralSamples(spline_workspace_);
    for (const auto &sample : samples)
    {
      const int control_point_index = sample.seg_idx * cps_num_prePiece_ + sample.step_in_seg;
      if (control_point_index >= 0 && control_point_index < cps_.cp_size)
      {
        cps_.points.col(control_point_index) = sample.p;
      }
    }
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // =====================================================
  //  Dense point sampling for collision checking
  // =====================================================
  bool PolyTrajOptimizer::computePointsToCheck(
      const PPoly3D &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    pts_check.clear();
    pts_check.resize(id_cps_end);
    const double RES = grid_map_->getResolution(), RES_2 = RES / 2;

    // Build durations vector from PPoly3D segments
    int num_segs = traj.getNumSegments();
    Eigen::VectorXd durations(num_segs);
    double dur_sum = 0;
    for (int i = 0; i < num_segs; ++i)
    {
      durations(i) = (*(traj.begin() + i)).duration();
      dur_sum += durations(i);
    }

    Eigen::VectorXd t_seg_start(num_segs + 1);
    t_seg_start(0) = 0;
    for (int i = 0; i < num_segs; ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);

    const double DURATION = dur_sum;
    double t_step = min(RES / max_vel_, durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5);
    double start_t = traj.getStartTime();
    Eigen::Vector3d pt_last = traj.evaluate(start_t, SplineTrajectory::Deriv::Pos);
    int id_cps_curr = 0, id_piece_curr = 0;

    double t = 0.0;
    while (true)
    {
      if (t > DURATION)
      {
        if (touch_goal_ && pts_check.size() > 0)
        {
          while (pts_check.back().size() == 0)
            pts_check.pop_back();

          if (pts_check.size() <= 0)
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)pts_check.size());
            return false;
          }
          else
            return true;
        }
        else
        {
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d, pts_check.size()=%d", touch_goal_, (int)pts_check.size());
          pts_check.clear();
          return false;
        }
      }

      const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / cps_num_prePiece_ * ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
      if (t >= next_t_stp)
      {
        if (id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1))
          ++id_piece_curr;
        if (++id_cps_curr >= id_cps_end)
          break;
      }

      Eigen::Vector3d pt = traj.evaluate(start_t + t, SplineTrajectory::Deriv::Pos);
      if (t < 1e-5 || pts_check[id_cps_curr].size() == 0 || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        pt_last = pt;
      }

      t += t_step;
    }

    return true;
  }

} // namespace ego_planner
