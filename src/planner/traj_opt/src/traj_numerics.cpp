#include "optimizer/poly_traj_optimizer.h"

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
  //  Generate trajectory from states using NUBSTrajectory
  // =====================================================
  NUBSTraj PolyTrajOptimizer::generateTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                                 const Eigen::MatrixXd &innerPts, const Eigen::VectorXd &durations)
  {
    NUBSTraj traj(3);
    Eigen::MatrixXd P_full;
    traj.generate(innerPts,iniState,finState,durations,P_full);

    return traj;
  }

  Eigen::MatrixXd PolyTrajOptimizer::getInitConstraintPoints() const
  {
    return nubsOpt_.getTrajectory().getControlPoints().transpose();
  }

  bool PolyTrajOptimizer::computePointsToCheck(
      const NUBSTraj &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    pts_check.clear();
    int num_cps = traj.getControlPoints().rows();
    pts_check.resize(num_cps);
    
    const double RES = grid_map_->getResolution();
    const double RES_2 = RES / 2.0;

    double t_total = traj.getTotalDuration();
    
    // Adaptive stepping: Ensure step doesn't exceed grid resolution
    double max_v = std::max(max_vel_, 0.1); // Guard against zero max_vel
    double t_step = std::min(RES / max_v, t_total / num_cps / 2.0); 

    const Eigen::VectorXd& u = traj.getKnots();
    int p = traj.getP(); // Degree of B-spline

    Eigen::Vector3d pt_last = traj.evaluate(0.0);
    double t = 0.0;

    while (true)
    {
      if (t > t_total)
      {
        if (touch_goal_ && !pts_check.empty())
        {
          while (!pts_check.empty() && pts_check.back().empty())
            pts_check.pop_back();

          if (pts_check.empty())
          {
            ROS_ERROR("Failed to get points list to check (0x02).");
            return false;
          }
          else
            return true;
        }
        else
        {
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d", touch_goal_);
          pts_check.clear();
          return false;
        }
      }

      // Map time t to B-spline span
      int span = traj.findSpan(t, num_cps, u);
      
      // Map the span to its dominant center control point index
      int id_cps_curr = span - p / 2;
      id_cps_curr = std::max(0, std::min(id_cps_curr, num_cps - 1));

      if (id_cps_curr >= id_cps_end)
        break;

      Eigen::Vector3d pt = traj.evaluate(t);

      // Spatial downsampling
      if (t < 1e-5 || pts_check[id_cps_curr].empty() || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::make_pair(t, pt));
        pt_last = pt;
      }

      if (t >= t_total) break;
      t += t_step;
      if (t > t_total) t = t_total; 
    }

    return true;
  }

  // =====================================================
  //  LBFGS cost function callback
  // =====================================================
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    fill(opt->min_ellip_dist2_.begin(), opt->min_ellip_dist2_.end(), std::numeric_limits<double>::max());
    opt->cost_manager_.resetAccumulation();

    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<Eigen::VectorXd> grad_vec(grad, n);

    opt->iter_num_++;

    // 1. Time Mapping: Convert unbounded variables x to real segment durations d

    // 2. Define Time Penalty Closure
    //time_cost_.weight = opt->wei_time_;
    auto time_cost_wrapper = [&](const std::vector<double> &T_vec, Eigen::VectorXd &gdT) 
    {
      opt->cost_manager_.segment_dt_ = T_vec;

  
      gdT.setZero(T_vec.size());
      return time_cost_(T_vec, gdT);
    };

    // 3. Evaluate total cost and gradients via NUBS framework
    double total_cost = opt->nubs_opt_.evaluate(
        x_vec, 
        grad_vec, 
        time_cost_, 
        opt->cost_manager_
    );

    // 4. Update the visual/logic cps_.points directly from the NUBS trajectory
    opt->cps_.points = opt->nubs_opt_.getTrajectory().getControlPoints().transpose();

    // 5. Check for rebound
    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints();
    }

    return total_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

} // namespace ego_planner
