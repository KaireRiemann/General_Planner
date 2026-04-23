#include "optimizer/poly_traj_optimizer.h"

using namespace std;

namespace ego_planner
{
  // =====================================================
  //  Generate trajectory from states using MINCO
  // =====================================================
  MINCOTraj PolyTrajOptimizer::generateTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                                  const Eigen::MatrixXd &innerPts, const Eigen::VectorXd &durations)
  {
    MINCOTraj traj;
    traj.generate(innerPts, iniState, finState, durations);
    return traj;
  }

  Eigen::MatrixXd PolyTrajOptimizer::getInitConstraintPoints() const
  {
    return getTrajectory().getInitConstraintPoints(cps_num_prePiece_);
  }

  const MINCOTraj &PolyTrajOptimizer::getTrajectory() const
  {
    switch (optimize_mode_)
    {
    case MODE_CORRIDOR:
      return corridorMincoOpt_.getTrajectory();
    case MODE_ESDF:
      return distanceFieldMincoOpt_.getTrajectory();
    case MODE_PLAIN:
    default:
      return mincoOpt_.getTrajectory();
    }
  }

  const SnapTraj &PolyTrajOptimizer::getSnapTrajectory() const
  {
    switch (optimize_mode_)
    {
    case MODE_CORRIDOR:
      return corridorSnapOpt_.getTrajectory();
    case MODE_ESDF:
      return distanceFieldSnapOpt_.getTrajectory();
    case MODE_PLAIN:
    default:
      return snapOpt_.getTrajectory();
    }
  }

  bool PolyTrajOptimizer::computePointsToCheck(
      const MINCOTraj &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    pts_check.clear();
    pts_check.resize(id_cps_end);

    const double RES = grid_map_->getResolution(), RES_2 = RES / 2.0;
    const Eigen::VectorXd durations = traj.getDurations();
    Eigen::VectorXd t_seg_start(durations.size() + 1);
    t_seg_start(0) = 0.0;
    for (int i = 0; i < durations.size(); ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);

    const double DURATION = durations.sum();
    const double t_step = std::min(RES / std::max(max_vel_, 0.1),
                                   durations.minCoeff() / std::max(cps_num_prePiece_, 1) / 1.5);

    Eigen::Vector3d pt_last = traj.evaluate(0.0, 0);
    double t = 0.0;
    int id_cps_curr = 0, id_piece_curr = 0;

    while (true)
    {
      if (t > DURATION)
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

      const double next_t_stp =
          t_seg_start(id_piece_curr) +
          durations(id_piece_curr) / cps_num_prePiece_ *
              ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
      if (t >= next_t_stp)
      {
        if (id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1))
        {
          ++id_piece_curr;
        }
        if (++id_cps_curr >= id_cps_end)
        {
          break;
        }
      }

      Eigen::Vector3d pt = traj.evaluate(t, 0);

      if (t < 1e-5 || pts_check[id_cps_curr].empty() ||
          (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::make_pair(t, pt));
        pt_last = pt;
      }

      t += t_step;
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
    if (opt->optimize_mode_ == MODE_CORRIDOR && opt->tracking_task_enabled_)
      opt->tracking_corridor_cost_manager_.resetAccumulation();
    else if (opt->optimize_mode_ == MODE_CORRIDOR && opt->perching_acceptance_active_)
      opt->perching_cost_manager_.resetAccumulation();
    else if (opt->optimize_mode_ == MODE_CORRIDOR)
      opt->corridor_cost_manager_.resetAccumulation();
    else if ((opt->optimize_mode_ == MODE_ESDF || opt->optimize_mode_ == MODE_PLAIN) &&
             opt->tracking_task_enabled_)
      opt->tracking_cost_manager_.resetAccumulation();
    else if ((opt->optimize_mode_ == MODE_ESDF || opt->optimize_mode_ == MODE_PLAIN) &&
             opt->perching_acceptance_active_)
      opt->perching_cost_manager_.resetAccumulation();
    else if (opt->optimize_mode_ == MODE_ESDF)
      opt->distance_field_cost_manager_.resetAccumulation();
    else
      opt->cost_manager_.resetAccumulation();

    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<Eigen::VectorXd> grad_vec(grad, n);

    opt->iter_num_++;
    opt->time_cost_.weight =
        (opt->perching_acceptance_active_
             ? std::max(opt->wei_time_, opt->wei_perching_time_)
             : opt->wei_time_);

    double total_cost = 0.0;
    if (opt->optimize_mode_ == MODE_CORRIDOR)
    {
      auto time_cost_wrapper = [&](const std::vector<double> &T_vec, Eigen::VectorXd &gdT) -> double
      {
        gdT.setZero(T_vec.size());
        return opt->time_cost_(T_vec, gdT);
      };

      if (opt->tracking_task_enabled_)
      {
        total_cost = (opt->terminal_mapping_ != nullptr && opt->terminal_mapping_->enabled())
                         ? opt->corridorMincoOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->tracking_corridor_cost_manager_,
                               opt->terminal_mapping_)
                         : opt->corridorMincoOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->tracking_corridor_cost_manager_);
      }
      else if (opt->perching_acceptance_active_)
      {
        total_cost = (opt->snap_terminal_mapping_ != nullptr && opt->snap_terminal_mapping_->enabled())
                         ? opt->corridorSnapOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->perching_cost_manager_,
                               opt->snap_terminal_mapping_)
                         : opt->corridorSnapOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->perching_cost_manager_);
      }
      else
      {
        total_cost = (opt->terminal_mapping_ != nullptr && opt->terminal_mapping_->enabled())
                         ? opt->corridorMincoOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->corridor_cost_manager_,
                               opt->terminal_mapping_)
                         : opt->corridorMincoOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->corridor_cost_manager_);
      }
    }
    else if (opt->optimize_mode_ == MODE_ESDF)
    {
      auto time_cost_wrapper = [&](const std::vector<double> &T_vec, Eigen::VectorXd &gdT) -> double
      {
        gdT.setZero(T_vec.size());
        return opt->time_cost_(T_vec, gdT);
      };

      if (opt->tracking_task_enabled_)
      {
        total_cost = (opt->terminal_mapping_ != nullptr && opt->terminal_mapping_->enabled())
                         ? opt->distanceFieldMincoOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->tracking_cost_manager_,
                               opt->terminal_mapping_)
                         : opt->distanceFieldMincoOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->tracking_cost_manager_);
      }
      else if (opt->perching_acceptance_active_)
      {
        total_cost = (opt->snap_terminal_mapping_ != nullptr && opt->snap_terminal_mapping_->enabled())
                         ? opt->distanceFieldSnapOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->perching_cost_manager_,
                               opt->snap_terminal_mapping_)
                         : opt->distanceFieldSnapOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->perching_cost_manager_);
      }
      else
      {
        total_cost = (opt->terminal_mapping_ != nullptr && opt->terminal_mapping_->enabled())
                         ? opt->distanceFieldMincoOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->distance_field_cost_manager_,
                               opt->terminal_mapping_)
                         : opt->distanceFieldMincoOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->distance_field_cost_manager_);
      }

      const Eigen::MatrixXd constraint_points =
          (opt->perching_acceptance_active_
               ? opt->distanceFieldSnapOpt_.getTrajectory().getInitConstraintPoints(opt->cps_num_prePiece_)
               : opt->distanceFieldMincoOpt_.getTrajectory().getInitConstraintPoints(opt->cps_num_prePiece_));
      opt->syncConstraintPointStorage(constraint_points);
    }
    else
    {
      auto time_cost_wrapper = [&](const std::vector<double> &T_vec, Eigen::VectorXd &gdT) -> double
      {
        opt->cost_manager_.segment_dt_ = T_vec;
        gdT.setZero(T_vec.size());
        return opt->time_cost_(T_vec, gdT);
      };

      if (opt->tracking_task_enabled_)
      {
        total_cost = (opt->terminal_mapping_ != nullptr && opt->terminal_mapping_->enabled())
                         ? opt->mincoOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->tracking_cost_manager_,
                               opt->terminal_mapping_)
                         : opt->mincoOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->tracking_cost_manager_);
      }
      else if (opt->perching_acceptance_active_)
      {
        total_cost = (opt->snap_terminal_mapping_ != nullptr && opt->snap_terminal_mapping_->enabled())
                         ? opt->snapOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->perching_cost_manager_,
                               opt->snap_terminal_mapping_)
                         : opt->snapOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->perching_cost_manager_);
      }
      else
      {
        total_cost = (opt->terminal_mapping_ != nullptr && opt->terminal_mapping_->enabled())
                         ? opt->mincoOpt_.evaluateWithTerminalMapping(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->cost_manager_,
                               opt->terminal_mapping_)
                         : opt->mincoOpt_.evaluate(
                               x_vec,
                               grad_vec,
                               time_cost_wrapper,
                               opt->cost_manager_);
      }

      const Eigen::MatrixXd constraint_points =
          (opt->perching_acceptance_active_
               ? opt->snapOpt_.getTrajectory().getInitConstraintPoints(opt->cps_num_prePiece_)
               : opt->mincoOpt_.getTrajectory().getInitConstraintPoints(opt->cps_num_prePiece_));
      opt->syncConstraintPointStorage(constraint_points);

      if (opt->allowRebound())
      {
        opt->roughlyCheckConstraintPoints();
      }
    }

    return total_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

} // namespace ego_planner
