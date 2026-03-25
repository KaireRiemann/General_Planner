#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include "optimizer/traj_types.h"
#include "optimizer/cost_functions.h"
#include <path_searching/dyn_a_star.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"

namespace ego_planner
{
  // =====================================================
  //  PolyTrajOptimizer (using SplineOptimizer)
  // =====================================================
  class PolyTrajOptimizer
  {

  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;

    // SplineOptimizer replaces MinJerkOpt
    SplineOpt splineOpt_;
    SplineOpt::Workspace spline_workspace_;

    SwarmTrajData *swarm_trajs_{NULL};
    ConstraintPoints cps_;

    int drone_id_;
    int cps_num_prePiece_;
    int variable_num_;
    int piece_num_;
    int iter_num_;
    std::vector<double> min_ellip_dist2_;
    bool touch_goal_;
    struct MultitopologyData_t
    {
      bool use_multitopology_trajs{false};
      bool initial_obstacles_avoided{false};
    } multitopology_data_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;
    double wei_swarm_, wei_swarm_mod_;
    double wei_feas_;
    double wei_sqrvar_;
    double wei_time_;
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_;
    double max_vel_, max_acc_, max_jer_;
    double rho_energy_{1.0};

    double t_now_;

    // Cost function instances
    TimeCostFunction time_cost_func_;
    IntegralCostFunction integral_cost_func_;
    SampleCostFunction sample_cost_func_;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);
    void setConstraintPoints(ConstraintPoints cps);
    void setUseMultitopologyTrajs(bool use_multitopology_trajs);

    /* helper functions */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    inline const SplineOpt &getSplineOpt(void) const { return splineOpt_; }
    inline const SplineTraj &getWorkingSpline(void) const { return splineOpt_.getWorkingSpline(spline_workspace_); }
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }

    // --- Numerical computation (implemented in traj_numerics.cpp) ---

    /**
     * @brief Generate a trajectory using SplineOptimizer from init states.
     * Returns a PPoly3D trajectory that can be used downstream.
     */
    PPoly3D generateTrajectory(
        const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
        const Eigen::MatrixXd &innerPts, const Eigen::VectorXd &durations);

    /**
     * @brief Get initial constraint points from the spline trajectory.
     */
    Eigen::MatrixXd getInitConstraintPoints(const PPoly3D &traj,
                                             const Eigen::VectorXd &durations,
                                             int K) const;

    /**
     * @brief Compute dense sample points for collision checking.
     */
    bool computePointsToCheck(const PPoly3D &traj, int id_end, PtsChk_t &pts_check);

    // --- Decision logic (implemented in poly_traj_optimizer.cpp) ---

    /** @brief Main optimization loop with retry/rebound control. */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);

    /** @brief Fine collision check: segments obstacles, A-star replanning. */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const PPoly3D &traj,
                                              const Eigen::MatrixXd &init_points,
                                              const bool flag_first_init /*= true*/);

    /** @brief In-optimization rebound detection. */
    bool roughlyCheckConstraintPoints(void);

    /** @brief Rebound gate: checks trajectory smoothness and multi-topo readiness. */
    bool allowRebound(void);

    /** @brief Generate alternative topologies around obstacles. */
    std::vector<ConstraintPoints> distinctiveTrajs(std::vector<std::pair<int, int>> segments);

  private:
    SplineOpt::EvaluationResult evaluateCurrentDecisionVariables(const Eigen::VectorXd &x,
                                                                Eigen::VectorXd &grad);
    void updateConstraintPointsFromSamples();

    /* callbacks by the L-BFGS optimizer (in traj_numerics.cpp) */
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

  public:
    typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif
