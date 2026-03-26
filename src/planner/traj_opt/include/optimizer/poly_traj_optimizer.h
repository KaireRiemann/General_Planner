#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include "optimizer/traj_types.h"
#include "CostFunctionalManager/EgoCostManager.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include <path_searching/dyn_a_star.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <memory>

namespace ego_planner
{
  using Types = cost_functional::PlanningTypesAdapter;
  // =====================================================
  //  PolyTrajOptimizer (using NUBSOptimizer)
  // =====================================================
  class PolyTrajOptimizer
  {

  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;

    // SplineOptimizer replaces MinJerkOpt
    SplineOpt splineOpt_;
    SplineOpt::Workspace spline_workspace_;

    //general cost functional manager
    cost_functional::LinearTimeCost time_cost_;
    cost_functional::EgoCostFunctionalManager cost_manager_;

    //NUBSOptimizer replaces MinJerkOpt
    NUBSOpt nubsOpt_;
    NUBSOpt::Workspace nubs_workspace_;

    Types::SwarmTrajData *swarm_trajs_{NULL};
    Types::ConstraintPoints cps_;

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

    using PtsChk_t = std::vector<std::vector<std::pair<double,Eigen::Vector3d>>>;

    //typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

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
    inline const NUBSOpt &getNUBSOpt(void) const { return nubsOpt_; }
    inline const NUBSTraj &getTraj(void) const { return nubsOpt_.getTrajectory(); }
    inline const SplineOpt &getSplineOpt(void) const { return splineOpt_; }
    inline const SplineTraj &getWorkingSpline(void) const { return splineOpt_.getWorkingSpline(spline_workspace_); }
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }

    // --- Numerical computation (implemented in traj_numerics.cpp) ---
    /**
     * @brief Generate a trajectory using NUBSOptimizer from init states.
     * Returns a NUBS<3> trajectory that can be used downstream.
     */
    NUBSTraj generateTrajectory(const Eigen::MatrixXd &initState, const Eigen::MatrixXd &finState,
                                const Eigen::MatrixXd &innerPts, const Eigen::VectorXd &durations);

    /**
     * @brief Get initial constraint points from the spline trajectory.
     * Directly use control points as initial constraint points there.
     */
    Eigen::MatrixXd getInitConstraintPoints(void) const;
    /**
     * @brief Perform dense sampling in the time domain.
     * map the sampled points to the B-spline control points that influence them the most
     */
    bool computePointsToCheck(const NUBSTraj &traj, int id_end, PtsChk_t &pts_check);

    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    // ====================================================================
    // --- Decision logic (implemented in poly_traj_optimizer.cpp) ---
    // ====================================================================

    /** @brief 优化主循环 (包含 L-BFGS 调用及 Rebound 逻辑) */
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);

    /** @brief 细致碰撞检测及 A* 引导点生成 */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const nubs::NUBSTrajectory<3> &traj,
                                              const Eigen::MatrixXd &init_points,
                                              const bool flag_first_init = true);

    bool roughlyCheckConstraintPoints(void);
    bool allowRebound(void);
    std::vector<Types::ConstraintPoints> distinctiveTrajs(std::vector<std::pair<int, int>> segments);S

  public:
    using Ptr = std::unique_ptr<PolyTrajOptimizer>;
    //typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif
