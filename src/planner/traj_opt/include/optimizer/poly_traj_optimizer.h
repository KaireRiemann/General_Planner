#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include "plan_env/grid_map.h"
#include "optimizer/traj_types.h"
#include "CostFunctionalManager/EgoCostManager.hpp"
#include "CostFunctionalManager/PerchingCostManager.hpp"
#include "CostFunctionalManager/CorridorCostManager.hpp"
#include "CostFunctionalManager/DistanceFieldCostManager.hpp"
#include "CostFunctionalManager/TrackingCostManager.hpp"
#include "CostFunctionalManager/TrackingCorridorCostManager.hpp"
#include "CostFunctionalManager/TrackingTypes.hpp"
#include "CostFunctionalManager/TrackingSemanticGuide.hpp"
#include "CostFunctionalManager/CostFunctional/TemporalCosts/LinearTimeCost.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"
#include "SpatialMap/SFCCommonTypes.hpp"
#include <path_searching/dyn_a_star.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <memory>
#include <limits>

namespace ego_planner
{
  using Types = cost_functional::PlanningTypesAdapter;
  // =====================================================
  //  PolyTrajOptimizer (using generic MINCOOptimizer)
  // =====================================================
  class PolyTrajOptimizer
  {
  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;

    //general cost functional manager
    cost_functional::LinearTimeCost time_cost_;
    cost_functional::EgoCostFunctionalManager cost_manager_;
    cost_functional::PerchingCostFunctionalManager perching_cost_manager_;
    cost_functional::CorridorCostFunctionalManager corridor_cost_manager_;
    cost_functional::DistanceFieldCostFunctionalManager distance_field_cost_manager_;
    cost_functional::TrackingCostFunctionalManager tracking_cost_manager_;
    cost_functional::TrackingCorridorCostFunctionalManager tracking_corridor_cost_manager_;

    MINCOOpt mincoOpt_;
    ESDFMINCOOpt distanceFieldMincoOpt_;
    CorridorMINCOOpt corridorMincoOpt_;
    spatial_map::PolytopeSpatialMap corridorSpatialMap_;
    spatial_map::PolyhedraV corridor_vpolys_;
    Eigen::VectorXi corridor_vpoly_idx_;
    Eigen::VectorXi corridor_hpoly_idx_;

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
    double wei_perching_time_;
    double wei_corridor_;
    double wei_corridor_ref_;
    double wei_dist_;
    double wei_tracking_near_;
    double wei_tracking_far_;
    double wei_tracking_vertical_;
    double wei_tracking_view_xy_;
    double wei_tracking_view_z_;
    double wei_tracking_terminal_pos_;
    double wei_tracking_terminal_vel_;
    double wei_tracking_los_;
    double wei_tracking_visible_fan_;
    double wei_tracking_view_dir_smooth_;
    double tracking_los_clearance_;
    double tracking_distance_min_;
    double tracking_distance_max_;
    double tracking_height_tolerance_;
    double tracking_smooth_eps_;
    double wei_perching_floor_;
    double wei_perching_thrust_;
    double wei_perching_omega_;
    double wei_perching_collision_;
    double safety_margin_;
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_;
    double corridor_clearance_, corridor_smoothing_;
    double max_vel_, max_acc_, max_jer_;
    double perching_floor_height_;
    double perching_thrust_min_;
    double perching_thrust_max_;
    double perching_omega_max_;
    double perching_robot_radius_;
    double perching_platform_radius_;
    double rho_energy_{1.0};
    enum OptimizeMode
    {
      MODE_PLAIN = 0,
      MODE_ESDF,
      MODE_CORRIDOR
    } optimize_mode_{MODE_PLAIN};

    double t_now_;
    bool tracking_task_enabled_{false};
    bool tracking_semantic_enabled_{false};
    const minco::TerminalMappingBase<TRAJ_DIM, MINCO_S> *terminal_mapping_{nullptr};
    cost_functional::TrackingReference tracking_reference_;
    cost_functional::TrackingSemanticGuide tracking_semantic_guide_;

    using PtsChk_t = std::vector<std::vector<std::pair<double,Eigen::Vector3d>>>;
    void resetSpatialOptimizationContext();
    void syncConstraintPointStorage(const Eigen::MatrixXd &constraint_points);

    //typedef std::vector<std::vector<std::pair<double, Eigen::Vector3d>>> PtsChk_t;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    struct PerchingCheckConfig
    {
      bool enabled{false};
      double terminal_relax_time{0.35};
      double contact_position_tolerance{0.18};
      double relative_tangential_speed_tolerance{0.45};
      double relative_normal_speed_tolerance{0.80};
    };

    struct PerchingTerminalMetrics
    {
      bool valid{false};
      double total_duration{0.0};
      double approach_check_until{0.0};
      Eigen::Vector3d expected_contact_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d expected_contact_velocity{Eigen::Vector3d::Zero()};
      Eigen::Vector3d expected_plate_velocity{Eigen::Vector3d::Zero()};
      Eigen::Vector3d surface_normal{Eigen::Vector3d::UnitZ()};
      Eigen::Vector3d final_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d final_velocity{Eigen::Vector3d::Zero()};
      double contact_position_error{std::numeric_limits<double>::infinity()};
      double relative_tangential_speed{std::numeric_limits<double>::infinity()};
      double relative_normal_speed{std::numeric_limits<double>::infinity()};
      double signed_relative_normal_speed{0.0};
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
    void setTerminalMapping(const minco::TerminalMappingBase<TRAJ_DIM, MINCO_S> *terminal_mapping);

    /* helper functions */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    inline const MINCOOpt &getMINCOOpt(void) const { return mincoOpt_; }
    const MINCOTraj &getTrajectory(void) const;
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }

    // --- Numerical computation (implemented in traj_numerics.cpp) ---
    MINCOTraj generateTrajectory(const Eigen::MatrixXd &initState, const Eigen::MatrixXd &finState,
                                 const Eigen::MatrixXd &innerPts, const Eigen::VectorXd &durations);

    /**
     * @brief Sample initial deformation points from the MINCO trajectory.
     */
    Eigen::MatrixXd getInitConstraintPoints(void) const;
    /**
     * @brief Perform dense sampling in the time domain and group points by guide sample index.
     */
    bool computePointsToCheck(const MINCOTraj &traj, int id_end, PtsChk_t &pts_check);

    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    // ====================================================================
    // --- Decision logic (implemented in poly_traj_optimizer.cpp) ---
    // ====================================================================
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            double &final_cost);
    bool optimizePerchingTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                    const minco::TerminalMappingBase<TRAJ_DIM, MINCO_S> &terminal_mapping,
                                    double &final_cost);
    bool optimizeTrackingTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                    const cost_functional::TrackingReference &tracking_ref,
                                    const cost_functional::TrackingSemanticGuide *tracking_semantic_guide,
                                    double &final_cost);

    bool optimizeTrajectoryWithDistanceField(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                             const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                             double &final_cost);
    bool optimizePerchingTrajectoryWithDistanceField(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                                     const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                                     const minco::TerminalMappingBase<TRAJ_DIM, MINCO_S> &terminal_mapping,
                                                     double &final_cost);
    bool optimizeTrackingTrajectoryWithDistanceField(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                                     const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                                     const cost_functional::TrackingReference &tracking_ref,
                                                     const cost_functional::TrackingSemanticGuide *tracking_semantic_guide,
                                                     double &final_cost);

    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            const spatial_map::PolyhedraH &corridor_hpolys,
                            const Eigen::VectorXi *corridor_piece_idx,
                            double &final_cost);
    bool optimizePerchingTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                    const spatial_map::PolyhedraH &corridor_hpolys,
                                    const Eigen::VectorXi *corridor_piece_idx,
                                    const minco::TerminalMappingBase<TRAJ_DIM, MINCO_S> &terminal_mapping,
                                    double &final_cost);
    bool optimizeTrackingTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                    const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                    const spatial_map::PolyhedraH &corridor_hpolys,
                                    const Eigen::VectorXi *corridor_piece_idx,
                                    const cost_functional::TrackingReference &tracking_ref,
                                    double &final_cost);
    bool optimizeTrackingTrajectoryWithVisibleRegions(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                                      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                                      const spatial_map::PolyhedraH &corridor_hpolys,
                                                      const Eigen::VectorXi *corridor_piece_idx,
                                                      const cost_functional::TrackingReference &tracking_ref,
                                                      const cost_functional::TrackingSemanticGuide &tracking_semantic_guide,
                                                      double &final_cost);

    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const MINCOTraj &traj,
                                              const Eigen::MatrixXd &init_points,
                                              const bool flag_first_init = true);

    bool roughlyCheckConstraintPoints(void);
    bool allowRebound(void);
    std::vector<Types::ConstraintPoints> distinctiveTrajs(std::vector<std::pair<int, int>> segments);
    bool isTrajectoryCollisionFree(const MINCOTraj &traj) const;
    bool isTrajectoryCollisionFreeUntil(const MINCOTraj &traj,
                                        double until_time) const;


    bool isTrajectoryInsideCorridor(const MINCOTraj &traj,
                                const spatial_map::PolyhedraH &corridor_hpolys,
                                double margin) const;
    bool isTrajectoryInsideCorridorUntil(const MINCOTraj &traj,
                                         const spatial_map::PolyhedraH &corridor_hpolys,
                                         double margin,
                                         double until_time) const;

    bool getLastPerchingExtraVariables(Eigen::VectorXd &extra_vars) const
    {
      if (!has_last_perching_extra_vars_)
      {
        extra_vars.resize(0);
        return false;
      }

      extra_vars = last_perching_extra_vars_;
      return extra_vars.size() > 0;
    }

    void clearLastPerchingExtraVariables()
    {
      last_perching_extra_vars_.resize(0);
      has_last_perching_extra_vars_ = false;
    }

    bool evaluatePerchingTerminalMetrics(
        const MINCOTraj &traj,
        const Eigen::MatrixXd &iniState,
        const Eigen::MatrixXd &nominalTailState,
        const minco::TerminalMappingBase<TRAJ_DIM, MINCO_S> &terminal_mapping,
        const Eigen::Ref<const Eigen::VectorXd> &extra_vars,
        PerchingTerminalMetrics &metrics) const;

    bool pointInsidePolytope(const Eigen::Vector3d &pt,
                            const spatial_map::PolyhedronH &hpoly,
                            double margin) const;

    bool pointInsideCorridor(const Eigen::Vector3d &pt,
                            const spatial_map::PolyhedraH &corridor_hpolys,
                            double margin) const;

  public:
    using Ptr = std::unique_ptr<PolyTrajOptimizer>;
    //typedef std::unique_ptr<PolyTrajOptimizer> Ptr;
  private:
    PerchingCheckConfig perching_check_config_;
    bool perching_acceptance_active_{false};
    bool has_last_perching_extra_vars_{false};
    Eigen::VectorXd last_perching_extra_vars_;
  };

} // namespace ego_planner
#endif
