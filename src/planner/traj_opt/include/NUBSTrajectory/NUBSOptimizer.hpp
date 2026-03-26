#ifndef NUBS_OPTIMIZER_HPP
#define NUBS_OPTIMIZER_HPP

#include "NUBSTrajectory.hpp" 
#include "optimizer/lbfgs.hpp"  
#include "SpatialMap/IdentityMap.hpp"
#include "SpatialMap/PolytopeSpatialMap.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <memory>
#include <type_traits>
#include <iostream>

namespace nubs
{
    // =========================================================================
    //  INTERFACE DOCUMENTATION / TypeTraits 
    // =========================================================================
    namespace TypeTraits
    {
        template <typename...>
        using void_t = void;

        // --- TimeMap Traits ---
        template <typename T, typename = void>
        struct HasTimeMapInterface : std::false_type {};

        template <typename T>
        struct HasTimeMapInterface<T, void_t<
            decltype(static_cast<double>(std::declval<T>().toTime(std::declval<double>()))),
            decltype(static_cast<double>(std::declval<T>().toTau(std::declval<double>()))),
            decltype(static_cast<double>(std::declval<T>().backward(std::declval<double>(), std::declval<double>(), std::declval<double>())))
        >> : std::true_type {};

        // --- SpatialMap Traits ---
        template <typename T, int DIM, typename = void>
        struct HasSpatialMapInterface : std::false_type {};

        template <typename T, int DIM>
        struct HasSpatialMapInterface<T, DIM, void_t<
            decltype(static_cast<int>(std::declval<T>().getUnconstrainedDim(std::declval<int>()))),
            decltype(std::declval<T>().toPhysical(std::declval<Eigen::VectorXd>(), std::declval<int>())),
            decltype(std::declval<T>().toUnconstrained(std::declval<Eigen::Matrix<double, DIM, 1>>(), std::declval<int>())),
            decltype(std::declval<T>().backwardGrad(std::declval<Eigen::VectorXd>(),
                                                    std::declval<Eigen::Matrix<double, DIM, 1>>(),
                                                    std::declval<int>())),
            decltype(std::declval<T>().addNormPenalty(std::declval<Eigen::VectorXd>(), std::declval<double&>(), std::declval<Eigen::VectorXd&>()))
        >> : std::true_type {};

        // --- TimeCost Func Traits ---
        template <typename T, typename = void>
        struct HasTimeCostInterface : std::false_type {};

        template <typename T>
        struct HasTimeCostInterface<T, void_t<
            decltype(static_cast<double>(std::declval<T>()(
                std::declval<const std::vector<double>&>(),
                std::declval<Eigen::VectorXd &>()
            )))
        >> : std::true_type {};
    }

    // =========================================================================
    //  Time Mapping Components
    // =========================================================================
    struct IdentityTimeMap
    {
        double toTime(double tau) const { return tau; }
        double toTau(double T) const { return T; }
        double backward(double tau, double T, double gradT) const { return gradT; }
    };

    struct QuadInvTimeMap
    {
        double toTime(double tau) const {
            return tau > 0 ? ((0.5 * tau + 1.0) * tau + 1.0) : (1.0 / ((0.5 * tau - 1.0) * tau + 1.0));
        }
        double toTau(double T) const {
            return T > 1.0 ? (std::sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - std::sqrt(2.0 / T - 1.0));
        }
        double backward(double tau, double T, double gradT) const {
            if (tau > 0) return gradT * (tau + 1.0);
            double den = (0.5 * tau - 1.0) * tau + 1.0;
            return gradT * (1.0 - tau) / (den * den);
        }
    };

    // =========================================================================
    //  NUBS Control Point Evaluator 
    // =========================================================================
    template <int DIM>
    class NUBSControlPointEvaluator
    {
    public:
        /**
         * @brief 计算给定控制点 C 和节点 u 下的代价，并求取关于控制点 C 的解析梯度
         * @param C              当前轨迹的控制点 (N_c x DIM)
         * @param u              节点向量 (knots)
         * @param p              B样条阶数 (degree)
         * @param user_cost_func 用户传入的代价评估函数 
         * @param gdC            [输出] 链式传导到控制点 C 的梯度
         * @return double        用户评价代价 Cost
         */
        template <typename UserCostFunc>
        static double evaluateCostAndGradC(const Eigen::MatrixXd& C, 
                                           const Eigen::VectorXd& u, 
                                           int p, 
                                           UserCostFunc&& user_cost_func,
                                           Eigen::MatrixXd& gdC)
        {
            double cost = 0.0;
            const int n = C.rows();
            gdC.setZero(n, DIM);

            if (n < 2) return cost;

            Eigen::MatrixXd V(std::max(0, n - 1), DIM); Eigen::MatrixXd gdV = Eigen::MatrixXd::Zero(std::max(0, n - 1), DIM);
            Eigen::MatrixXd A(std::max(0, n - 2), DIM); Eigen::MatrixXd gdA = Eigen::MatrixXd::Zero(std::max(0, n - 2), DIM);
            Eigen::MatrixXd J(std::max(0, n - 3), DIM); Eigen::MatrixXd gdJ = Eigen::MatrixXd::Zero(std::max(0, n - 3), DIM);

            std::vector<double> dt_v(std::max(0, n - 1)), dt_a(std::max(0, n - 2)), dt_j(std::max(0, n - 3));

            // Velocity (C -> V)
            for (int i = 0; i < n - 1; ++i) {
                dt_v[i] = u(i + p + 1) - u(i + 1);
                if (dt_v[i] > 1e-9) {
                    V.row(i) = (p / dt_v[i]) * (C.row(i + 1) - C.row(i));
                } else {
                    V.row(i).setZero();
                }
            }
            // Acceleration (V -> A)
            for (int i = 0; i < n - 2; ++i) {
                dt_a[i] = u(i + p + 1) - u(i + 2);
                if (dt_a[i] > 1e-9) {
                    A.row(i) = ((p - 1) / dt_a[i]) * (V.row(i + 1) - V.row(i));
                } else {
                    A.row(i).setZero();
                }
            }
            // Jerk (A -> J)
            for (int i = 0; i < n - 3; ++i) {
                dt_j[i] = u(i + p + 1) - u(i + 3);
                if (dt_j[i] > 1e-9) {
                    J.row(i) = ((p - 2) / dt_j[i]) * (A.row(i + 1) - A.row(i));
                } else {
                    J.row(i).setZero();
                }
            }

            //user_cost_func(V, A, J, cost, gdV, gdA, gdJ);
            user_cost_func(C, V, A, J, cost, gdC, gdV, gdA, gdJ);


            // gdJ -> gdA
            for (int i = 0; i < n - 3; ++i) {
                if (dt_j[i] > 1e-9) {
                    double factor = (p - 2) / dt_j[i];
                    gdA.row(i + 1) += factor * gdJ.row(i);
                    gdA.row(i)     -= factor * gdJ.row(i);
                }
            }
            // gdA -> gdV
            for (int i = 0; i < n - 2; ++i) {
                if (dt_a[i] > 1e-9) {
                    double factor = (p - 1) / dt_a[i];
                    gdV.row(i + 1) += factor * gdA.row(i);
                    gdV.row(i)     -= factor * gdA.row(i);
                }
            }
            // gdV -> gdC
            for (int i = 0; i < n - 1; ++i) {
                if (dt_v[i] > 1e-9) {
                    double factor = p / dt_v[i];
                    gdC.row(i + 1) += factor * gdV.row(i);
                    gdC.row(i)     -= factor * gdV.row(i);
                }
            }

            return cost;
        }

        /**
         * @brief 将对时间 T 的差分评估封装在一起
         */
        template <typename UserCostFunc>
        static void evaluateTotal(const Eigen::MatrixXd& C, 
                                  const Eigen::VectorXd& knots, 
                                  const Eigen::VectorXd& T,
                                  int p, 
                                  int nc, 
                                  const NUBSTrajectory<DIM>& traj_template,
                                  UserCostFunc&& user_cost_func,
                                  double& cost, 
                                  Eigen::MatrixXd& gdC, 
                                  Eigen::VectorXd& gdT)
        {
            cost = evaluateCostAndGradC(C, knots, p, std::forward<UserCostFunc>(user_cost_func), gdC);
    
            double eps = 1e-6;
            gdT.resize(T.size());
            Eigen::MatrixXd dummy_gdC; 

            for (int i = 0; i < T.size(); ++i)
            {
                Eigen::VectorXd T_p = T, T_m = T;
                T_p(i) += eps;
                T_m(i) -= eps;

                Eigen::VectorXd u_plus = traj_template.generateKnots(T_p, nc);
                double cost_p = evaluateCostAndGradC(C, u_plus, p, user_cost_func, dummy_gdC);

                Eigen::VectorXd u_minus = traj_template.generateKnots(T_m, nc);
                double cost_m = evaluateCostAndGradC(C, u_minus, p, user_cost_func, dummy_gdC);

                gdT(i) = (cost_p - cost_m) / (2.0 * eps);
            }
        }
    };

    // =========================================================================
    //  通用 NUBS 求解器引擎 NUBSOptimizer
    // =========================================================================
    template <int DIM, int MAX_P = 7,
              typename TimeMap = nubs::QuadInvTimeMap,           
              typename SpatialMap = spatial_map::PolytopeSpatialMap> 
    class NUBSOptimizer
    {
        static_assert(TypeTraits::HasTimeMapInterface<TimeMap>::value, "TimeMap does not satisfy requirements.");
        static_assert(TypeTraits::HasSpatialMapInterface<SpatialMap, DIM>::value, "SpatialMap does not satisfy requirements.");

    public:
        using TrajType = NUBSTrajectory<DIM, MAX_P>;
        using MatrixType = Eigen::Matrix<double, Eigen::Dynamic, DIM>;

        struct Workspace
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Eigen::VectorXd cache_T;
            MatrixType cache_P_inner;
            Eigen::MatrixXd headState;
            Eigen::MatrixXd tailState;
            
            Eigen::MatrixXd P_full; 
            
            Eigen::MatrixXd gdC_energy;
            Eigen::VectorXd gdT_energy;
            
            Eigen::MatrixXd gdC_user;
            Eigen::VectorXd gdT_user;
            Eigen::VectorXd gdT_time;

            Eigen::MatrixXd gradByPoints;
            Eigen::VectorXd gradByTimes;
        };

    private:
        TrajType traj_;
        int num_segments_ = 0;
        int s_ = 0;
        
        std::vector<double> ref_times_;
        MatrixType ref_waypoints_;
        
        TimeMap default_time_map_;
        SpatialMap default_spatial_map_;
        
        const TimeMap* active_time_map_ = nullptr;
        const SpatialMap* active_spatial_map_ = nullptr;
        
        mutable std::unique_ptr<Workspace> ws_;
        double rho_energy_ = 1.0; 

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        NUBSOptimizer(int sys_order = 3) : traj_(sys_order), s_(sys_order) 
        {
            ws_ = std::make_unique<Workspace>();
        }

        void setEnergyWeights(double rho) { rho_energy_ = rho; }
        void setTimeMap(const TimeMap* tm) 
        { 
            active_time_map_ = (tm != nullptr) ? tm : &default_time_map_; 
        
        }
        void setSpatialMap(const SpatialMap* sm) 
        { 
            active_spatial_map_ = (sm != nullptr) ? sm : &default_spatial_map_;
        }

        /**
         * @brief 初始化参考状态 (航路点, 时间段, 以及首尾状态如 P,V,A)
         */
        bool setInitState(const std::vector<double> &time_segments,
                          const MatrixType &waypoints,
                          const Eigen::MatrixXd &boundary_head, 
                          const Eigen::MatrixXd &boundary_tail) 
        {
            num_segments_ = time_segments.size();
            ref_times_ = time_segments;
            ref_waypoints_ = waypoints;

            ws_->cache_T.resize(num_segments_);
            if (num_segments_ > 1) {
                ws_->cache_P_inner.resize(num_segments_ - 1, DIM);
            }
            ws_->headState = boundary_head;
            ws_->tailState = boundary_tail;

            int nc = traj_.getCtrlPtNum(num_segments_);
            ws_->gdC_energy.resize(nc, DIM);
            ws_->gdT_energy.resize(num_segments_);
            ws_->gdC_user.resize(nc, DIM);
            ws_->gdT_user.resize(num_segments_);
            ws_->gdT_time.resize(num_segments_);

            return true;
        }

        /**
         * @brief 提取当前状态并将其映射到无约束参数空间
         */
        Eigen::VectorXd generateInitialGuess() const
        {
            int dim_T = num_segments_;
            int dim_P = 0;
            for (int i = 1; i < num_segments_; ++i) {
                dim_P += active_spatial_map_->getUnconstrainedDim(i);
            }

            Eigen::VectorXd x(dim_T + dim_P);

            for (int i = 0; i < num_segments_; ++i) {
                x(i) = active_time_map_->toTau(ref_times_[i]);
            }

            int offset = dim_T;
            for (int i = 1; i < num_segments_; ++i) {
                int dof = active_spatial_map_->getUnconstrainedDim(i);
                x.segment(offset, dof) = active_spatial_map_->toUnconstrained(ref_waypoints_.row(i).transpose(), i);
                offset += dof;
            }

            return x;
        }

        /**
         * @brief Evaluate，约束消除 -> 轨迹生成 -> 代价评估 -> 梯度反传
         */
        template <typename TimeCostFunc, typename ControlPointCostFunc>
        double evaluate(const Eigen::Ref<const Eigen::VectorXd>& x, 
                        Eigen::Ref<Eigen::VectorXd> grad_out,
                        TimeCostFunc &&time_cost_func,
                        ControlPointCostFunc &&cp_cost_func)
        {
            static_assert(TypeTraits::HasTimeCostInterface<typename std::decay<TimeCostFunc>::type>::value, 
                          "TimeCostFunc does not satisfy requirements.");

            double total_cost = 0.0;
            grad_out.setZero();

            for (int i = 0; i < num_segments_; ++i) 
            {
                ws_->cache_T(i) = active_time_map_->toTime(x(i));
            }

            int offset = num_segments_;
            for (int i = 1; i < num_segments_; ++i) 
            {
                int dof = active_spatial_map_->getUnconstrainedDim(i);
                Eigen::VectorXd xi = x.segment(offset, dof);
                ws_->cache_P_inner.row(i - 1) = active_spatial_map_->toPhysical(xi, i).transpose();
                
                Eigen::VectorXd grad_xi = Eigen::VectorXd::Zero(dof);
                active_spatial_map_->addNormPenalty(xi, total_cost, grad_xi);
                grad_out.segment(offset, dof) += grad_xi;

                offset += dof;
            }

            traj_.generate(ws_->cache_P_inner, ws_->headState, ws_->tailState, ws_->cache_T, ws_->P_full);

            double energy_cost = 0.0;
            if (rho_energy_ > 0) {
                traj_.getEnergyPartialGradByCoeffs(energy_cost, ws_->gdC_energy);
                traj_.getEnergyPartialGradByTimes(ws_->cache_T, ws_->gdT_energy);
                total_cost += rho_energy_ * energy_cost;
            } else {
                ws_->gdC_energy.setZero();
                ws_->gdT_energy.setZero();
            }

            std::vector<double> T_vec(ws_->cache_T.data(), ws_->cache_T.data() + ws_->cache_T.size());
            total_cost += time_cost_func(T_vec, ws_->gdT_time);

            double cp_cost = 0.0;
            ws_->gdC_user.setZero();
            ws_->gdT_user.setZero();
            
            NUBSControlPointEvaluator<DIM>::evaluateTotal(
                traj_.getControlPoints(), 
                traj_.getKnots(), 
                ws_->cache_T, 
                traj_.getP(), 
                traj_.getCtrlPtNum(num_segments_), 
                traj_, 
                std::forward<ControlPointCostFunc>(cp_cost_func),
                cp_cost, 
                ws_->gdC_user, 
                ws_->gdT_user
            );
            total_cost += cp_cost;

            Eigen::MatrixXd gdC_total = rho_energy_ * ws_->gdC_energy + ws_->gdC_user;
            Eigen::VectorXd gdT_direct_total = rho_energy_ * ws_->gdT_energy + ws_->gdT_user + ws_->gdT_time;

            
            traj_.propagateGrad(gdC_total, gdT_direct_total, ws_->cache_T, ws_->gradByPoints, ws_->gradByTimes);

            for (int i = 0; i < num_segments_; ++i) 
            {
                grad_out(i) += active_time_map_->backward(x(i), ws_->cache_T(i), ws_->gradByTimes(i));
            }

            offset = num_segments_;
            for (int i = 1; i < num_segments_; ++i) {
                int dof = active_spatial_map_->getUnconstrainedDim(i);
                Eigen::Matrix<double, DIM, 1> grad_p = ws_->gradByPoints.row(i - 1).transpose();
                grad_out.segment(offset, dof) += active_spatial_map_->backwardGrad(x.segment(offset, dof), grad_p, i);
                offset += dof;
            }

            return total_cost;
        }

        const TrajType& getTrajectory() const { return traj_; }

        // =========================================================================
        //  梯度检测工具 (Gradient Checker)
        // =========================================================================
        struct GradientCheckResult
        {
            bool valid = false;          
            double error_norm = 0.0;      
            double rel_error = 0.0;       
            Eigen::VectorXd analytical;   
            Eigen::VectorXd numerical;   
            
            std::string makeReport() const {
                std::stringstream ss;
                if (valid) 
                    ss << "Gradient Check PASSED! Norm: " << error_norm << "\n";
                else
                    ss << "Gradient Check FAILED! Norm: " << error_norm << "\n";
                return ss.str();
            }
        };

        /**
         * @brief NUBS 专用的梯度检查器，对比 evaluate 的解析梯度与中心差分数值梯度
         * @param x 无约束的待优化参数向量
         * @param tf 用户的时间代价函数
         * @param cp_func 用户的控制点动力学代价函数
         * @param eps 差分步长
         * @param tol 容差限
         */
        template <typename TFunc, typename CPFunc>
        GradientCheckResult checkGradients(const Eigen::VectorXd &x, 
                                           TFunc &&tf, 
                                           CPFunc &&cp_func, 
                                           double eps = 1e-6,
                                           double tol = 1e-4)
        {
            GradientCheckResult res;

            res.analytical.resize(x.size());
            evaluate(x, res.analytical, tf, cp_func);

            res.numerical.resize(x.size());
            Eigen::VectorXd dummy_grad(x.size());
            Eigen::VectorXd x_temp = x;

            for (int i = 0; i < x.size(); ++i)
            {
                double old_val = x_temp(i);
                
                // +eps
                x_temp(i) = old_val + eps;
                double c_p = evaluate(x_temp, dummy_grad, tf, cp_func);
                
                // -eps
                x_temp(i) = old_val - eps;
                double c_m = evaluate(x_temp, dummy_grad, tf, cp_func);
                
                
                x_temp(i) = old_val;

                res.numerical(i) = (c_p - c_m) / (2.0 * eps);
            }

            evaluate(x, res.analytical, tf, cp_func);

        
            Eigen::VectorXd diff = res.analytical - res.numerical;
            res.error_norm = diff.norm();

            double grad_norm = res.analytical.norm();
            res.rel_error = (grad_norm > 1e-9) ? (res.error_norm / grad_norm) : res.error_norm;

            res.valid = (res.rel_error < 1e-5 || res.error_norm < tol);

            return res;
        }
    };

} // namespace nubs

#endif // NUBS_OPTIMIZER_HPP