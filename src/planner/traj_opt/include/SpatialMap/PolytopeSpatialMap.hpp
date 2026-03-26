#ifndef SPLINE_TRAJECTORY_POLYTOPE_SPATIAL_MAP_HPP
#define SPLINE_TRAJECTORY_POLYTOPE_SPATIAL_MAP_HPP

#include "SpatialMap/SFCCommonTypes.hpp"
#include "optimizer/lbfgs.hpp"

#include <cmath>
#include <cfloat>

namespace spatial_map 
{
struct PolytopeSpatialMap
{
    using VectorType = Eigen::Vector3d;

    const PolyhedraV *v_polys = nullptr;
    const Eigen::VectorXi *v_poly_idx = nullptr;
    int num_segments = 0;

    void reset(const PolyhedraV *polys, const Eigen::VectorXi *indices, int segments)
    {
        v_polys = polys;
        v_poly_idx = indices;
        num_segments = segments;
    }

    int getUnconstrainedDim(int index) const
    {
        if (!v_polys || !v_poly_idx || index <= 0 || index >= num_segments)
            return 3;
        const int poly_id = (*v_poly_idx)(index - 1);
        return (*v_polys)[poly_id].cols();
    }

    VectorType toPhysical(const Eigen::VectorXd &xi, int index) const
    {
        if (!v_polys || !v_poly_idx || index <= 0 || index >= num_segments)
            return xi.head<3>();

        const int poly_id = (*v_poly_idx)(index - 1);
        const PolyhedronV &poly = (*v_polys)[poly_id];
        const int k = poly.cols();

        const double norm = xi.norm();
        if (norm < 1e-12)
            return poly.col(0);

        const Eigen::VectorXd unit_xi = xi / norm;
        const Eigen::VectorXd r = unit_xi.head(k - 1);
        return poly.rightCols(k - 1) * r.cwiseProduct(r) + poly.col(0);
    }

    Eigen::VectorXd toUnconstrained(const Eigen::VectorXd &p, int index) const
    {
        if (!v_polys || !v_poly_idx || index <= 0 || index >= num_segments)
            return p;

        const int poly_id = (*v_poly_idx)(index - 1);
        const PolyhedronV &poly = (*v_polys)[poly_id];
        const int k = poly.cols();

        Eigen::Matrix3Xd ov_poly(3, k + 1);
        ov_poly.col(0) = p;
        ov_poly.rightCols(k) = poly;

        Eigen::VectorXd xi(k);
        xi.setConstant(std::sqrt(1.0 / static_cast<double>(k)));

        lbfgs::lbfgs_parameter_t params;
        lbfgs::lbfgs_load_default_parameters(&params); // 最好先加载一次默认参数防脏数据
        params.past = 0;
        params.delta = 1.0e-5;
        params.g_epsilon = FLT_EPSILON;
        params.max_iterations = 128;

        double min_cost = 0.0;
        
        // --- 核心修复 1：适配 C-Style 调用签名 ---
        // 参数依次为：维度(n), 数据指针(x), 成本指针(fx), 评估函数, 步长函数, 进度函数, 实例指针(instance), 优化参数
        lbfgs::lbfgs_optimize(
            xi.size(), 
            xi.data(), 
            &min_cost, 
            &PolytopeSpatialMap::costTinyNLS,
            nullptr, 
            nullptr, 
            &ov_poly, 
            &params
        );

        return xi;
    }

    Eigen::VectorXd backwardGrad(const Eigen::VectorXd &xi,
                                 const Eigen::VectorXd &grad_p,
                                 int index) const
    {
        if (!v_polys || !v_poly_idx || index <= 0 || index >= num_segments)
            return grad_p;

        const int poly_id = (*v_poly_idx)(index - 1);
        const PolyhedronV &poly = (*v_polys)[poly_id];
        const int k = poly.cols();

        Eigen::VectorXd grad_xi = Eigen::VectorXd::Zero(k);
        const double norm = xi.norm();
        if (norm < 1e-12)
            return grad_xi;

        const double norm_inv = 1.0 / norm;
        const Eigen::VectorXd unit_xi = xi * norm_inv;
        Eigen::VectorXd grad_q(k);
        grad_q.head(k - 1) = (poly.rightCols(k - 1).transpose() * grad_p).array() *
                             unit_xi.head(k - 1).array() * 2.0;
        grad_q(k - 1) = 0.0;
        grad_xi = (grad_q - unit_xi * unit_xi.dot(grad_q)) * norm_inv;
        return grad_xi;
    }

    void addNormPenalty(const Eigen::VectorXd& xi, double& cost, Eigen::VectorXd& grad_xi) const 
    {
        double sqrNormViolation = xi.squaredNorm() - 1.0;
        if (sqrNormViolation > 0.0) {
            double c = sqrNormViolation * sqrNormViolation;
            cost += c * sqrNormViolation; 
            grad_xi += (3.0 * c) * 2.0 * xi;
        }
    }

private:
    // --- 核心修复 2：完全匹配 lbfgs_evaluate_t 的 C-Style 签名 ---
    static double costTinyNLS(void *ptr,
                              const double *x,
                              double *g,
                              const int n)
    {
        // 使用 Eigen::Map 零开销包装 C 数组，完美复用后续的 Eigen 代码！
        Eigen::Map<const Eigen::VectorXd> xi(x, n);
        Eigen::Map<Eigen::VectorXd> gradXi(g, n);

        const Eigen::Matrix3Xd &ov_poly = *(Eigen::Matrix3Xd *)ptr;

        const double sqrNormXi = xi.squaredNorm();
        const double invNormXi = 1.0 / std::sqrt(sqrNormXi);
        const Eigen::VectorXd unitXi = xi * invNormXi;
        const Eigen::VectorXd r = unitXi.head(n - 1);
        const Eigen::Vector3d delta = ov_poly.rightCols(n - 1) * r.cwiseProduct(r) +
                                      ov_poly.col(1) - ov_poly.col(0);

        double cost = delta.squaredNorm();
        gradXi.head(n - 1) = (ov_poly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                             r.array() * 2.0;
        gradXi(n - 1) = 0.0;
        gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

        const double sqrNormViolation = sqrNormXi - 1.0;
        if (sqrNormViolation > 0.0)
        {
            double c = sqrNormViolation * sqrNormViolation;
            const double dc = 3.0 * c;
            c *= sqrNormViolation;
            cost += c;
            gradXi += dc * 2.0 * xi;
        }

        return cost;
    }
};
} // namespace spatial_map

#endif