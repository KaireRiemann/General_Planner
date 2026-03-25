#pragma once

#include "TrajectoryOptAdapters/PolytopeProjectionSolverAdapter.hpp"
#include "TrajectoryOptComponents/SFCCommonTypes.hpp"

#include <cmath>

#include <Eigen/Eigen>

namespace traj_opt_components
{
class PolytopeSpatialMap
{
public:
    using VectorType = Eigen::Vector3d;

    const PolyhedraV *v_polys = nullptr;
    const Eigen::VectorXi *v_poly_idx = nullptr;
    int num_segments = 0;
    bool identity_mode = false;
    PolyhedraV owned_polys;
    Eigen::VectorXi owned_poly_idx;

    void reset(const PolyhedraV *polys,
               const Eigen::VectorXi *indices,
               int segments,
               bool identity = false)
    {
        v_polys = polys;
        v_poly_idx = indices;
        num_segments = segments;
        identity_mode = identity;
    }

    void reset(const PolyhedronV *poly,
               int segments,
               bool identity = false)
    {
        owned_polys.clear();
        owned_polys.push_back(*poly);
        owned_poly_idx = Eigen::VectorXi::Zero(segments);
        reset(&owned_polys, &owned_poly_idx, segments, identity);
    }

    int getUnconstrainedDim(int index) const
    {
        if (identity_mode || !v_polys || !v_poly_idx || index <= 0 || index > num_segments)
        {
            return 3;
        }
        return (*v_polys)[(*v_poly_idx)(index - 1)].cols();
    }

    VectorType toPhysical(const Eigen::VectorXd &xi, int index) const
    {
        if (identity_mode || !v_polys || !v_poly_idx || index <= 0 || index > num_segments)
        {
            return xi.head<3>();
        }

        const int poly_id = (*v_poly_idx)(index - 1);
        const auto &poly = (*v_polys)[poly_id];
        const int k = poly.cols();
        const double norm = xi.norm();
        if (norm < 1e-12)
        {
            return poly.col(0);
        }

        const Eigen::VectorXd unit_xi = xi / norm;
        const Eigen::VectorXd r = unit_xi.head(k - 1);
        return poly.rightCols(k - 1) * r.cwiseProduct(r) + poly.col(0);
    }

    Eigen::VectorXd toUnconstrained(const Eigen::VectorXd &p, int index) const
    {
        if (identity_mode || !v_polys || !v_poly_idx || index <= 0 || index > num_segments)
        {
            return p;
        }

        Eigen::Matrix3Xd point(3, 1);
        point.col(0) = p.head<3>();
        Eigen::VectorXd xi;
        backwardP(point,
                  Eigen::VectorXi::Constant(1, (*v_poly_idx)(index - 1)),
                  *v_polys,
                  xi);
        return xi;
    }

    Eigen::VectorXd backwardGrad(const Eigen::VectorXd &xi,
                                 const Eigen::VectorXd &grad_p,
                                 int index) const
    {
        if (identity_mode || !v_polys || !v_poly_idx || index <= 0 || index > num_segments)
        {
            return grad_p;
        }

        Eigen::Matrix3Xd grad_p_mat(3, 1);
        grad_p_mat.col(0) = grad_p.head<3>();
        Eigen::VectorXd grad_xi;
        backwardGradP(xi,
                      Eigen::VectorXi::Constant(1, (*v_poly_idx)(index - 1)),
                      *v_polys,
                      grad_p_mat,
                      grad_xi);
        return grad_xi;
    }

    void addNormPenalty(const Eigen::VectorXd &x,
                        int spatial_offset,
                        int spatial_dim,
                        Eigen::VectorXd &grad,
                        double &cost) const
    {
        if (identity_mode || !v_polys || !v_poly_idx || spatial_dim <= 0)
        {
            return;
        }

        const Eigen::VectorXd xi = x.segment(spatial_offset, spatial_dim);
        Eigen::VectorXd grad_xi = grad.segment(spatial_offset, spatial_dim);
        normRestrictionLayer(xi, *v_poly_idx, *v_polys, cost, grad_xi);
        grad.segment(spatial_offset, spatial_dim) = grad_xi;
    }

private:
    static inline void normRestrictionLayer(const Eigen::VectorXd &xi,
                                            const Eigen::VectorXi &v_idx,
                                            const PolyhedraV &v_polys,
                                            double &cost,
                                            Eigen::VectorXd &grad_xi)
    {
        const long size_p = v_idx.size();
        grad_xi.resize(xi.size());
        grad_xi.setZero();

        double sqr_norm_q, sqr_norm_violation, c, dc;
        Eigen::VectorXd q;
        for (long i = 0, j = 0, k; i < size_p; ++i, j += k)
        {
            k = v_polys[v_idx(i)].cols();
            q = xi.segment(j, k);
            sqr_norm_q = q.squaredNorm();
            sqr_norm_violation = sqr_norm_q - 1.0;
            if (sqr_norm_violation > 0.0)
            {
                c = sqr_norm_violation * sqr_norm_violation;
                dc = 3.0 * c;
                c *= sqr_norm_violation;
                cost += c;
                grad_xi.segment(j, k) += dc * 2.0 * q;
            }
        }
    }

    static inline void backwardGradP(const Eigen::VectorXd &xi,
                                     const Eigen::VectorXi &v_idx,
                                     const PolyhedraV &v_polys,
                                     const Eigen::Matrix3Xd &grad_p,
                                     Eigen::VectorXd &grad_xi)
    {
        const long size_p = v_idx.size();
        grad_xi.resize(xi.size());

        double norm_inv;
        Eigen::VectorXd q, grad_q, unit_q;
        for (long i = 0, j = 0, k, l; i < size_p; ++i, j += k)
        {
            l = v_idx(i);
            k = v_polys[l].cols();
            q = xi.segment(j, k);
            norm_inv = 1.0 / q.norm();
            unit_q = q * norm_inv;
            grad_q.resize(k);
            grad_q.head(k - 1) = (v_polys[l].rightCols(k - 1).transpose() * grad_p.col(i)).array() *
                                 unit_q.head(k - 1).array() * 2.0;
            grad_q(k - 1) = 0.0;
            grad_xi.segment(j, k) = (grad_q - unit_q * unit_q.dot(grad_q)) * norm_inv;
        }
    }

    static inline double costTinyNLS(void *ptr,
                                     const Eigen::VectorXd &xi,
                                     Eigen::VectorXd &gradXi)
    {
        const long n = xi.size();
        const Eigen::Matrix3Xd &ov_poly = *(Eigen::Matrix3Xd *)ptr;

        const double sqr_norm_xi = xi.squaredNorm();
        const double inv_norm_xi = 1.0 / std::sqrt(sqr_norm_xi);
        const Eigen::VectorXd unit_xi = xi * inv_norm_xi;
        const Eigen::VectorXd r = unit_xi.head(n - 1);
        const Eigen::Vector3d delta = ov_poly.rightCols(n - 1) * r.cwiseProduct(r) +
                                      ov_poly.col(1) - ov_poly.col(0);
        double cost = delta.squaredNorm();
        gradXi.head(n - 1) = (ov_poly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                             r.array() * 2.0;
        gradXi(n - 1) = 0.0;
        gradXi = (gradXi - unit_xi.dot(gradXi) * unit_xi).eval() * inv_norm_xi;

        const double sqr_norm_violation = sqr_norm_xi - 1.0;
        if (sqr_norm_violation > 0.0)
        {
            double c = sqr_norm_violation * sqr_norm_violation;
            const double dc = 3.0 * c;
            c *= sqr_norm_violation;
            cost += c;
            gradXi += dc * 2.0 * xi;
        }

        return cost;
    }

    static inline void backwardP(const Eigen::Matrix3Xd &P,
                                 const Eigen::VectorXi &v_idx,
                                 const PolyhedraV &v_polys,
                                 Eigen::VectorXd &xi)
    {
        const long size_p = P.cols();
        long xi_dim = 0;
        for (long i = 0; i < size_p; ++i)
        {
            xi_dim += v_polys[v_idx(i)].cols();
        }
        xi.resize(xi_dim);

        double min_sqr_d;
        Eigen::Matrix3Xd ov_poly;
        for (long i = 0, j = 0, k, l; i < size_p; ++i, j += k)
        {
            l = v_idx(i);
            k = v_polys[l].cols();

            ov_poly.resize(3, k + 1);
            ov_poly.col(0) = P.col(i);
            ov_poly.rightCols(k) = v_polys[l];
            Eigen::VectorXd x(k);
            x.setConstant(std::sqrt(1.0 / static_cast<double>(k)));
            traj_opt_adapters::PolytopeProjectionSolverAdapter::optimize(x,
                                                                         min_sqr_d,
                                                                         &PolytopeSpatialMap::costTinyNLS,
                                                                         &ov_poly);

            xi.segment(j, k) = x;
        }
    }
};
}
