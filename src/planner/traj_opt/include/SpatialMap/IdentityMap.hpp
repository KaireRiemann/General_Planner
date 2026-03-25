#ifndef IDENTITY_SPATIAL_MAP_HPP
#define IDENTITY_SPATIAL_MAP_HPP

#include<cmath>
#include<cfloat>

namespace spatial_map
{
    /**
     * @brief IdentitySpatialMap
     * Default unconstrained mapping (xi = p).
     */
    template <int DIM>
    struct IdentitySpatialMap
    {
        using VectorType = Eigen::Matrix<double, DIM, 1>;

        int getUnconstrainedDim(int index) const { return DIM; }

        VectorType toPhysical(const VectorType& xi, int index) const
        {
            return xi;
        }

        VectorType toUnconstrained(const VectorType& p, int index) const
        {
            return p;
        }

        VectorType backwardGrad(const VectorType& xi, const VectorType& grad_p, int index) const
        {
            return grad_p;
        }

        void addNormPenalty(const Eigen::VectorXd& /*xi*/, double& /*cost*/, Eigen::VectorXd& /*grad_xi*/) const {}
    };

}//namespace spatial_map


#endif