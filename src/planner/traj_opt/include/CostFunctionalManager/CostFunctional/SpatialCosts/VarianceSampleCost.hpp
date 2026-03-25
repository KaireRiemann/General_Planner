#ifndef VARIANCE_SAMPLE_COST_HPP
#define VARIANCE_SAMPLE_COST_HPP

#include <Eigen/Eigen>
#include "CostFunctional/PenaltyUtils.hpp"
#include "CostFunctionalManager/PlanningTypesAdapter.hpp"

namespace cost_functional
{
    inline double accumulateVarianceSampleCost(const Eigen::MatrixXd &C,
                                               const double weight_sqrvar,
                                               Eigen::MatrixXd &gdC)
    {
        if (weight_sqrvar <= 0.0 || C.rows() < 2)
        {
            return 0.0;
        }

        const int point_num = C.rows();
        const int edge_num = point_num - 1;

        Eigen::MatrixXd dps = C.bottomRows(edge_num) - C.topRows(edge_num);
        
        Eigen::VectorXd dsqrs = dps.rowwise().squaredNorm();
        
        const double dquarsum = dsqrs.squaredNorm();
        const double dquarmean = dquarsum / edge_num;
        double cost = weight_sqrvar * dquarmean;

        for (int i = 0; i <= edge_num; ++i)
        {
            if (i != 0) 
            {
                gdC.row(i) += weight_sqrvar * (4.0 * dsqrs(i - 1) / edge_num * dps.row(i - 1));
            }
            if (i != edge_num) 
            {
                gdC.row(i) += weight_sqrvar * (-4.0 * dsqrs(i) / edge_num * dps.row(i));
            }
        }

        return cost;
    }

}//namespace cost_functional

#endif