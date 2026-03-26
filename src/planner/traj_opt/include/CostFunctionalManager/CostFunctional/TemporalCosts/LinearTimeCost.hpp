#ifndef LINEAR_TIME_COST_HPP
#define LINEAR_TIME_COST_HPP

#include <vector>

#include <Eigen/Eigen>

namespace cost_functional
{
struct LinearTimeCost
{
    double weight = 0.0;

    double operator()(const std::vector<double> &Ts, Eigen::VectorXd &grad) const
    {
        double cost = 0.0;
        for (size_t i = 0; i < Ts.size(); ++i)
        {
            cost += weight * Ts[i];
            grad(i) += weight;
        }
        return cost;
    }
};
}//namespace cost_functional

#endif
