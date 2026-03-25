#pragma once

#include "optimizer/lbfgs.hpp"

#include <Eigen/Eigen>

#include <cfloat>

namespace traj_opt_adapters
{
class PolytopeProjectionSolverAdapter
{
public:
    using CostFunction = double (*)(void *instance,
                                    const Eigen::VectorXd &x,
                                    Eigen::VectorXd &g);

    static inline void optimize(Eigen::VectorXd &x,
                                double &min_cost,
                                CostFunction cost_function,
                                void *instance)
    {
        struct CallbackData
        {
            CostFunction cost_function;
            void *instance;
            Eigen::VectorXd gradient;
        };

        auto evaluate = [](void *ptr, const double *x_raw, double *g_raw, const int n) -> double
        {
            auto *data = static_cast<CallbackData *>(ptr);
            Eigen::Map<const Eigen::VectorXd> eigen_x(x_raw, n);
            data->gradient.resize(n);
            const double cost = data->cost_function(data->instance, eigen_x, data->gradient);
            Eigen::Map<Eigen::VectorXd>(g_raw, n) = data->gradient;
            return cost;
        };

        CallbackData data{cost_function, instance, Eigen::VectorXd()};
        lbfgs::lbfgs_parameter_t params;
        lbfgs::lbfgs_load_default_parameters(&params);
        params.past = 0;
        params.delta = 1.0e-5;
        params.g_epsilon = FLT_EPSILON;
        params.max_iterations = 128;
        lbfgs::lbfgs_optimize(static_cast<int>(x.size()),
                              x.data(),
                              &min_cost,
                              evaluate,
                              nullptr,
                              nullptr,
                              &data,
                              &params);
    }
};
} // namespace traj_opt_adapters
