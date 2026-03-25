#ifndef PENALTY_UTILS_HPP
#define PENALTY_UTILS_HPP

#include<cmath>

namespace cost_functional
{
    inline bool smoothedL1(const double &x,
                       const double &mu,
                       double &f,
                       double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }

        const double xdmu = x / mu;
        const double sqrxdmu = xdmu * xdmu;
        const double mumxd2 = mu - 0.5 * x;
        f = mumxd2 * sqrxdmu * xdmu;
        df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
        return true;
    }

    inline bool quadraticPenalty(const double &x, 
                                 double &f, double &df)
    {
        if(x <= 0.0)
        {
            return false;
        }

        f = x * x;
        df = 2.0 * x;
        return true;
    }

    inline bool positivePartCubic(const double &x,
                              double &f,
                              double &df)
    {
        if (x <= 0.0)
        {
            return false;
        }

        const double sqr_x = x * x;
        f = sqr_x * x;
        df = 3.0 * sqr_x;
        return true;
    }

}//cost_functional

#endif