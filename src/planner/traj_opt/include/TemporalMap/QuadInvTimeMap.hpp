#ifndef QUAD_INV_TIME_MAP_HPP
#define QUAD_INV_TIME_MAP_HPP

#include<cmath>
#include<cfloat>

namespace temporal_map
{
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

}//namespace temporal_map

#endif