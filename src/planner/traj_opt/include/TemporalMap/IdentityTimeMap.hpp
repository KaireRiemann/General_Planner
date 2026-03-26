#ifndef IDENTITY_TIME_MAP_HPP
#define IDENTITY_TIME_MAP_HPP

#include<cmath>
#include<cfloat>

namespace temporal_map
{
    struct IdentityTimeMap
    {
        double toTime(double tau) const { return tau; }
        double toTau(double T) const { return T; }
        double backward(double tau, double T, double gradT) const { return gradT; }
    };

}//namespace temporal_map

#endif