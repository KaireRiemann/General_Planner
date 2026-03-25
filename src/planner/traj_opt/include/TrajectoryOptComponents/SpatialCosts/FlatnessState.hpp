#pragma once

#include <Eigen/Eigen>

namespace traj_opt_components
{
struct FlatnessPenaltyState
{
    double thrust = 0.0;
    Eigen::Vector4d quaternion = Eigen::Vector4d::Zero();
    Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
};

template <typename FlatnessMapPtr, typename VecT>
inline FlatnessPenaltyState evaluateFlatnessPenaltyState(FlatnessMapPtr flatmap,
                                                         const VecT &velocity,
                                                         const VecT &acceleration,
                                                         const VecT &jerk,
                                                         const double psi = 0.0,
                                                         const double dpsi = 0.0)
{
    FlatnessPenaltyState state;
    if (flatmap != nullptr)
    {
        flatmap->forward(velocity,
                         acceleration,
                         jerk,
                         psi,
                         dpsi,
                         state.thrust,
                         state.quaternion,
                         state.angular_rate);
    }
    return state;
}
} // namespace traj_opt_components
