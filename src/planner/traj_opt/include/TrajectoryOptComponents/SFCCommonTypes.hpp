#pragma once

#include <vector>

#include <Eigen/Eigen>

namespace traj_opt_components
{
using PolyhedronV = Eigen::Matrix3Xd;
using PolyhedronH = Eigen::MatrixX4d;
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
using PolyhedraV = AlignedVector<PolyhedronV>;
using PolyhedraH = AlignedVector<PolyhedronH>;
}
