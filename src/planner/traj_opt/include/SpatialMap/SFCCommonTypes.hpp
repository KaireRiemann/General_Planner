#ifndef SFC_COMMON_TYPES_HPP
#define SFC_COMMON_TYPES_HPP

#include <vector>
#include <Eigen/Eigen>

namespace spatial_map
{
    using PolyhedronV = Eigen::Matrix3Xd;
    using PolyhedronH = Eigen::MatrixX4d;
    using PolyhedraV = std::vector<PolyhedronV>;
    using PolyhedraH = std::vector<PolyhedronH>;
}

#endif
