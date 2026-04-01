#ifndef TRACKING_TYPES_HPP
#define TRACKING_TYPES_HPP

#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace cost_functional
{
    struct TrackingReference
    {
        std::vector<double> t_ref;
        std::vector<Eigen::Vector3d> p_ref;
        std::vector<Eigen::Vector3d> v_ref;
        std::vector<double> t_view_ref;
        std::vector<Eigen::Vector3d> p_view_ref;
        std::vector<Eigen::Vector3d> v_view_ref;

        bool valid() const
        {
            if (t_ref.empty() || p_ref.empty() || t_ref.size() != p_ref.size())
            {
                return false;
            }
            if (!v_ref.empty() && v_ref.size() != p_ref.size())
            {
                return false;
            }
            for (std::size_t i = 1; i < t_ref.size(); ++i)
            {
                if (t_ref[i] + 1.0e-9 < t_ref[i - 1])
                {
                    return false;
                }
            }
            return true;
        }

        bool empty() const
        {
            return !valid();
        }

        bool viewValid() const
        {
            if (t_view_ref.empty() || p_view_ref.empty() || t_view_ref.size() != p_view_ref.size())
            {
                return false;
            }
            if (!v_view_ref.empty() && v_view_ref.size() != p_view_ref.size())
            {
                return false;
            }
            for (std::size_t i = 1; i < t_view_ref.size(); ++i)
            {
                if (t_view_ref[i] + 1.0e-9 < t_view_ref[i - 1])
                {
                    return false;
                }
            }
            return true;
        }
    };

    namespace detail
    {
        inline bool sampleReferenceSeries(const std::vector<double> &t_ref,
                                          const std::vector<Eigen::Vector3d> &p_ref,
                                          const std::vector<Eigen::Vector3d> &v_ref,
                                          Eigen::Vector3d &p_out,
                                          Eigen::Vector3d &v_out,
                                          const double t_query)
        {
            if (t_ref.empty() || p_ref.empty() || t_ref.size() != p_ref.size())
            {
                return false;
            }
            if (!v_ref.empty() && v_ref.size() != p_ref.size())
            {
                return false;
            }

            if (t_query <= t_ref.front())
            {
                p_out = p_ref.front();
                if (!v_ref.empty())
                {
                    v_out = v_ref.front();
                }
                else if (t_ref.size() >= 2)
                {
                    const double dt = std::max(1.0e-6, t_ref[1] - t_ref[0]);
                    v_out = (p_ref[1] - p_ref[0]) / dt;
                }
                else
                {
                    v_out.setZero();
                }
                return true;
            }

            if (t_query >= t_ref.back())
            {
                p_out = p_ref.back();
                if (!v_ref.empty())
                {
                    v_out = v_ref.back();
                }
                else if (t_ref.size() >= 2)
                {
                    const std::size_t n = t_ref.size();
                    const double dt = std::max(1.0e-6, t_ref[n - 1] - t_ref[n - 2]);
                    v_out = (p_ref[n - 1] - p_ref[n - 2]) / dt;
                }
                else
                {
                    v_out.setZero();
                }
                return true;
            }

            const auto upper = std::upper_bound(t_ref.begin(), t_ref.end(), t_query);
            const std::size_t idx1 = static_cast<std::size_t>(std::distance(t_ref.begin(), upper));
            const std::size_t idx0 = idx1 - 1;
            const double t0 = t_ref[idx0];
            const double t1 = t_ref[idx1];
            const double dt = std::max(1.0e-6, t1 - t0);
            const double alpha = std::max(0.0, std::min(1.0, (t_query - t0) / dt));

            p_out = (1.0 - alpha) * p_ref[idx0] + alpha * p_ref[idx1];

            if (!v_ref.empty())
            {
                v_out = (1.0 - alpha) * v_ref[idx0] + alpha * v_ref[idx1];
            }
            else
            {
                v_out = (p_ref[idx1] - p_ref[idx0]) / dt;
            }
            return true;
        }
    } // namespace detail

    inline bool sampleTrackingReference(const TrackingReference &ref,
                                        const double t_query,
                                        Eigen::Vector3d &p_out,
                                        Eigen::Vector3d &v_out)
    {
        return ref.valid() &&
               detail::sampleReferenceSeries(ref.t_ref, ref.p_ref, ref.v_ref, p_out, v_out, t_query);
    }

    inline bool sampleTrackingViewReference(const TrackingReference &ref,
                                            const double t_query,
                                            Eigen::Vector3d &p_out,
                                            Eigen::Vector3d &v_out)
    {
        return ref.viewValid() &&
               detail::sampleReferenceSeries(ref.t_view_ref, ref.p_view_ref, ref.v_view_ref, p_out, v_out, t_query);
    }

    inline Eigen::Vector3d terminalTrackingPosition(const TrackingReference &ref)
    {
        return (ref.valid() ? ref.p_ref.back() : Eigen::Vector3d::Zero());
    }

    inline Eigen::Vector3d terminalTrackingVelocity(const TrackingReference &ref)
    {
        if (!ref.valid())
        {
            return Eigen::Vector3d::Zero();
        }
        if (!ref.v_ref.empty())
        {
            return ref.v_ref.back();
        }
        if (ref.t_ref.size() >= 2)
        {
            const std::size_t n = ref.t_ref.size();
            const double dt = std::max(1.0e-6, ref.t_ref[n - 1] - ref.t_ref[n - 2]);
            return (ref.p_ref[n - 1] - ref.p_ref[n - 2]) / dt;
        }
        return Eigen::Vector3d::Zero();
    }
} // namespace cost_functional

#endif
