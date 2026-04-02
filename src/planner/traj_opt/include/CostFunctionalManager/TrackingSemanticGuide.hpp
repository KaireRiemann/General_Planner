#ifndef TRACKING_SEMANTIC_GUIDE_HPP
#define TRACKING_SEMANTIC_GUIDE_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cost_functional
{
    struct VisibleFanRegion
    {
        double t{0.0};
        Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d target_velocity{Eigen::Vector3d::Zero()};
        double min_tracking_distance{0.0};
        double max_tracking_distance{0.0};
        double yaw_center{0.0};
        double yaw_half_span{0.0};
        double z_center{0.0};
        double z_half_span{0.0};
        double visibility_margin{0.0};
        Eigen::Vector3d preferred_viewpoint{Eigen::Vector3d::Zero()};
        Eigen::Vector3d preferred_view_velocity{Eigen::Vector3d::Zero()};
        bool valid{false};
    };

    struct SemanticGuideEdge
    {
        int from_region_idx{-1};
        int to_region_idx{-1};
        std::vector<Eigen::Vector3d> dense_path;
        std::vector<double> dense_times;
        double visibility_ratio{0.0};
        double min_visibility_margin{0.0};
        double mean_distance_error{0.0};
        bool valid{false};
    };

    struct TrackingSemanticGuide
    {
        std::vector<Eigen::Vector3d> target_samples;
        std::vector<Eigen::Vector3d> target_vel_samples;
        std::vector<Eigen::Vector3d> viewpoint_series;
        std::vector<double> viewpoint_times;
        std::vector<VisibleFanRegion> visible_regions;
        std::vector<SemanticGuideEdge> edges;
        std::vector<Eigen::Vector3d> corridor_seed_path;
        std::vector<double> corridor_seed_times;
        std::vector<Eigen::Vector3d> candidate_points;
        bool valid{false};

        double hypothesis_score{-std::numeric_limits<double>::infinity()};
        double visibility_score{0.0};
        double path_length{0.0};

        void clear()
        {
            target_samples.clear();
            target_vel_samples.clear();
            viewpoint_series.clear();
            viewpoint_times.clear();
            visible_regions.clear();
            edges.clear();
            corridor_seed_path.clear();
            corridor_seed_times.clear();
            candidate_points.clear();
            valid = false;
            hypothesis_score = -std::numeric_limits<double>::infinity();
            visibility_score = 0.0;
            path_length = 0.0;
        }

        bool consistent() const
        {
            if (!valid)
            {
                return false;
            }
            if (viewpoint_series.empty() || viewpoint_times.size() != viewpoint_series.size())
            {
                return false;
            }
            if (!target_samples.empty() && target_samples.size() != viewpoint_series.size())
            {
                return false;
            }
            if (!target_vel_samples.empty() && target_vel_samples.size() != viewpoint_series.size())
            {
                return false;
            }
            if (!visible_regions.empty() && visible_regions.size() != viewpoint_series.size())
            {
                return false;
            }
            if (!edges.empty() && edges.size() + 1 != viewpoint_series.size())
            {
                return false;
            }
            if (corridor_seed_path.size() < 2 || corridor_seed_times.size() != corridor_seed_path.size())
            {
                return false;
            }
            for (std::size_t i = 0; i < viewpoint_times.size(); ++i)
            {
                if (!std::isfinite(viewpoint_times[i]) || !viewpoint_series[i].allFinite())
                {
                    return false;
                }
                if (i > 0 && viewpoint_times[i] + 1.0e-9 < viewpoint_times[i - 1])
                {
                    return false;
                }
            }
            for (std::size_t i = 0; i < corridor_seed_times.size(); ++i)
            {
                if (!std::isfinite(corridor_seed_times[i]) || !corridor_seed_path[i].allFinite())
                {
                    return false;
                }
                if (i > 0 && corridor_seed_times[i] + 1.0e-9 < corridor_seed_times[i - 1])
                {
                    return false;
                }
            }
            return true;
        }
    };

    namespace semantic_guide
    {
        inline double wrapAngle(double a)
        {
            while (a > M_PI)
            {
                a -= 2.0 * M_PI;
            }
            while (a < -M_PI)
            {
                a += 2.0 * M_PI;
            }
            return a;
        }

        inline double angleDiff(double a, double b)
        {
            return wrapAngle(a - b);
        }

        inline double interpolateAngle(double a0, double a1, const double alpha)
        {
            return wrapAngle(a0 + alpha * angleDiff(a1, a0));
        }

        inline bool sampleVisibleFanRegion(const TrackingSemanticGuide &guide,
                                           const double t_query,
                                           VisibleFanRegion &region_out)
        {
            if (!guide.consistent() || guide.visible_regions.empty())
            {
                return false;
            }

            if (t_query <= guide.visible_regions.front().t)
            {
                region_out = guide.visible_regions.front();
                return region_out.valid;
            }
            if (t_query >= guide.visible_regions.back().t)
            {
                region_out = guide.visible_regions.back();
                return region_out.valid;
            }

            auto upper = std::upper_bound(
                guide.visible_regions.begin(),
                guide.visible_regions.end(),
                t_query,
                [](double t, const VisibleFanRegion &region)
                { return t < region.t; });

            if (upper == guide.visible_regions.begin() || upper == guide.visible_regions.end())
            {
                if (upper == guide.visible_regions.end())
                {
                    region_out = guide.visible_regions.back();
                }
                else
                {
                    region_out = guide.visible_regions.front();
                }
                return region_out.valid;
            }

            const std::size_t idx1 = static_cast<std::size_t>(std::distance(guide.visible_regions.begin(), upper));
            const std::size_t idx0 = idx1 - 1;
            const VisibleFanRegion &r0 = guide.visible_regions[idx0];
            const VisibleFanRegion &r1 = guide.visible_regions[idx1];
            const double dt = std::max(1.0e-6, r1.t - r0.t);
            const double alpha = std::max(0.0, std::min(1.0, (t_query - r0.t) / dt));

            region_out = r0;
            region_out.t = t_query;
            region_out.target_position = (1.0 - alpha) * r0.target_position + alpha * r1.target_position;
            region_out.target_velocity = (1.0 - alpha) * r0.target_velocity + alpha * r1.target_velocity;
            region_out.min_tracking_distance =
                (1.0 - alpha) * r0.min_tracking_distance + alpha * r1.min_tracking_distance;
            region_out.max_tracking_distance =
                (1.0 - alpha) * r0.max_tracking_distance + alpha * r1.max_tracking_distance;
            region_out.yaw_center = interpolateAngle(r0.yaw_center, r1.yaw_center, alpha);
            region_out.yaw_half_span = (1.0 - alpha) * r0.yaw_half_span + alpha * r1.yaw_half_span;
            region_out.z_center = (1.0 - alpha) * r0.z_center + alpha * r1.z_center;
            region_out.z_half_span = (1.0 - alpha) * r0.z_half_span + alpha * r1.z_half_span;
            region_out.visibility_margin = (1.0 - alpha) * r0.visibility_margin + alpha * r1.visibility_margin;
            region_out.preferred_viewpoint = (1.0 - alpha) * r0.preferred_viewpoint + alpha * r1.preferred_viewpoint;
            region_out.preferred_view_velocity =
                (1.0 - alpha) * r0.preferred_view_velocity + alpha * r1.preferred_view_velocity;
            region_out.valid = r0.valid && r1.valid;
            return region_out.valid;
        }

        inline bool pointInsideVisibleFan(const Eigen::Vector3d &position,
                                          const VisibleFanRegion &region,
                                          const double dist_margin = 0.0,
                                          const double yaw_margin = 0.0,
                                          const double z_margin = 0.0)
        {
            if (!region.valid || !position.allFinite())
            {
                return false;
            }

            const Eigen::Vector3d rel = position - region.target_position;
            const double dist_xy = rel.head<2>().norm();
            if (dist_xy < std::max(0.0, region.min_tracking_distance - dist_margin) ||
                dist_xy > region.max_tracking_distance + dist_margin)
            {
                return false;
            }

            const double yaw = std::atan2(rel.y(), rel.x());
            if (std::abs(angleDiff(yaw, region.yaw_center)) > region.yaw_half_span + yaw_margin)
            {
                return false;
            }

            if (std::abs(position.z() - region.z_center) > region.z_half_span + z_margin)
            {
                return false;
            }
            return true;
        }
    } // namespace semantic_guide

} // namespace cost_functional

#endif
