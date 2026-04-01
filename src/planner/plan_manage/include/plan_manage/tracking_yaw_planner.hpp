#ifndef TRACKING_YAW_PLANNER_HPP
#define TRACKING_YAW_PLANNER_HPP

#include "optimizer/traj_types.h"
#include "CostFunctionalManager/TrackingTypes.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace ego_planner
{
class TrackingYawPlanner
{
public:
    struct YawPlanResult
    {
        std::vector<double> t;
        std::vector<double> yaw;
    };

    static double wrapAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static void unwrapInPlace(std::vector<double> &yaws)
    {
        if (yaws.empty()) return;
        for (std::size_t i = 1; i < yaws.size(); ++i)
        {
            double diff = yaws[i] - yaws[i - 1];
            while (diff > M_PI)
            {
                yaws[i] -= 2.0 * M_PI;
                diff = yaws[i] - yaws[i - 1];
            }
            while (diff < -M_PI)
            {
                yaws[i] += 2.0 * M_PI;
                diff = yaws[i] - yaws[i - 1];
            }
        }
    }

    static YawPlanResult planFacingTarget(const MINCOTraj3D &traj,
                                          const cost_functional::TrackingReference &ref,
                                          const double dt,
                                          const double max_yaw_rate,
                                          const double yaw0)
    {
        YawPlanResult out;
        const double total_t = traj.getTotalDuration();
        if (total_t <= 1.0e-6 || !ref.valid())
        {
            return out;
        }

        const double sample_dt = std::max(dt, 0.02);
        const double yaw_rate_lim = std::max(max_yaw_rate, 0.1);

        for (double t = 0.0; t <= total_t + 1.0e-6; t += sample_dt)
        {
            const double st = std::min(t, total_t);
            Eigen::Vector3d drone_pos = traj.evaluate(st, 0);

            Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
            Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
            if (!cost_functional::sampleTrackingReference(ref, st, target_pos, target_vel))
            {
                continue;
            }

            const Eigen::Vector3d rel = target_pos - drone_pos;
            double yaw_des = std::atan2(rel.y(), rel.x());

            out.t.push_back(st);
            out.yaw.push_back(yaw_des);
        }

        if (out.yaw.empty())
        {
            return out;
        }

        out.yaw.front() = yaw0;
        unwrapInPlace(out.yaw);

        // yaw rate limiting
        for (std::size_t i = 1; i < out.yaw.size(); ++i)
        {
            const double dt_i = std::max(1.0e-3, out.t[i] - out.t[i - 1]);
            const double max_step = yaw_rate_lim * dt_i;
            const double raw_step = out.yaw[i] - out.yaw[i - 1];
            const double clamped_step = std::max(-max_step, std::min(max_step, raw_step));
            out.yaw[i] = out.yaw[i - 1] + clamped_step;
        }

        for (double &yaw : out.yaw)
        {
            yaw = wrapAngle(yaw);
        }

        return out;
    }
};
}

#endif