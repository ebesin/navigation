/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
#define MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_

#include "mpc_utils/geometry/geometry.hpp"
#include "mpc_utils/math/constants.hpp"

#include "mpc_msgs/msg/path_with_lane_id.hpp"
#include "mpc_msgs/msg/trajectory.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace motion_utils {
/**
 * @brief An interpolation function that finds the closest interpolated point on the trajectory from
 * the given pose
 * @param trajectory input trajectory
 * @param target_pose target_pose
 * @param use_zero_order_for_twist flag to decide wether to use zero order hold interpolation for
 * twist information
 * @return resampled path(poses)
 */
mpc_msgs::msg::TrajectoryPoint calcInterpolatedPoint(const mpc_msgs::msg::Trajectory& trajectory,
                                                     const geometry_msgs::msg::Pose&  target_pose,
                                                     const bool                       use_zero_order_hold_for_twist = false,
                                                     const double dist_threshold = std::numeric_limits<double>::max(),
                                                     const double yaw_threshold  = std::numeric_limits<double>::max());

/**
 * @brief An interpolation function that finds the closest interpolated point on the path from
 * the given pose
 * @param path input path
 * @param target_pose target_pose
 * @param use_zero_order_for_twist flag to decide wether to use zero order hold interpolation for
 * twist information
 * @return resampled path(poses)
 */
mpc_msgs::msg::PathPointWithLaneId calcInterpolatedPoint(const mpc_msgs::msg::PathWithLaneId& path,
                                                         const geometry_msgs::msg::Pose&      target_pose,
                                                         const bool   use_zero_order_hold_for_twist = false,
                                                         const double dist_threshold = std::numeric_limits<double>::max(),
                                                         const double yaw_threshold  = std::numeric_limits<double>::max());

/**
 * @brief An interpolation function that finds the closest interpolated point on the path that is a
 * certain length away from the given pose
 * @param points input path
 * @param target_length length from the front point of the path
 * @return resampled pose
 */
template<class T> geometry_msgs::msg::Pose calcInterpolatedPose(const T& points, const double target_length)
{
    if (points.empty()) {
        geometry_msgs::msg::Pose interpolated_pose;
        return interpolated_pose;
    }

    if (points.size() < 2 || target_length < 0.0) {
        return mpc_utils::getPose(points.front());
    }

    double accumulated_length = 0;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        const auto&  curr_pose = mpc_utils::getPose(points.at(i));
        const auto&  next_pose = mpc_utils::getPose(points.at(i + 1));
        const double length    = mpc_utils::calcDistance3d(curr_pose, next_pose);
        if (accumulated_length + length > target_length) {
            const double ratio = (target_length - accumulated_length) / std::max(length, 1e-6);
            return mpc_utils::calcInterpolatedPose(curr_pose, next_pose, ratio);
        }
        accumulated_length += length;
    }

    return mpc_utils::getPose(points.back());
}

}   // namespace motion_utils

#endif   // MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
