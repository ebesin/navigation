/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__TRAJECTORY__TMP_CONVERSION_HPP_
#define MOTION_UTILS__TRAJECTORY__TMP_CONVERSION_HPP_

#include "mpc_utils/geometry/geometry.hpp"
#include "mpc_utils/geometry/pose_deviation.hpp"

#include "mpc_msgs/msg/trajectory.hpp"
#include "mpc_msgs/msg/trajectory_point.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <vector>

namespace motion_utils {
/**
 * @brief Convert std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> to
 * autoware_auto_planning_msgs::msg::Trajectory. This function is temporarily added for porting to
 * autoware_auto_msgs. We should consider whether to remove this function after the porting is done.
 * @attention This function just clips
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> up to the capacity of Trajectory.
 * Therefore, the error handling out of this function is necessary if the size of the input greater
 * than the capacity.
 * @todo Decide how to handle the situation that we need to use the trajectory with the size of
 * points larger than the capacity. (Tier IV)
 */
inline mpc_msgs::msg::Trajectory convertToTrajectory(const std::vector<mpc_msgs::msg::TrajectoryPoint>& trajectory)
{
    mpc_msgs::msg::Trajectory output{};
    for (const auto& pt : trajectory) {
        output.points.push_back(pt);
        if (output.points.size() >= output.CAPACITY) {
            break;
        }
    }
    return output;
}

/**
 * @brief Convert autoware_auto_planning_msgs::msg::Trajectory to
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>.
 */
inline std::vector<mpc_msgs::msg::TrajectoryPoint> convertToTrajectoryPointArray(const mpc_msgs::msg::Trajectory& trajectory)
{
    std::vector<mpc_msgs::msg::TrajectoryPoint> output(trajectory.points.size());
    std::copy(trajectory.points.begin(), trajectory.points.end(), output.begin());
    return output;
}

}   // namespace motion_utils

#endif   // MOTION_UTILS__TRAJECTORY__TMP_CONVERSION_HPP_
