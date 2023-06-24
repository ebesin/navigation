/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__MARKER__MARKER_HELPER_HPP_
#define MOTION_UTILS__MARKER__MARKER_HELPER_HPP_

#include "motion_utils/resample/resample_utils.hpp"
#include "mpc_utils/mpc_utils.hpp"

#include <functional>
#include <string>
#include <vector>

namespace motion_utils {
using geometry_msgs::msg::Pose;

visualization_msgs::msg::MarkerArray createStopVirtualWallMarker(const Pose&         pose,
                                                                 const std::string&  module_name,
                                                                 const rclcpp::Time& now,
                                                                 const int32_t       id,
                                                                 const double        longitudinal_offset = 0.0,
                                                                 const std::string&  ns_prefix           = "");

visualization_msgs::msg::MarkerArray createSlowDownVirtualWallMarker(const Pose&         pose,
                                                                     const std::string&  module_name,
                                                                     const rclcpp::Time& now,
                                                                     const int32_t       id,
                                                                     const double        longitudinal_offset = 0.0,
                                                                     const std::string&  ns_prefix           = "");

visualization_msgs::msg::MarkerArray createDeadLineVirtualWallMarker(const Pose&         pose,
                                                                     const std::string&  module_name,
                                                                     const rclcpp::Time& now,
                                                                     const int32_t       id,
                                                                     const double        longitudinal_offset = 0.0,
                                                                     const std::string&  ns_prefix           = "");

visualization_msgs::msg::MarkerArray createDeletedStopVirtualWallMarker(const rclcpp::Time& now, const int32_t id);

visualization_msgs::msg::MarkerArray createDeletedSlowDownVirtualWallMarker(const rclcpp::Time& now, const int32_t id);

visualization_msgs::msg::MarkerArray createDeletedDeadLineVirtualWallMarker(const rclcpp::Time& now, const int32_t id);
}   // namespace motion_utils

#endif   // MOTION_UTILS__MARKER__MARKER_HELPER_HPP_
