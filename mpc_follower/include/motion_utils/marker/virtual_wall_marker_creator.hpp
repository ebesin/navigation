/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__MARKER__VIRTUAL_WALL_MARKER_CREATOR_HPP_
#define MOTION_UTILS__MARKER__VIRTUAL_WALL_MARKER_CREATOR_HPP_

#include "motion_utils/marker/marker_helper.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace motion_utils {

/// @brief type of virtual wall associated with different marker styles and namespace
enum VirtualWallType
{
    stop,
    slowdown,
    deadline
};
/// @brief virtual wall to be visualized in rviz
struct VirtualWall
{
    geometry_msgs::msg::Pose pose{};
    std::string              text{};
    std::string              ns{};
    VirtualWallType          style = stop;
    double                   longitudinal_offset{};
};
typedef std::vector<VirtualWall> VirtualWalls;

/// @brief class to manage the creation of virtual wall markers
/// @details creates both ADD and DELETE markers
class VirtualWallMarkerCreator
{
    struct MarkerCount
    {
        size_t previous = 0UL;
        size_t current  = 0UL;
    };

    using create_wall_function = std::function<visualization_msgs::msg::MarkerArray(const geometry_msgs::msg::Pose& pose,
                                                                                    const std::string&              module_name,
                                                                                    const rclcpp::Time&             now,
                                                                                    const int32_t                   id,
                                                                                    const double       longitudinal_offset,
                                                                                    const std::string& ns_prefix)>;

    VirtualWalls                                 virtual_walls;
    std::unordered_map<std::string, MarkerCount> marker_count_per_namespace;

    /// @brief internal cleanup: clear the stored markers and remove unused namespace from the map
    void cleanup();

public:
    /// @brief add a virtual wall
    /// @param virtual_wall virtual wall to add
    void add_virtual_wall(const VirtualWall& virtual_wall);
    /// @brief add virtual walls
    /// @param virtual_walls virtual walls to add
    void add_virtual_walls(const VirtualWalls& walls);

    /// @brief create markers for the stored virtual walls
    /// @details also create DELETE markers for the namespace+ids that are no longer used
    /// @param now current time to be used for displaying the markers
    visualization_msgs::msg::MarkerArray create_markers(const rclcpp::Time& now = rclcpp::Time());
};
}   // namespace motion_utils

#endif   // MOTION_UTILS__MARKER__VIRTUAL_WALL_MARKER_CREATOR_HPP_
