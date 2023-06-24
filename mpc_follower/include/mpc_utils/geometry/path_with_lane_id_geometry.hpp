/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__PATH_WITH_LANE_ID_GEOMETRY_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__PATH_WITH_LANE_ID_GEOMETRY_HPP_

#include "mpc_utils/geometry/geometry.hpp"

#include <mpc_msgs/msg/path_with_lane_id.hpp>

namespace mpc_utils {
template<> inline geometry_msgs::msg::Point getPoint(const mpc_msgs::msg::PathPointWithLaneId& p)
{
    return p.point.pose.position;
}

template<> inline geometry_msgs::msg::Pose getPose(const mpc_msgs::msg::PathPointWithLaneId& p)
{
    return p.point.pose;
}

template<> inline double getLongitudinalVelocity(const mpc_msgs::msg::PathPointWithLaneId& p)
{
    return p.point.longitudinal_velocity_mps;
}

template<> inline void setPose(const geometry_msgs::msg::Pose& pose, mpc_msgs::msg::PathPointWithLaneId& p)
{
    p.point.pose = pose;
}

template<> inline void setLongitudinalVelocity(const double velocity, mpc_msgs::msg::PathPointWithLaneId& p)
{
    p.point.longitudinal_velocity_mps = velocity;
}
}   // namespace mpc_utils

#endif   // TIER4_AUTOWARE_UTILS__GEOMETRY__PATH_WITH_LANE_ID_GEOMETRY_HPP_
