/*
 * @Author       : dwayne
 * @Date         : 2023-06-27
 * @LastEditTime : 2023-06-27
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#pragma once

#include "coor_tools.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace utils_tool {

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    tf2::Matrix3x3 m(tf_q);
    double         roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}
}   // namespace utils_tool