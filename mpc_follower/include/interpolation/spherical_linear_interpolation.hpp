/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_
#define INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_

#include "interpolation/interpolation_utils.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#    include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#    include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <vector>

namespace interpolation {
geometry_msgs::msg::Quaternion slerp(const geometry_msgs::msg::Quaternion& src_quat,
                                     const geometry_msgs::msg::Quaternion& dst_quat,
                                     const double                          ratio);

std::vector<geometry_msgs::msg::Quaternion> slerp(const std::vector<double>&                         base_keys,
                                                  const std::vector<geometry_msgs::msg::Quaternion>& base_values,
                                                  const std::vector<double>&                         query_keys);
}   // namespace interpolation

#endif   // INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_
