/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_

#include "mpc_utils/geometry/geometry.hpp"
#include "mpc_utils/math/normalization.hpp"

#include <tf2/utils.h>

namespace mpc_utils
{
struct PoseDeviation
{
  double lateral{0.0};
  double longitudinal{0.0};
  double yaw{0.0};
};

inline double calcLateralDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Eigen::Vector3d base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Eigen::Vector3d diff_vec{dx, dy, 0};

  const Eigen::Vector3d cross_vec = base_unit_vec.cross(diff_vec);

  return cross_vec.z();
}

inline double calcLongitudinalDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Eigen::Vector3d base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Eigen::Vector3d diff_vec{dx, dy, 0};

  return base_unit_vec.dot(diff_vec);
}

inline double calcYawDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  const auto base_yaw = tf2::getYaw(base_pose.orientation);
  const auto target_yaw = tf2::getYaw(target_pose.orientation);
  return normalizeRadian(target_yaw - base_yaw);
}

inline PoseDeviation calcPoseDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  PoseDeviation deviation{};

  deviation.lateral = calcLateralDeviation(base_pose, target_pose.position);
  deviation.longitudinal = calcLongitudinalDeviation(base_pose, target_pose.position);
  deviation.yaw = calcYawDeviation(base_pose, target_pose);

  return deviation;
}
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_
