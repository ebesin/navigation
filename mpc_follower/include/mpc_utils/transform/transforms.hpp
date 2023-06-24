/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_
#define TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

namespace mpc_utils {
template<typename PointT>
void transformPointCloud(const pcl::PointCloud<PointT>&    cloud_in,
                         pcl::PointCloud<PointT>&          cloud_out,
                         const Eigen::Matrix<float, 4, 4>& transform)
{
    if (cloud_in.empty() || cloud_in.width == 0) {
        RCLCPP_WARN(rclcpp::get_logger("transformPointCloud"), "input point cloud is empty!");
    }
    else {
        pcl::transformPointCloud(cloud_in, cloud_out, transform);
    }
}

template<typename PointT>
void transformPointCloud(const pcl::PointCloud<PointT>& cloud_in,
                         pcl::PointCloud<PointT>&       cloud_out,
                         const Eigen::Affine3f&         transform)
{
    if (cloud_in.empty() || cloud_in.width == 0) {
        RCLCPP_WARN(rclcpp::get_logger("transformPointCloud"), "input point cloud is empty!");
    }
    else {
        pcl::transformPointCloud(cloud_in, cloud_out, transform);
    }
}
}   // namespace mpc_utils

#endif   // TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_
