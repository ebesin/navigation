/*
 * @Author       : dwayne
 * @Date         : 2023-06-24
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_

#include "vehicle_info_util/vehicle_info.hpp"

#include <rclcpp/rclcpp.hpp>

namespace vehicle_info_util {
/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters.
/// This class supposes that necessary parameters are set when the node is launched.
class VehicleInfoUtil
{
public:
    /// Constructor
    explicit VehicleInfoUtil(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node, std::string name);

    /// Get vehicle info
    VehicleInfo getVehicleInfo();

private:
    /// Buffer for base parameters
    VehicleInfo vehicle_info_;
};

}   // namespace vehicle_info_util

#endif   // VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_
