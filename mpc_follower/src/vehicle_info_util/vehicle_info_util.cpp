/*
 * @Author       : dwayne
 * @Date         : 2023-06-24
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "vehicle_info_util/vehicle_info_util.hpp"

#include <string>

namespace {
template<class T> T getParameter(rclcpp::Node& node, const std::string& name)
{
    if (node.has_parameter(name)) {
        return node.get_parameter(name).get_value<T>();
    }

    try {
        return node.declare_parameter<T>(name);
    }
    catch (const rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(node.get_logger(), "Failed to get parameter `%s`, please set it when you launch the node.", name.c_str());
        throw(ex);
    }
}
}   // namespace

namespace vehicle_info_util {
VehicleInfoUtil::VehicleInfoUtil(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node, std::string name)
{
    vehicle_info_.wheel_radius_m      = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.wheel_width_m       = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.wheel_base_m        = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.wheel_tread_m       = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.front_overhang_m    = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.rear_overhang_m     = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.left_overhang_m     = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.right_overhang_m    = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.vehicle_height_m    = node->get_parameter(name + ".wheel_radius").as_double();
    vehicle_info_.max_steer_angle_rad = node->get_parameter(name + ".wheel_radius").as_double();
    // node->get_parameter(name + ".wheel_radius", vehicle_info_.wheel_radius_m);
    // node->get_parameter(name + ".wheel_width", vehicle_info_.wheel_width_m);
    // node->get_parameter(name + ".wheel_base", vehicle_info_.wheel_base_m);
    // node->get_parameter(name + ".wheel_tread", vehicle_info_.wheel_tread_m);
    // node->get_parameter(name + ".front_overhang", vehicle_info_.front_overhang_m);
    // node->get_parameter(name + ".rear_overhang", vehicle_info_.rear_overhang_m);
    // node->get_parameter(name + ".left_overhang", vehicle_info_.left_overhang_m);
    // node->get_parameter(name + ".right_overhang", vehicle_info_.right_overhang_m);
    // node->get_parameter(name + ".vehicle_height", vehicle_info_.vehicle_height_m);
    // node->get_parameter(name + ".max_steer_angle", vehicle_info_.max_steer_angle_rad);
}

VehicleInfo VehicleInfoUtil::getVehicleInfo()
{
    return createVehicleInfo(vehicle_info_.wheel_radius_m,
                             vehicle_info_.wheel_width_m,
                             vehicle_info_.wheel_base_m,
                             vehicle_info_.wheel_tread_m,
                             vehicle_info_.front_overhang_m,
                             vehicle_info_.rear_overhang_m,
                             vehicle_info_.left_overhang_m,
                             vehicle_info_.right_overhang_m,
                             vehicle_info_.vehicle_height_m,
                             vehicle_info_.max_steer_angle_rad);
}
}   // namespace vehicle_info_util
