/*
 * @Author       : dwayne
 * @Date         : 2023-06-24
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_

#include "mpc_utils/mpc_utils.hpp"

namespace vehicle_info_util {
/// Data class for vehicle info
struct VehicleInfo
{
    // Base parameters. These describe the vehicle's bounding box and the
    // position and radius of the wheels.
    double wheel_radius_m;
    double wheel_width_m;
    double wheel_base_m;
    double wheel_tread_m;
    double front_overhang_m;
    double rear_overhang_m;
    double left_overhang_m;
    double right_overhang_m;
    double vehicle_height_m;
    double max_steer_angle_rad;

    // Derived parameters, i.e. calculated from base parameters
    // The offset values are relative to the base frame origin, which is located
    // on the ground below the middle of the rear axle, and can be negative.
    double vehicle_length_m;
    double vehicle_width_m;
    double min_longitudinal_offset_m;
    double max_longitudinal_offset_m;
    double min_lateral_offset_m;
    double max_lateral_offset_m;
    double min_height_offset_m;
    double max_height_offset_m;

    mpc_utils::LinearRing2d createFootprint(const double margin = 0.0) const { return createFootprint(margin, margin); }

    mpc_utils::LinearRing2d createFootprint(const double lat_margin, const double lon_margin) const
    {
        using mpc_utils::LinearRing2d;
        using mpc_utils::Point2d;

        const double x_front  = front_overhang_m + wheel_base_m + lon_margin;
        const double x_center = wheel_base_m / 2.0;
        const double x_rear   = -(rear_overhang_m + lon_margin);
        const double y_left   = wheel_tread_m / 2.0 + left_overhang_m + lat_margin;
        const double y_right  = -(wheel_tread_m / 2.0 + right_overhang_m + lat_margin);

        LinearRing2d footprint;
        footprint.push_back(Point2d{x_front, y_left});
        footprint.push_back(Point2d{x_front, y_right});
        footprint.push_back(Point2d{x_center, y_right});
        footprint.push_back(Point2d{x_rear, y_right});
        footprint.push_back(Point2d{x_rear, y_left});
        footprint.push_back(Point2d{x_center, y_left});
        footprint.push_back(Point2d{x_front, y_left});

        return footprint;
    }
};

/// Create vehicle info from base parameters
inline VehicleInfo createVehicleInfo(const double wheel_radius_m,
                                     const double wheel_width_m,
                                     const double wheel_base_m,
                                     const double wheel_tread_m,
                                     const double front_overhang_m,
                                     const double rear_overhang_m,
                                     const double left_overhang_m,
                                     const double right_overhang_m,
                                     const double vehicle_height_m,
                                     const double max_steer_angle_rad)
{
    // Calculate derived parameters
    const double vehicle_length_m_          = front_overhang_m + wheel_base_m + rear_overhang_m;
    const double vehicle_width_m_           = wheel_tread_m + left_overhang_m + right_overhang_m;
    const double min_longitudinal_offset_m_ = -rear_overhang_m;
    const double max_longitudinal_offset_m_ = front_overhang_m + wheel_base_m;
    const double min_lateral_offset_m_      = -(wheel_tread_m / 2.0 + right_overhang_m);
    const double max_lateral_offset_m_      = wheel_tread_m / 2.0 + left_overhang_m;
    const double min_height_offset_m_       = 0.0;
    const double max_height_offset_m_       = vehicle_height_m;

    return VehicleInfo{
        // Base parameters
        wheel_radius_m,
        wheel_width_m,
        wheel_base_m,
        wheel_tread_m,
        front_overhang_m,
        rear_overhang_m,
        left_overhang_m,
        right_overhang_m,
        vehicle_height_m,
        max_steer_angle_rad,
        // Derived parameters
        vehicle_length_m_,
        vehicle_width_m_,
        min_longitudinal_offset_m_,
        max_longitudinal_offset_m_,
        min_lateral_offset_m_,
        max_lateral_offset_m_,
        min_height_offset_m_,
        max_height_offset_m_,
    };
}
}   // namespace vehicle_info_util

#endif   // VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
