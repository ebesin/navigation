/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
#define MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mpc_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>

namespace motion_utils {

using mpc_msgs::msg::Trajectory;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

class VehicleStopCheckerBase
{
public:
    VehicleStopCheckerBase(rclcpp::Node* node, double buffer_duration);
    rclcpp::Logger getLogger() { return logger_; }
    void           addTwist(const TwistStamped& twist);
    bool           isVehicleStopped(const double stop_duration = 0.0) const;

protected:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger           logger_;

private:
    double                   buffer_duration_;
    std::deque<TwistStamped> twist_buffer_;
};

class VehicleStopChecker : public VehicleStopCheckerBase
{
public:
    explicit VehicleStopChecker(rclcpp::Node* node);

protected:
    rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
    Odometry::SharedPtr                       odometry_ptr_;

private:
    static constexpr double velocity_buffer_time_sec = 10.0;
    void                    onOdom(const Odometry::SharedPtr msg);
};

class VehicleArrivalChecker : public VehicleStopChecker
{
public:
    explicit VehicleArrivalChecker(rclcpp::Node* node);

    bool isVehicleStoppedAtStopPoint(const double stop_duration = 0.0) const;

private:
    static constexpr double th_arrived_distance_m = 1.0;

    rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

    Trajectory::SharedPtr trajectory_ptr_;

    void onTrajectory(const Trajectory::SharedPtr msg);
};
}   // namespace motion_utils

#endif   // MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
