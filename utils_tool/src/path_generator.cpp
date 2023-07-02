/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-06-30
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "path_generator.hpp"

namespace utils_tool {
PathGenerator::PathGenerator(std::string name)
    : Node(name)
{
    get_parameter_or("traj_type", traj_type_, std::string("line"));
    get_parameter_or("resulation", resulation_, 0.1);
    get_parameter_or("velocity", velocity_, 0.5);
    get_parameter_or("wheel_base", wheel_base_, 0.65);

    get_parameter_or("path_length", path_length_, 20.0);
    get_parameter_or("path_heading", path_heading_, 0.0);
    trajectory_publisher_ =
        create_publisher<Trajectory>("/mpc_controller/input/reference_trajectory", rclcpp::SystemDefaultsQoS());
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/generate_path", rclcpp::SystemDefaultsQoS());
    if (traj_type_ == std::string("line")) {
        generateLineCurve(path_length_, path_heading_);
    }
    else {
        RCLCPP_ERROR(get_logger(), "no such type of curve!!!");
    }
}

PathGenerator::~PathGenerator() {}

void PathGenerator::perfectPath(Trajectory& traj, double velocity, double wheel_base)
{
    for (int i = 0; i < traj.points.size() - 1; i++) {
        traj.points[i].longitudinal_velocity_mps = velocity;
        traj.points[i].acceleration_mps2         = 0;
        traj.points[i].time_from_start           = rclcpp::Duration::from_seconds(i * resulation_ / velocity);
        traj.points[i].rear_wheel_angle_rad      = 0;
        //前轮转角计算
        double delta_theta =
            getYawFromQuaternion(traj.points[i + 1].pose.orientation) - getYawFromQuaternion(traj.points[i].pose.orientation);
        if (delta_theta > M_PI)
            delta_theta = delta_theta - 2 * M_PI;
        else if (delta_theta < -M_PI)
            delta_theta = delta_theta + 2 * M_PI;
        double tan_theta                     = wheel_base * delta_theta / resulation_;
        traj.points[i].front_wheel_angle_rad = atan(tan_theta);
        traj.points[i].heading_rate_rps      = velocity * tan_theta / wheel_base;
    }
    traj.points[traj.points.size() - 1].longitudinal_velocity_mps = 0;
    traj.points[traj.points.size() - 1].acceleration_mps2         = 0;
    traj.points[traj.points.size() - 1].time_from_start =
        rclcpp::Duration::from_seconds((traj.points.size() - 1) * resulation_ / velocity);
    traj.points[traj.points.size() - 1].rear_wheel_angle_rad  = 0;
    traj.points[traj.points.size() - 1].front_wheel_angle_rad = 0;
    traj.points[traj.points.size() - 1].heading_rate_rps      = 0;
}

void PathGenerator::generateSinCurve() {}

void PathGenerator::generateLineCurve(double length, double heading)
{
    Trajectory          traj;
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    int size             = static_cast<int>(round(length / resulation_));
    RCLCPP_INFO(get_logger(), "path point size:%d", size);
    double vec_i = cos(heading);
    double vec_j = sin(heading);
    traj.points.resize(size + 1);
    path.poses.resize(size + 1);
    traj.points[0].pose.position.x = 0;
    traj.points[0].pose.position.y = 0;
    for (int i = 1; i <= size; i++) {
        traj.points[i].pose.position.x = i * resulation_ * vec_i;
        traj.points[i].pose.position.y = i * resulation_ * vec_j;
        traj.points[i - 1].pose.orientation =
            createQuaternionMsgFromYaw(getHeadingFromVector(traj.points[i].pose.position.x - traj.points[i - 1].pose.position.x,
                                                            traj.points[i].pose.position.y - traj.points[i - 1].pose.position.y));
        path.poses[i].pose = traj.points[i].pose;
    }
    traj.points[size].pose.orientation = traj.points[size - 1].pose.orientation;
    perfectPath(traj, velocity_, wheel_base_);
    RCLCPP_INFO_STREAM(get_logger(),
                       "last_x:" << traj.points[size].pose.position.x << "  last_y:" << traj.points[size].pose.position.y);
    trajectory_publisher_->publish(traj);
    path_publisher_->publish(path);
}

void PathGenerator::generateCircleCurve() {}
}   // namespace utils_tool