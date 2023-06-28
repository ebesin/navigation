/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-06-28
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
    trajectory_publisher_ =
        create_publisher<Trajectory>("/mpc_controller/input/reference_trajectory", rclcpp::SystemDefaultsQoS());
}

PathGenerator::~PathGenerator() {}

void PathGenerator::perfectPath(Trajectory& traj, double velocity)
{
    for (int i = 0; i < traj.points.size() - 1; i++) {
        traj.points[i].pose.orientation =
            createQuaternionMsgFromYaw(getHeadingFromVector(traj.points[i + 1].pose.position.x - traj.points[i].pose.position.x,
                                                            traj.points[i + 1].pose.position.y - traj.points[i].pose.position.y));
        traj.points[i].longitudinal_velocity_mps = velocity;
        traj.points[i].time_from_start           = rclcpp::Duration::from_seconds(i * resulation_ / velocity);
    }
}

void PathGenerator::generateSinCurve() {}

void PathGenerator::generateLineCurve(double length, double heading)
{
    int    size  = static_cast<int>(round(length / resulation_));
    double vec_i = cos(heading);
    double vec_j = sin(heading);
    traj_.points.resize(size + 1);
    for (int i = 0; i <= size; i++) {
        traj_.points[i].pose.position.x = i * resulation_ * vec_i;
        traj_.points[i].pose.position.y = i * resulation_ + vec_j;
    }
}

void PathGenerator::generateCircleCurve() {}
}   // namespace utils_tool