/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-07-02
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "path_generator.hpp"

namespace utils_tool {
PathGenerator::PathGenerator(std::string name)
    : Node(name)
{
    declare_parameter("traj_type", "line");
    declare_parameter("resulation", 0.1);
    declare_parameter("velocity", 0.5);
    declare_parameter("wheel_base", 0.65);

    get_parameter_or("traj_type", traj_type_, std::string("line"));
    get_parameter_or("resulation", resulation_, 0.1);
    get_parameter_or("velocity", velocity_, 0.5);
    get_parameter_or("wheel_base", wheel_base_, 0.65);


    trajectory_publisher_ =
        create_publisher<Trajectory>("/mpc_controller/input/reference_trajectory", rclcpp::SystemDefaultsQoS());
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/generate_path", rclcpp::SystemDefaultsQoS());
    if (traj_type_ == std::string("line")) {
        declare_parameter("path_length", 20.0);
        declare_parameter("path_heading", 0.0);
        path_length_  = get_parameter("path_length").as_double();
        path_heading_ = get_parameter("path_heading").as_double();
        generateLineCurve(path_length_, path_heading_);
    }
    else if (traj_type_ == std::string("sin")) {
        declare_parameter("path_length", 20.0);
        declare_parameter("path_heading", 0.0);
        declare_parameter("sin_cycle", 4 * M_PI);
        declare_parameter("sin_amplitude", 4.0);
        path_length_  = get_parameter("path_length").as_double();
        path_heading_ = get_parameter("path_heading").as_double();
        cycle_        = get_parameter("sin_cycle").as_double();
        amplitude_    = get_parameter("sin_amplitude").as_double();
        generateSinCurve(path_length_, path_heading_, cycle_, amplitude_);
    }
    else {
        RCLCPP_ERROR(get_logger(), "no such type of curve!!!");
    }
}

PathGenerator::~PathGenerator() {}

void PathGenerator::perfectPath(Trajectory& traj, double velocity, double wheel_base)
{
    auto getDistance = [](geometry_msgs::msg::Point a, geometry_msgs::msg::Point b) {
        return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    };
    rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);
    for (int i = 0; i < traj.points.size() - 1; i++) {
        traj.points[i].longitudinal_velocity_mps = velocity;
        traj.points[i].acceleration_mps2         = 0;
        traj.points[i].time_from_start           = duration;
        duration                                 = duration + rclcpp::Duration::from_seconds(
                                  getDistance(traj.points[i].pose.position, traj.points[i + 1].pose.position) / velocity);

        traj.points[i].rear_wheel_angle_rad = 0;
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
    traj.points[traj.points.size() - 1].time_from_start           = duration;
    traj.points[traj.points.size() - 1].rear_wheel_angle_rad      = 0;
    traj.points[traj.points.size() - 1].front_wheel_angle_rad     = 0;
    traj.points[traj.points.size() - 1].heading_rate_rps          = 0;
}

void PathGenerator::generateSinCurve(double length, double heading, double cycle, double amplitude)
{
    Trajectory          traj;
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    int size             = static_cast<int>(round(length / resulation_));
    RCLCPP_INFO(get_logger(), "path point size:%d", size);
    auto sinCurve = [](double cycle, double amplitude, double x) {
        double omega = 2 * M_PI / cycle;
        return amplitude * sin(omega * x);
    };
    auto sinCurveDerivative = [](double cycle, double amplitude, double x) {
        double omega = 2 * M_PI / cycle;
        return amplitude * omega * cos(omega * x);
    };
    traj.points.resize(size + 1);
    path.poses.resize(size + 1);
    traj.points[0].pose.position.x = 0;
    traj.points[0].pose.position.y = 0;
    path.poses[0].pose             = traj.points[0].pose;
    for (int i = 1; i <= size; i++) {
        double x_ori = i * resulation_;
        double y_ori = sinCurve(cycle, amplitude, i * resulation_);
        traj.points[i - 1].pose.orientation =
            createQuaternionMsgFromYaw(atan(sinCurveDerivative(cycle, amplitude, i * resulation_)) + heading);
        traj.points[i].pose.position.x = cos(heading) * x_ori - sin(heading) * y_ori;
        traj.points[i].pose.position.y = sin(heading) * x_ori + cos(heading) * y_ori;
        path.poses[i].pose             = traj.points[i].pose;
    }
    traj.points[size].pose.orientation = traj.points[size - 1].pose.orientation;
    perfectPath(traj, velocity_, wheel_base_);
    traj_ = traj;
    path_ = path;
    // trajectory_publisher_->publish(traj);
    // path_publisher_->publish(path);
    startTimer();
}

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
    path.poses[0].pose             = traj.points[0].pose;
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
    traj_ = traj;
    path_ = path;
    // trajectory_publisher_->publish(traj);
    // path_publisher_->publish(path);
    startTimer();
}

void PathGenerator::generateCircleCurve() {}
void PathGenerator::timerCallback()
{
    trajectory_publisher_->publish(traj_);
    path_publisher_->publish(path_);
}

void PathGenerator::startTimer()
{
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));
    pub_timer_           = create_wall_timer(period_ns, std::bind(&PathGenerator::timerCallback, this));
}
}   // namespace utils_tool
