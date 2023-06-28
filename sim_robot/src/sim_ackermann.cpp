/*
 * @Author       : dwayne
 * @Date         : 2023-06-26
 * @LastEditTime : 2023-06-28
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "sim_ackermann.hpp"
namespace sim_robot {

SimAckermann::SimAckermann(std::string name)
    : Node(name)
{
    declareParameter();
    /*get parameters*/
    origin_x_       = get_parameter("origin_x").as_double();
    origin_y_       = get_parameter("origin_y").as_double();
    origin_phi_     = get_parameter("origin_phi").as_double();
    pub_period_     = get_parameter("pub_period").as_double();
    min_sim_time_   = get_parameter("min_sim_time").as_double();
    wheel_base_     = get_parameter("wheel_base").as_double();
    cmd_sub_topic_  = get_parameter("cmd_sub_topic").as_string();
    odom_pub_toipc_ = get_parameter("odom_pub_toipc").as_string();

    /*initialize robot current status and simulator*/
    current_state_ptr_ = std::make_shared<sim_robot::BicycleKinematics::State>(origin_x_, origin_y_, origin_phi_);
    current_cmd_ptr_   = std::make_shared<geometry_msgs::msg::Twist>();
    simulator_ptr_ = std::make_shared<sim_robot::BicycleKinematics>(wheel_base_, rclcpp::Duration::from_seconds(min_sim_time_));

    /*initialize subscriber and publisher*/
    cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&SimAckermann::cmdCallback, this, std::placeholders::_1));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());

    current_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>("/mpc_controller/input/current_odometry", rclcpp::SystemDefaultsQoS());
    current_accel_publisher_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "/mpc_controller/input/current_accel", rclcpp::SystemDefaultsQoS());
    current_operation_mode_publisher_ = create_publisher<mpc_msgs::msg::OperationModeState>(
        "/mpc_controller/input/current_operation_mode", rclcpp::SystemDefaultsQoS());
    current_steering_publisher_ =
        create_publisher<mpc_msgs::msg::SteeringReport>("/mpc_controller/input/current_steering", rclcpp::SystemDefaultsQoS());

    /*initialize timer*/
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(pub_period_));
    pub_timer            = create_wall_timer(period_ns, std::bind(&SimAckermann::timerCallback, this));
}

SimAckermann::~SimAckermann() {}


void SimAckermann::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
{
    if (std::abs(cmd->angular.z) > 0.4363) cmd->angular.z = cmd->angular.z > 0 ? 0.4363 : -0.4363;

    current_cmd_ptr_ = cmd;
    if (!is_init_) {
        last_time_ = get_clock()->now();
        is_init_   = true;
        return;
    }
    current_time_                  = get_clock()->now();
    rclcpp::Duration time_interval = current_time_ - last_time_;
    if (time_interval > rclcpp::Duration::from_seconds(0.1)) {
        RCLCPP_WARN(get_logger(), "%fs has passed since the last command was received", time_interval.seconds());
        last_time_ = get_clock()->now();
        return;
    }
    simulator_ptr_->calKinematics(
        *current_state_ptr_, sim_robot::BicycleKinematics::CtrlInput(cmd->linear.x, cmd->angular.z), time_interval);
    last_time_ = current_time_;
}

void SimAckermann::timerCallback()
{
    // RCLCPP_INFO(get_logger(), "timer callback");
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id       = "map";
    odom.child_frame_id        = "odom";
    odom.pose.pose.position.x  = current_state_ptr_->x_;
    odom.pose.pose.position.y  = current_state_ptr_->y_;
    odom.pose.pose.orientation = createQuaternionMsgFromYaw(current_state_ptr_->phi_);
    odom.twist.twist           = *current_cmd_ptr_;
    odom.twist.twist.angular.z = current_cmd_ptr_->linear.x / wheel_base_ * tan(current_cmd_ptr_->angular.z);
    // RCLCPP_INFO_STREAM(get_logger(),
    //                    "x: " << current_state_ptr_->x_ << " y:" << current_state_ptr_->y_ << " phi:" << current_state_ptr_->phi_);
    odom_publisher_->publish(odom);
    current_odom_publisher_->publish(odom);
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    current_accel_publisher_->publish(accel);
    mpc_msgs::msg::OperationModeState oms;
    oms.is_autonomous_mode_available = true;
    current_operation_mode_publisher_->publish(oms);
    mpc_msgs::msg::SteeringReport sr;
    sr.steering_tire_angle = current_cmd_ptr_->angular.z;
    current_steering_publisher_->publish(sr);
}

geometry_msgs::msg::Quaternion SimAckermann::createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

void SimAckermann::declareParameter()
{
    declare_parameter("origin_x", 0.0);
    declare_parameter("origin_y", 0.0);
    declare_parameter("origin_phi", 0.0);
    declare_parameter("pub_period", 0.02);
    declare_parameter("min_sim_time", 0.001);
    declare_parameter("wheel_base", 0.65);
    declare_parameter("cmd_sub_topic", "cmd_vel");
    declare_parameter("odom_pub_toipc", "odom");
}

}   // namespace sim_robot