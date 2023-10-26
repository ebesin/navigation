/*
 * @Author       : dwayne
 * @Date         : 2023-04-21
 * @LastEditTime : 2023-06-27
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "visualization_tools.hpp"

namespace guided_hybrid_a_star {
VisualizationTools ::VisualizationTools(const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("visualization_tools", "", options)
{}

VisualizationTools ::~VisualizationTools() {}


nav2_util::CallbackReturn VisualizationTools::on_configure(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    serch_tree_publisher_ = create_publisher<visualization_msgs::msg::Marker>("serch_tree", 1);
    // odom_subscriber_      = create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", rclcpp::SystemDefaultsQoS(), std::bind(&VisualizationTools::odomCallback, this, std::placeholders::_1));
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VisualizationTools::on_activate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Activating");
    serch_tree_publisher_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VisualizationTools::on_deactivate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "DeActivating");
    serch_tree_publisher_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VisualizationTools::on_cleanup(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Cleaning");
    serch_tree_publisher_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VisualizationTools::on_shutdown(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), " Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void VisualizationTools::add_point_pair(Vec2d first, Vec2d second)
{
    geometry_msgs::msg::Point point;
    point.x = first.x();
    point.y = first.y();
    point.z = 0.0;
    tree_list_.points.emplace_back(point);
    point.x = second.x();
    point.y = second.y();
    point.z = 0.0;
    tree_list_.points.emplace_back(point);
}

void VisualizationTools::clear_markers()
{
    tree_list_.points.clear();
    tree_list_.header.frame_id = "map";
    tree_list_.header.stamp    = rclcpp::Clock().now();
    tree_list_.type            = visualization_msgs::msg::Marker::LINE_LIST;
    tree_list_.action          = visualization_msgs::msg::Marker::DELETEALL;
    tree_list_.ns              = "searched_tree";
    tree_list_.scale.x         = 0.005;
    serch_tree_publisher_->publish(tree_list_);
}

void VisualizationTools::publish_point_pairs()
{
    tree_list_.header.frame_id = "map";
    tree_list_.header.stamp    = rclcpp::Clock().now();
    tree_list_.type            = visualization_msgs::msg::Marker::LINE_LIST;
    tree_list_.action          = visualization_msgs::msg::Marker::ADD;
    tree_list_.ns              = "searched_tree";
    tree_list_.scale.x         = 0.005;

    tree_list_.color.a = 0.5;
    tree_list_.color.r = 1;
    tree_list_.color.g = 1;
    tree_list_.color.b = 0;

    tree_list_.pose.orientation.w = 1.0;
    tree_list_.pose.orientation.x = 0.0;
    tree_list_.pose.orientation.y = 0.0;
    tree_list_.pose.orientation.z = 0.0;

    serch_tree_publisher_->publish(tree_list_);
}

}   // namespace guided_hybrid_a_star