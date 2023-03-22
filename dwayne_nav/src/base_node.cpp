/*
 * @Author       : iPEK
 * @Date         : 2023-03-11
 * @LastEditTime : 2023-03-16
 * @Description  : 
 * 
 * Copyright (c) 2023 by iPEK, All Rights Reserved. 
 */
#include "base_node.hpp"


namespace base_node {
BaseNode::BaseNode(const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("base_node", "", options)
{
    RCLCPP_INFO(get_logger(), "Creating Base Node...");
    declare_parameter("sub_topic", "/car/pose");
    declare_parameter("odom_topic", "odom");
    get_parameter("sub_topic", car_topic_);
    get_parameter("odom_topic", odom_topic_);
}

BaseNode::~BaseNode() {}


nav2_util::CallbackReturn BaseNode::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Configuring");
    car_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        car_topic_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&BaseNode::carPoseCallback, this, std::placeholders::_1));

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 1);

    odom_map_tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

    base_link_odom_transform_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BaseNode::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Activating");
    odom_publisher_->on_activate();
    createBond();
    // makeStatisTransform();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BaseNode::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "DeActivating");
    odom_publisher_->on_deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BaseNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Cleaning");
    car_pose_sub_.reset();
    odom_publisher_.reset();
    odom_map_tf_static_broadcaster_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BaseNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), " Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void BaseNode::carPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    // RCLCPP_DEBUG(get_logger(), "car_pose received");
    auto odom             = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.frame_id = "odom";
    odom->pose.pose       = pose->pose;
    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.stamp            = this->get_clock()->now();
    tmp_tf_stamped.header.frame_id         = "odom";
    tmp_tf_stamped.child_frame_id          = " base_footprint";
    tmp_tf_stamped.transform.translation.x = pose->pose.position.x;
    tmp_tf_stamped.transform.translation.y = pose->pose.position.y;
    tmp_tf_stamped.transform.translation.z = pose->pose.position.z;
    tmp_tf_stamped.transform.rotation      = pose->pose.orientation;

    odom_publisher_->publish(std::move(odom));
    base_link_odom_transform_broadcaster_->sendTransform(tmp_tf_stamped);
}
}   // namespace base_node