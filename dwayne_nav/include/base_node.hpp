/*
 * @Author       : dwayne
 * @Date         : 2023-03-11
 * @LastEditTime : 2023-04-22
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef BASE_NODE_HPP_
#define BASE_NODE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <memory>
#include <string>

namespace base_node {
class BaseNode : public nav2_util::LifecycleNode
{
public:
    /**
     * @description  : constructor
     * @param         {NodeOptions&} options:
     * @return        {*}
     */
    BaseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~BaseNode();

protected:
    /**
     * @description  : Configure member variables and initializes base_node
     * @return        {*} SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

    /**
     * @description  : Activate member variables
     * @return        {*} SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

    /**
     * @description  : Dectivate member variables
     * @return        {*} SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

    /**
     * @description  : Reset member variables
     * @return        {*} SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

    /**
     * @description  : Called when in shutdown state
     * @return        {*} SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

    void carPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr pose);

    // topic to subscribe
    std::string car_topic_;
    // odom topic to publish
    std::string odom_topic_;
    // subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_sub_;
    // Publishers for the odom
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>                     odom_map_tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                           base_link_odom_transform_broadcaster_;
};

}   // namespace base_node

#endif