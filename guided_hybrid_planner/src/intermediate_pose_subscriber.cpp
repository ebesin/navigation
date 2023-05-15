/*
 * @Author       : dwayne
 * @Date         : 2023-04-25
 * @LastEditTime : 2023-04-25
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#include "intermediate_pose_subscriber.hpp"

namespace guided_hybrid_a_star {
IntermeidatePoseSubscriber::IntermeidatePoseSubscriber(std::string sub_topic_name, const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("intermeidate_pose_subscriber", "", options)
{
    RCLCPP_INFO(get_logger(), "Creating intermeidate_pose_subscriber...");
    this->sub_topic_name_ = sub_topic_name;
}

nav2_util::CallbackReturn IntermeidatePoseSubscriber::on_configure(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Configuring");
    intermeidate_pose_subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>(
        sub_topic_name_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&IntermeidatePoseSubscriber::messageCallBack, this, std::placeholders::_1));
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn IntermeidatePoseSubscriber::on_activate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Activating");
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn IntermeidatePoseSubscriber::on_deactivate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "DeActivating");
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn IntermeidatePoseSubscriber::on_cleanup(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Cleaning");
    intermeidate_pose_subscriber_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn IntermeidatePoseSubscriber::on_shutdown(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), " Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void IntermeidatePoseSubscriber::messageCallBack(const geometry_msgs::msg::PointStamped::SharedPtr point)
{
    buff_mutex_.lock();
    intermediate_points_.push_back(point);
    RCLCPP_INFO_STREAM(get_logger(),
                       "receive_intermeidate_pose------>"
                           << "x: " << point->point.x << " y: " << point->point.y << " z: " << point->point.z);

    buff_mutex_.unlock();
}

void IntermeidatePoseSubscriber::getIntermeidatePose(
    std::vector<geometry_msgs::msg::PointStamped::SharedPtr>& intermediate_points)
{
    for (int i = 0; i < intermediate_points_.size(); i++) {
        intermediate_points.push_back(intermediate_points_[i]);
    }
    intermediate_points_.clear();
}

}   // namespace guided_hybrid_a_star