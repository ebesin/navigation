/*
 * @Author       : dwayne
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-06-28
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#include "init_pose_subscriber.hpp"

#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <memory>

#include "coor_tools.h"

namespace guided_hybrid_a_star {

InitPoseSubscriber2D::InitPoseSubscriber2D(std::string name,
                                           const std::string& parent_namespace,
                                           const std::string& local_namespace)
    : nav2_util::LifecycleNode(
          name, "",
          rclcpp::NodeOptions().arguments(
              {"--ros-args", "-r",
               std::string("__ns:=") +
                   nav2_util::add_namespaces(parent_namespace, local_namespace),
               "--ros-args", "-r",
               name + ":" + std::string("__node:=") + name})) {
  RCLCPP_INFO(get_logger(), "Creating Init_Pose_Subscriber...");
  this->name_ = name;
}

nav2_util::CallbackReturn InitPoseSubscriber2D::on_configure(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Configuring");
  init_pose_subscriber_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", rclcpp::SystemDefaultsQoS(),
          std::bind(&InitPoseSubscriber2D::messageCallBack, this,
                    std::placeholders::_1));
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InitPoseSubscriber2D::on_activate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Activating");
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose->pose.pose.position.x = -3.61773;
  pose->pose.pose.position.y = -2.75289;
  pose->pose.pose.orientation = utils_tool::createQuaternionMsgFromYaw(1.57);
  init_poses_.emplace_back(pose);
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InitPoseSubscriber2D::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "DeActivating");
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InitPoseSubscriber2D::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Cleaning");
  init_pose_subscriber_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn InitPoseSubscriber2D::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), " Shutdown");
  return nav2_util::CallbackReturn::SUCCESS;
}

void InitPoseSubscriber2D::messageCallBack(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        init_pose_ptr) {
  buff_mutex_.lock();
  init_poses_.emplace_back(init_pose_ptr);
  RCLCPP_INFO_STREAM(get_logger(),
                     "receive_init_pose------>"
                         << "x: " << init_pose_ptr->pose.pose.position.x
                         << " y: " << init_pose_ptr->pose.pose.position.y
                         << " z: " << init_pose_ptr->pose.pose.position.z);
  buff_mutex_.unlock();
}

void InitPoseSubscriber2D::parseData(
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>&
        pose_data_buff) {
  buff_mutex_.lock();
  if (!init_poses_.empty()) {
    pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(),
                          init_poses_.end());
    init_poses_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace guided_hybrid_a_star
