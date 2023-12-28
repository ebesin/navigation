/*
 * @Author       : dwayne
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-04-22
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#include "goal_pose_subscriber.hpp"

#include <memory>

#include "coor_tools.h"

namespace guided_hybrid_a_star {

GoalPoseSubscriber2D::GoalPoseSubscriber2D(std::string sub_topic_name,
                                           const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("init_pose_subscriber", "", options) {
  RCLCPP_INFO(get_logger(), "Creating Goal_Pose_Subscriber...");
  this->sub_topic_name_ = sub_topic_name;
}

nav2_util::CallbackReturn GoalPoseSubscriber2D::on_configure(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Configuring");
  goal_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      sub_topic_name_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GoalPoseSubscriber2D::messageCallBack, this,
                std::placeholders::_1));
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GoalPoseSubscriber2D::on_activate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Activating");
  geometry_msgs::msg::PoseStamped::SharedPtr pose =
      std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose->pose.position.x = 16.7137;
  pose->pose.position.y = 9.66026;
  pose->pose.orientation = utils_tool::createQuaternionMsgFromYaw(1.57);
  goal_poses_.emplace_back(pose);
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GoalPoseSubscriber2D::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "DeActivating");
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GoalPoseSubscriber2D::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Cleaning");
  goal_pose_subscriber_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GoalPoseSubscriber2D::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), " Shutdown");
  return nav2_util::CallbackReturn::SUCCESS;
}

void GoalPoseSubscriber2D::messageCallBack(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ptr) {
  buff_mutex_.lock();
  goal_poses_.emplace_back(goal_pose_ptr);
  RCLCPP_INFO_STREAM(get_logger(),
                     "receive_goal_pose------>"
                         << "x: " << goal_pose_ptr->pose.position.x
                         << " y: " << goal_pose_ptr->pose.position.y
                         << " z: " << goal_pose_ptr->pose.position.z);
  buff_mutex_.unlock();
}

void GoalPoseSubscriber2D::parseData(
    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr>& pose_data_buff) {
  buff_mutex_.lock();
  if (!goal_poses_.empty()) {
    pose_data_buff.insert(pose_data_buff.end(), goal_poses_.begin(),
                          goal_poses_.end());
    goal_poses_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace guided_hybrid_a_star
