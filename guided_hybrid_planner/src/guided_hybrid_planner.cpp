/*
 * @Author       : dwayne
 * @Date         : 2023-04-08
 * @LastEditTime : 2023-04-25
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#include "guided_hybrid_planner.hpp"

namespace guided_hybrid_a_star {

GuidedHybridPlanner::GuidedHybridPlanner() {}

GuidedHybridPlanner::~GuidedHybridPlanner() {}

void GuidedHybridPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ = costmap_ros->getCostmap();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();
  RCLCPP_INFO(logger_, "Configuring %s of type GuidedHybridPlanner",
              name.c_str());

  nav2_util::declare_parameter_if_not_declared(node, name + ".max_steer_angle",
                                               rclcpp::ParameterValue(27.0));
  node->get_parameter(name + ".max_steer_angle", serch_info_.max_steer_angle);

  nav2_util::declare_parameter_if_not_declared(node, name + ".wheel_base",
                                               rclcpp::ParameterValue(0.65));
  node->get_parameter(name + ".wheel_base", serch_info_.wheel_base);

  nav2_util::declare_parameter_if_not_declared(node, name + ".max_iterations",
                                               rclcpp::ParameterValue(1000000));
  node->get_parameter(name + ".max_iterations", serch_info_.max_iterations);

  nav2_util::declare_parameter_if_not_declared(node, name + ".travel_unknown",
                                               rclcpp::ParameterValue(false));
  node->get_parameter(name + ".travel_unknown", serch_info_.travel_unknown);

  nav2_util::declare_parameter_if_not_declared(node, name + ".shot_distance",
                                               rclcpp::ParameterValue(3.0));
  node->get_parameter(name + ".shot_distance", serch_info_.shot_distance);

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".angle_segment_size", rclcpp::ParameterValue(72));
  node->get_parameter(name + ".angle_segment_size",
                      serch_info_.angle_segment_size);

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".steer_angle_segment_size", rclcpp::ParameterValue(10));
  node->get_parameter(name + ".steer_angle_segment_size",
                      serch_info_.steer_angle_segment_size);

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".move_step_distance", rclcpp::ParameterValue(1.414));
  node->get_parameter(name + ".move_step_distance",
                      serch_info_.move_step_distance);

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".allow_reverse_expansion", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_reverse_expansion",
                      serch_info_.allow_reverse_expansion);

  nav2_util::declare_parameter_if_not_declared(node, name + ".reverse_penalty",
                                               rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".reverse_penalty", serch_info_.reverse_penalty);

  nav2_util::declare_parameter_if_not_declared(node, name + ".steering_penalty",
                                               rclcpp::ParameterValue(1.05));
  node->get_parameter(name + ".steering_penalty", serch_info_.steering_penalty);

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".steering_change_penalty", rclcpp::ParameterValue(1.5));
  node->get_parameter(name + ".steering_change_penalty",
                      serch_info_.steering_change_penalty);

  nav2_util::declare_parameter_if_not_declared(node, name + ".cost_penalty",
                                               rclcpp::ParameterValue(2.0));
  node->get_parameter(name + ".cost_penalty", serch_info_.cost_penalty);

  if (serch_info_.max_iterations < 0) {
    RCLCPP_WARN(logger_, "最大迭代次数<0,将重新设置为最大值");
    serch_info_.max_iterations = std::numeric_limits<int>::max();
  }

  auto angel2Rad = [](double angle) { return angle / 180 * M_PI; };

  serch_info_.max_steer_angle = angel2Rad(serch_info_.max_steer_angle);
  serch_info_.shot_distance =
      serch_info_.shot_distance / costmap_->getResolution();
  serch_info_.move_step_distance =
      serch_info_.move_step_distance / costmap_->getResolution();

  collision_checker_ = std::make_shared<GridCollisionChecker>(
      costmap_, serch_info_.angle_segment_size);

  guided_hybrid_a_star_ =
      std::make_unique<GuidedHybridAStar>(costmap_, serch_info_);

  plan_publisher_ =
      node->create_publisher<nav_msgs::msg::Path>("origin_path", 1);
}

void GuidedHybridPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating plugin %s of type GuidedHybridPlanner",
              name_.c_str());
  plan_publisher_->on_activate();
}

void GuidedHybridPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type GuidedHybridPlanner",
              name_.c_str());
  plan_publisher_->on_deactivate();
}

void GuidedHybridPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type GuidedHybridPlanner",
              name_.c_str());
  plan_publisher_.reset();
}

nav_msgs::msg::Path
GuidedHybridPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) {
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  guided_hybrid_a_star_->setCollisionChecker(collision_checker_);

  unsigned int mx, my;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  guided_hybrid_a_star_->setStart(
      Vec3d(mx, my, tf2::getYaw(start.pose.orientation)));

  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  guided_hybrid_a_star_->setGoal(
      Vec3d(mx, my, tf2::getYaw(goal.pose.orientation)));

  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  VectorVec3d path;
  std::string error;
  try {
    if (!guided_hybrid_a_star_->computePath()) {
      std::cout << "---------error---------" << std::endl;
      return plan;
    }
  } catch (const std::runtime_error &e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (guided_hybrid_a_star_->backtracePath(path)) {
    plan.poses.reserve(path.size());
    for (int i = path.size() - 1; i >= 0; --i) {
      pose.pose.position.x = static_cast<float>(costmap_->getOriginX()) +
                             (path[i].x() + 0.5) * costmap_->getResolution();
      pose.pose.position.y = static_cast<float>(costmap_->getOriginX()) +
                             (path[i].y() + 0.5) * costmap_->getResolution();
      tf2::Quaternion q;
      q.setEuler(0.0, 0.0, path[i].z());
      pose.pose.orientation = tf2::toMsg(q);
      plan.poses.push_back(pose);
    }
  } else {
    std::cout << "---------error2---------" << std::endl;
  }

  if (plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(plan);
  }

  return plan;
}
} // namespace guided_hybrid_a_star

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(guided_hybrid_a_star::GuidedHybridPlanner,
                       nav2_core::GlobalPlanner)
