/*
 * @Author       : dwayne
 * @Date         : 2023-06-27
 * @LastEditTime : 2023-06-30
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#pragma once

#include <string>

#include "coor_tools.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace utils_tool {
class VisualizationTools : public rclcpp::Node {
 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;
  /*odom marker and subscriber*/
  visualization_msgs::msg::MarkerArray markers_;
  visualization_msgs::msg::Marker odom_robot_marker_;
  visualization_msgs::msg::Marker odom_text_marker_;
  // visualization_msgs::msg::Marker                          path_marker_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  // rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_subscriber_;

  /*parameters*/
  bool isVisualizationOdom_;
  std::string odom_topic_name_;

  /*callback functions*/
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

  /*declare and get parameters*/
  void declareAndGetParameters();

  /*init markers*/
  void initMarkers();

 public:
  VisualizationTools(std::string name);
  ~VisualizationTools();
};

}  // namespace utils_tool
