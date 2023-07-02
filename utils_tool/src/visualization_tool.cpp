/*
 * @Author       : dwayne
 * @Date         : 2023-06-27
 * @LastEditTime : 2023-06-30
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#include "visualization_tool.hpp"

namespace utils_tool {

VisualizationTools::VisualizationTools(std::string name)
    : Node(name)
{
    declareAndGetParameters();
    initMarkers();
    odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_, rclcpp::SystemDefaultsQoS(), std::bind(&VisualizationTools::odomCallback, this, std::placeholders::_1));
    // path_subscriber_ =
    marker_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", rclcpp::SystemDefaultsQoS());
}

VisualizationTools::~VisualizationTools() {}

void VisualizationTools::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{

    markers_.markers[0].pose = odom->pose.pose;
    std::string marker_text("linear_velocity:" + std::to_string(odom->twist.twist.linear.x) + "\n" +
                            "angular_velocity:" + std::to_string(odom->twist.twist.angular.z) + "\n" +
                            "heading:" + std::to_string(radian2Angle(getYawFromQuaternion(odom->pose.pose.orientation))));
    markers_.markers[1].text          = marker_text;
    markers_.markers[1].pose.position = odom->pose.pose.position;
    markers_.markers[1].pose.position.x -= 1;
    marker_publisher_->publish(markers_);
}

void VisualizationTools::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {}

void VisualizationTools::initMarkers()
{
    /*odom marker*/
    odom_robot_marker_.header.frame_id = "map";
    odom_robot_marker_.type            = visualization_msgs::msg::Marker::CUBE;
    odom_robot_marker_.action          = visualization_msgs::msg::Marker::MODIFY;
    odom_robot_marker_.ns              = std::string("utils_tool");
    odom_robot_marker_.id              = 0;

    odom_robot_marker_.scale.x = 0.978;   //长
    odom_robot_marker_.scale.y = 0.721;   //宽
    odom_robot_marker_.scale.z = 0.330;   //高

    odom_robot_marker_.color.a = 0.5;
    odom_robot_marker_.color.r = 1;
    odom_robot_marker_.color.g = 1;
    odom_robot_marker_.color.b = 0;

    /*odom text marker*/
    odom_text_marker_.header.frame_id = "map";
    odom_text_marker_.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    odom_text_marker_.action          = visualization_msgs::msg::Marker::MODIFY;
    odom_text_marker_.ns              = std::string("utils_tool");
    odom_robot_marker_.id             = 1;

    odom_text_marker_.scale.x = 0.2;   //长
    odom_text_marker_.scale.y = 0.2;   //宽
    odom_text_marker_.scale.z = 0.2;   //高

    odom_text_marker_.color.a = 0.8;
    odom_text_marker_.color.r = 1;
    odom_text_marker_.color.g = 1;
    odom_text_marker_.color.b = 0;


    markers_.markers.push_back(odom_robot_marker_);
    markers_.markers.push_back(odom_text_marker_);
}

void VisualizationTools::declareAndGetParameters()
{
    get_parameter_or("is_visualization_odom", isVisualizationOdom_, true);
    get_parameter_or("odom_topic_name", odom_topic_name_, std::string("odom"));
}

}   // namespace utils_tool