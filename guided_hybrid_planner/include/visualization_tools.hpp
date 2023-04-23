/*
 * @Author       : dwayne
 * @Date         : 2023-04-21
 * @LastEditTime : 2023-04-22
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "type.hpp"
#include "visualization_msgs/msg/marker.hpp"

#ifndef VISUALIZATION_TOOLS_
#    define VISUALIZATION_TOOLS_


namespace guided_hybrid_a_star {
class VisualizationTools : public nav2_util::LifecycleNode
{
public:
    VisualizationTools(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VisualizationTools();

    /**
   * @brief Configure node
   */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief Activate node
   */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief Deactivate node
   */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief Cleanup node
   */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief shutdown node
   */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

    void add_point_pair(Vec2d first, Vec2d second);

    void publish_point_pairs();



private:
    /* data */
    VectorVec4d                                                                      point_pairs_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr serch_tree_publisher_;
    visualization_msgs::msg::Marker                                                  tree_list_;
};


}   // namespace guided_hybrid_a_star

#endif