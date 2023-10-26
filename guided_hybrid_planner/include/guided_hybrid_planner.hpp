/*
 * @Author       : dwayne
 * @Date         : 2023-04-08
 * @LastEditTime : 2023-04-18
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#ifndef GUIDED_HYBRID_PLANNER_HPP_
#define GUIDED_HYBRID_PLANNER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "guided_hybrid_astar.hpp"
#include "iostream"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "type.hpp"

namespace guided_hybrid_a_star {
class GuidedHybridPlanner : public nav2_core::GlobalPlanner {
public:
  /**
   * @description  : constructor
   * @return        {*}
   */
  GuidedHybridPlanner();

  /**
   * @description  : destructor
   * @return        {*}
   */
  ~GuidedHybridPlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav2_msgs::Path of the generated path
   */
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  std::unique_ptr<GuidedHybridAStar> guided_hybrid_a_star_;

  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("GuidedHybridPlanner")};
  nav2_costmap_2d::Costmap2D *costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex _mutex;

  geometry_msgs::msg::PoseStamped start_;
  geometry_msgs::msg::PoseStamped goal_;

  SearchInfo serch_info_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
      plan_publisher_;

  // Color::Modifier blue
};
} // namespace guided_hybrid_a_star

#endif
