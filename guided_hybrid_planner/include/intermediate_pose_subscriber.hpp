/*
 * @Author       : dwayne
 * @Date         : 2023-04-25
 * @LastEditTime : 2023-04-25
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "hybrid_state_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <mutex>
namespace guided_hybrid_a_star {
class IntermeidatePoseSubscriber : public nav2_util::LifecycleNode
{
public:
    IntermeidatePoseSubscriber(std::string sub_topic_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
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

    void getIntermeidatePose(std::vector<geometry_msgs::msg::PointStamped::SharedPtr>& intermediate_points);

private:
    void messageCallBack(const geometry_msgs::msg::PointStamped::SharedPtr point);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr intermeidate_pose_subscriber_;
    std::string                                                       sub_topic_name_;

    std::vector<geometry_msgs::msg::PointStamped::SharedPtr> intermediate_points_;
    std::mutex                                               buff_mutex_;
};
}   // namespace guided_hybrid_a_star