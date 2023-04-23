/*
 * @Author       : dwayne
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-04-19
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <deque>
#include <mutex>

namespace guided_hybrid_a_star {
class InitPoseSubscriber2D : public nav2_util::LifecycleNode
{
public:
    /**
     * @description  : constructor.
     * @return        {*}
     */
    InitPoseSubscriber2D(std::string        name,
                         const std::string& parent_namespace,
                         const std::string& local_namespace);


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

    void parseData(
        std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>& pose_data_buff);



private:
    void messageCallBack(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_pose_ptr);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
                                                                         init_pose_subscriber_;
    std::string                                                          name_;
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> init_poses_;
    std::mutex                                                           buff_mutex_;
};
}   // namespace guided_hybrid_a_star