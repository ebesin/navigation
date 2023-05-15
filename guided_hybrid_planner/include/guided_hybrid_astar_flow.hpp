/*
 * @Author       : dwayne
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-04-25
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "geometry_msgs/msg/point.hpp"
#include "goal_pose_subscriber.hpp"
#include "guided_hybrid_astar.hpp"
#include "init_pose_subscriber.hpp"
#include "intermediate_pose_subscriber.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "smoother.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "visualization_tools.hpp"
#include <chrono>
#include <deque>
#include <mutex>
#include <vector>

namespace guided_hybrid_a_star {
class GuidedHybridAstarFlow : public nav2_util::LifecycleNode
{
public:
    GuidedHybridAstarFlow(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~GuidedHybridAstarFlow();

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

    /**
     * @description  : 读取起点和终点
     * @return        {*}
     */
    void readData();

    /**
     * @description  : 读取起点和终点
     * @return        {*}
     */
    void initPoseData();

    /**
     * @description  : 读取起点和终点
     * @return        {*}
     */
    bool hasValidPose();

    /**
     * @description  : 等待地图建立
     * @return        {*}
     */
    void waitForCostmap();

    void timerCallback();

    /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav2_msgs::Path of the generated path
   */
    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal);

private:
    void mapToWorld(nav2_costmap_2d::Costmap2D* costmap, const double& mx, const double& my, double& wx, double& wy);
    void worldToMap(nav2_costmap_2d::Costmap2D* costmap, const double& wx, const double& wy, double& mx, double& my);

    std::string name_;
    std::string global_frame_ = "map";
    std::mutex  mutex_;
    // Global Costmap
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D*                    costmap_;
    std::unique_ptr<nav2_util::NodeThread>         costmap_thread_;

    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> init_poses_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr             current_init_pose;
    std::shared_ptr<InitPoseSubscriber2D>                                init_pose_subscriber_;
    std::unique_ptr<nav2_util::NodeThread>                               init_pose_subscriber_thread_;

    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> goal_poses_;
    geometry_msgs::msg::PoseStamped::SharedPtr             current_goal_pose;
    std::shared_ptr<GoalPoseSubscriber2D>                  goal_pose_subscriber_;
    std::unique_ptr<nav2_util::NodeThread>                 goal_pose_subscriber_thread_;

    std::vector<geometry_msgs::msg::PointStamped::SharedPtr> intermediate_points_;
    std::shared_ptr<IntermeidatePoseSubscriber>              intermeidate_pose_subscriber_;
    std::unique_ptr<nav2_util::NodeThread>                   intermeidate_pose_subscriber_thread_;

    SearchInfo                         serch_info_;
    std::unique_ptr<GuidedHybridAStar> guided_hybrid_a_star_;

    geometry_msgs::msg::PoseStamped                                      start_;
    geometry_msgs::msg::PoseStamped                                      goal_;
    std::shared_ptr<GridCollisionChecker>                                collision_checker_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

    rclcpp::TimerBase::SharedPtr timer_base_;


    std::shared_ptr<VisualizationTools>    visualization_publisher_;
    std::unique_ptr<nav2_util::NodeThread> visualization_publisher_thread_;

    std::shared_ptr<Smoother> smoother_;
};
}   // namespace guided_hybrid_a_star