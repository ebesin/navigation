/*
 * @Author       : dwayne
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-04-22
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "guided_hybrid_astar_flow.hpp"

namespace guided_hybrid_a_star {
GuidedHybridAstarFlow::GuidedHybridAstarFlow(const std::string& name, const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode(name, "", options)
{
    RCLCPP_INFO(get_logger(), "Creating guided_hybrid_a_star...");
    name_ = name;

    declare_parameter("max_steer_angle", rclcpp::ParameterValue(27.0));
    declare_parameter("wheel_base", rclcpp::ParameterValue(0.65));
    declare_parameter("max_iterations", rclcpp::ParameterValue(1000000));
    declare_parameter("travel_unknown", rclcpp::ParameterValue(false));
    declare_parameter("shot_distance", rclcpp::ParameterValue(3.0));
    declare_parameter("angle_segment_size", rclcpp::ParameterValue(72));
    declare_parameter("steer_angle_segment_size", rclcpp::ParameterValue(10));
    declare_parameter("move_step_distance", rclcpp::ParameterValue(1.414));
    declare_parameter("allow_reverse_expansion", rclcpp::ParameterValue(true));
    declare_parameter("reverse_penalty", rclcpp::ParameterValue(2.0));
    declare_parameter("steering_penalty", rclcpp::ParameterValue(1.05));
    declare_parameter("steering_change_penalty", rclcpp::ParameterValue(1.5));
    declare_parameter("cost_penalty", rclcpp::ParameterValue(2.0));

    costmap_ros_ =
        std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap", std::string{get_namespace()}, "global_costmap");
    //启动线程运行costmap节点
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    init_pose_subscriber_ = std::make_shared<InitPoseSubscriber2D>("init_pose_subscriber", "", "init_pose_subscriber");
    //启动线程运行init_pose_subscriber节点
    init_pose_subscriber_thread_ = std::make_unique<nav2_util::NodeThread>(init_pose_subscriber_);

    goal_pose_subscriber_ = std::make_shared<GoalPoseSubscriber2D>("/goal_pose");
    //启动线程运行oal_pose_subscriber节点
    goal_pose_subscriber_thread_ = std::make_unique<nav2_util::NodeThread>(goal_pose_subscriber_);

    visualization_publisher_        = std::make_shared<VisualizationTools>();
    visualization_publisher_thread_ = std::make_unique<nav2_util::NodeThread>(visualization_publisher_);
}

GuidedHybridAstarFlow::~GuidedHybridAstarFlow() {}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_configure(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Configuring");
    auto node = shared_from_this();
    init_pose_subscriber_->configure();
    goal_pose_subscriber_->configure();
    costmap_ros_->configure();
    visualization_publisher_->configure();
    costmap_ = costmap_ros_->getCostmap();
    RCLCPP_DEBUG(node->get_logger(), "Costmap size: %d,%d", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());


    node->get_parameter("max_steer_angle", serch_info_.max_steer_angle);
    node->get_parameter("wheel_base", serch_info_.wheel_base);
    node->get_parameter("max_iterations", serch_info_.max_iterations);
    node->get_parameter("travel_unknown", serch_info_.travel_unknown);
    node->get_parameter("shot_distance", serch_info_.shot_distance);
    node->get_parameter("angle_segment_size", serch_info_.angle_segment_size);
    node->get_parameter("steer_angle_segment_size", serch_info_.steer_angle_segment_size);
    node->get_parameter("move_step_distance", serch_info_.move_step_distance);
    node->get_parameter("allow_reverse_expansion", serch_info_.allow_reverse_expansion);
    node->get_parameter("reverse_penalty", serch_info_.reverse_penalty);
    node->get_parameter("steering_penalty", serch_info_.steering_penalty);
    node->get_parameter("steering_change_penalty", serch_info_.steering_change_penalty);
    node->get_parameter("cost_penalty", serch_info_.cost_penalty);

    RCLCPP_INFO_STREAM(get_logger(), "max_steer_angle: " << serch_info_.max_steer_angle);
    RCLCPP_INFO_STREAM(get_logger(), "wheel_base: " << serch_info_.wheel_base);
    RCLCPP_INFO_STREAM(get_logger(), "max_iterations: " << serch_info_.max_iterations);
    RCLCPP_INFO_STREAM(get_logger(), "travel_unknown: " << serch_info_.travel_unknown);
    RCLCPP_INFO_STREAM(get_logger(), "shot_distance: " << serch_info_.shot_distance);
    RCLCPP_INFO_STREAM(get_logger(), "angle_segment_size: " << serch_info_.angle_segment_size);
    RCLCPP_INFO_STREAM(get_logger(), "steer_angle_segment_size: " << serch_info_.steer_angle_segment_size);
    RCLCPP_INFO_STREAM(get_logger(), "move_step_distance: " << serch_info_.move_step_distance);
    RCLCPP_INFO_STREAM(get_logger(), "allow_reverse_expansion: " << serch_info_.allow_reverse_expansion);
    RCLCPP_INFO_STREAM(get_logger(), "reverse_penalty: " << serch_info_.reverse_penalty);
    RCLCPP_INFO_STREAM(get_logger(), "steering_penalty: " << serch_info_.steering_penalty);
    RCLCPP_INFO_STREAM(get_logger(), "steering_change_penalty: " << serch_info_.steering_change_penalty);
    RCLCPP_INFO_STREAM(get_logger(), "cost_penalty: " << serch_info_.cost_penalty);

    if (serch_info_.max_iterations < 0) {
        RCLCPP_WARN(node->get_logger(), "最大迭代次数<0,将重新设置为最大值");
        serch_info_.max_iterations = std::numeric_limits<int>::max();
    }

    auto angel2Rad = [](double angle) { return angle / 180 * M_PI; };

    serch_info_.max_steer_angle = angel2Rad(serch_info_.max_steer_angle);
    serch_info_.shot_distance   = serch_info_.shot_distance / costmap_->getResolution();
    // serch_info_.move_step_distance = serch_info_.move_step_distance / costmap_->getResolution();
    serch_info_.wheel_base = serch_info_.wheel_base / costmap_->getResolution();

    // collision_checker_ =
    //     std::make_shared<GridCollisionChecker>(costmap_, serch_info_.angle_segment_size);

    guided_hybrid_a_star_ = std::make_unique<GuidedHybridAStar>(costmap_, serch_info_);

    plan_publisher_ = node->create_publisher<nav_msgs::msg::Path>("origin_path", 1);

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_activate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Activating");
    init_pose_subscriber_->activate();
    goal_pose_subscriber_->activate();
    costmap_ros_->activate();
    visualization_publisher_->activate();
    plan_publisher_->on_activate();

    timer_base_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000)),
                                    std::bind(&GuidedHybridAstarFlow::timerCallback, this));

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_deactivate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "DeActivating");

    init_pose_subscriber_->deactivate();
    goal_pose_subscriber_->deactivate();
    costmap_ros_->deactivate();
    visualization_publisher_->deactivate();
    plan_publisher_->on_deactivate();
    if (timer_base_) {
        timer_base_->cancel();
        timer_base_->reset();
    }
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_cleanup(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Cleaning");
    init_pose_subscriber_->cleanup();
    init_poses_.clear();
    goal_pose_subscriber_->cleanup();
    goal_poses_.clear();
    costmap_ros_->cleanup();
    visualization_publisher_->cleanup();
    plan_publisher_.reset();
    costmap_ = nullptr;
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_shutdown(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), " Shutdown");
    return nav2_util::CallbackReturn::SUCCESS;
}

void GuidedHybridAstarFlow::readData()
{
    init_pose_subscriber_->parseData(init_poses_);
    goal_pose_subscriber_->parseData(goal_poses_);
}

void GuidedHybridAstarFlow::initPoseData()
{
    current_init_pose = init_poses_.front();
    init_poses_.pop_front();
    current_goal_pose = goal_poses_.front();
    goal_poses_.pop_front();
}

bool GuidedHybridAstarFlow::hasValidPose()
{
    return (!init_poses_.empty()) && (!goal_poses_.empty());
}

void GuidedHybridAstarFlow::waitForCostmap()
{
    // Don't compute a plan until costmap is valid (after clear costmap)
    rclcpp::Rate r(100);
    while (!costmap_ros_->isCurrent()) {
        r.sleep();
    }
}

void GuidedHybridAstarFlow::timerCallback()
{
    readData();
    if (hasValidPose()) {
        initPoseData();
        waitForCostmap();
        collision_checker_ = std::make_shared<GridCollisionChecker>(costmap_, serch_info_.angle_segment_size);

        nav_msgs::msg::Path plan = createPlan(current_init_pose->pose.pose, current_goal_pose->pose);
        plan_publisher_->publish(plan);
    }
}

nav_msgs::msg::Path GuidedHybridAstarFlow::createPlan(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal)
{
    std::lock_guard<std::mutex> lock_reinit(mutex_);
    guided_hybrid_a_star_->setCollisionChecker(collision_checker_);
    guided_hybrid_a_star_->setVisualizationToolsPtr(visualization_publisher_);
    unsigned int mx, my;
    costmap_->worldToMap(start.position.x, start.position.y, mx, my);
    guided_hybrid_a_star_->setStart(Vec3d(mx, my, tf2::getYaw(start.orientation)));

    costmap_->worldToMap(goal.position.x, goal.position.y, mx, my);
    guided_hybrid_a_star_->setGoal(Vec3d(mx, my, tf2::getYaw(goal.orientation)));

    nav_msgs::msg::Path plan;
    plan.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header             = plan.header;
    pose.pose.position.z    = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    VectorVec3d path;
    std::string error;
    try {
        if (!guided_hybrid_a_star_->complutePath()) {
            std::cout << "---------error---------" << std::endl;
            return plan;
        }
    }
    catch (const std::runtime_error& e) {
        error = "invalid use: ";
        error += e.what();
    }

    if (guided_hybrid_a_star_->backtracePath(path)) {
        plan.poses.reserve(path.size());
        for (int i = path.size() - 1; i >= 0; --i) {
            // costmap_->mapToWorld(
            //     path[i].x(), path[i].y(), pose.pose.position.x, pose.pose.position.y);
            pose.pose.position.x = costmap_->getOriginX() + (path[i].x() + 0.5) * costmap_->getResolution();
            pose.pose.position.y = costmap_->getOriginY() + (path[i].y() + 0.5) * costmap_->getResolution();
            tf2::Quaternion q;
            q.setEuler(0.0, 0.0, path[i].z());
            pose.pose.orientation = tf2::toMsg(q);
            plan.poses.push_back(pose);
        }
    }
    else {
        std::cout << "---------error2---------" << std::endl;
    }

    if (plan_publisher_->get_subscription_count() > 0) {
        plan_publisher_->publish(plan);
    }
    return plan;
}

}   // namespace guided_hybrid_a_star