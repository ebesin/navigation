/*
 * @Author       : dwayne
 * @Date         : 2023-04-19
 * @LastEditTime : 2023-05-05
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#include "guided_hybrid_astar_flow.hpp"

#include <unistd.h>

#include <fstream>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "coor_tools.h"
#include "optimizer_utils.h"

// #include "ilqr_path_planner.h"

namespace guided_hybrid_a_star {
GuidedHybridAstarFlow::GuidedHybridAstarFlow(const std::string& name,
                                             const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode(name, "", options) {
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
  declare_parameter("change_direction_penalty", rclcpp::ParameterValue(4.0));
  declare_parameter("steering_penalty", rclcpp::ParameterValue(1.05));
  declare_parameter("steering_change_penalty", rclcpp::ParameterValue(1.5));
  declare_parameter("cost_penalty", rclcpp::ParameterValue(2.0));
  declare_parameter("publish_serch_tree", rclcpp::ParameterValue(true));
  declare_parameter("show_log", rclcpp::ParameterValue(true));
  declare_parameter("serch_radius", rclcpp::ParameterValue(1.0));

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{get_namespace()}, "global_costmap");
  // 启动线程运行costmap节点
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  init_pose_subscriber_ = std::make_shared<InitPoseSubscriber2D>(
      "init_pose_subscriber", "", "init_pose_subscriber");
  // 启动线程运行init_pose_subscriber节点
  init_pose_subscriber_thread_ =
      std::make_unique<nav2_util::NodeThread>(init_pose_subscriber_);

  goal_pose_subscriber_ = std::make_shared<GoalPoseSubscriber2D>("/goal_pose");
  // 启动线程运行goal_pose_subscriber节点
  goal_pose_subscriber_thread_ =
      std::make_unique<nav2_util::NodeThread>(goal_pose_subscriber_);

  intermeidate_pose_subscriber_ =
      std::make_shared<IntermeidatePoseSubscriber>("/clicked_point");
  // 启动线程运行oal_pose_subscriber节点
  intermeidate_pose_subscriber_thread_ =
      std::make_unique<nav2_util::NodeThread>(intermeidate_pose_subscriber_);

  visualization_publisher_ = std::make_shared<VisualizationTools>();
  visualization_publisher_thread_ =
      std::make_unique<nav2_util::NodeThread>(visualization_publisher_);
}

GuidedHybridAstarFlow::~GuidedHybridAstarFlow() {}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_configure(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();
  init_pose_subscriber_->configure();
  goal_pose_subscriber_->configure();
  intermeidate_pose_subscriber_->configure();
  costmap_ros_->configure();
  visualization_publisher_->configure();
  costmap_ = costmap_ros_->getCostmap();
  RCLCPP_INFO(node->get_logger(), "Costmap size: %d,%d",
              costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  node->get_parameter("max_steer_angle", serch_info_.max_steer_angle);
  node->get_parameter("wheel_base", serch_info_.wheel_base);
  node->get_parameter("max_iterations", serch_info_.max_iterations);
  node->get_parameter("travel_unknown", serch_info_.travel_unknown);
  node->get_parameter("shot_distance", serch_info_.shot_distance);
  node->get_parameter("angle_segment_size", serch_info_.angle_segment_size);
  node->get_parameter("steer_angle_segment_size",
                      serch_info_.steer_angle_segment_size);
  node->get_parameter("move_step_distance", serch_info_.move_step_distance);
  node->get_parameter("allow_reverse_expansion",
                      serch_info_.allow_reverse_expansion);
  node->get_parameter("reverse_penalty", serch_info_.reverse_penalty);
  node->get_parameter("change_direction_penalty",
                      serch_info_.change_direction_penalty);
  node->get_parameter("steering_penalty", serch_info_.steering_penalty);
  node->get_parameter("steering_change_penalty",
                      serch_info_.steering_change_penalty);
  node->get_parameter("cost_penalty", serch_info_.cost_penalty);
  node->get_parameter("publish_serch_tree", serch_info_.publish_serch_tree);
  node->get_parameter("show_log", serch_info_.show_log);
  node->get_parameter("serch_radius", serch_info_.serch_radius);

  RCLCPP_INFO_STREAM(get_logger(),
                     "max_steer_angle: " << serch_info_.max_steer_angle);
  RCLCPP_INFO_STREAM(get_logger(), "wheel_base: " << serch_info_.wheel_base);
  RCLCPP_INFO_STREAM(get_logger(),
                     "max_iterations: " << serch_info_.max_iterations);
  RCLCPP_INFO_STREAM(get_logger(),
                     "travel_unknown: " << serch_info_.travel_unknown);
  RCLCPP_INFO_STREAM(get_logger(),
                     "shot_distance: " << serch_info_.shot_distance);
  RCLCPP_INFO_STREAM(get_logger(),
                     "angle_segment_size: " << serch_info_.angle_segment_size);
  RCLCPP_INFO_STREAM(get_logger(), "steer_angle_segment_size: "
                                       << serch_info_.steer_angle_segment_size);
  RCLCPP_INFO_STREAM(get_logger(),
                     "move_step_distance: " << serch_info_.move_step_distance);
  RCLCPP_INFO_STREAM(get_logger(), "allow_reverse_expansion: "
                                       << serch_info_.allow_reverse_expansion);
  RCLCPP_INFO_STREAM(get_logger(),
                     "reverse_penalty: " << serch_info_.reverse_penalty);
  RCLCPP_INFO_STREAM(get_logger(), "change_direction_penalty: "
                                       << serch_info_.change_direction_penalty);
  RCLCPP_INFO_STREAM(get_logger(),
                     "steering_penalty: " << serch_info_.steering_penalty);
  RCLCPP_INFO_STREAM(get_logger(), "steering_change_penalty: "
                                       << serch_info_.steering_change_penalty);
  RCLCPP_INFO_STREAM(get_logger(),
                     "cost_penalty: " << serch_info_.cost_penalty);
  RCLCPP_INFO_STREAM(get_logger(),
                     "publish_serch_tree: " << serch_info_.publish_serch_tree);
  RCLCPP_INFO_STREAM(get_logger(), "show_log: " << serch_info_.show_log);
  RCLCPP_INFO_STREAM(get_logger(),
                     "serch_radius: " << serch_info_.serch_radius);

  if (serch_info_.max_iterations < 0) {
    RCLCPP_WARN(node->get_logger(), "最大迭代次数<0,将重新设置为最大值");
    serch_info_.max_iterations = std::numeric_limits<int>::max();
  }

  auto angel2Rad = [](double angle) { return angle / 180 * M_PI; };

  serch_info_.max_steer_angle = angel2Rad(serch_info_.max_steer_angle);

  SmootherParams params;
  params.get(node, "");
  smoother_ = std::make_shared<Smoother>(params);
  smoother_->initialize(static_cast<double>(serch_info_.wheel_base /
                                            tan(serch_info_.max_steer_angle)));

  serch_info_.shot_distance =
      serch_info_.shot_distance / costmap_->getResolution();
  // serch_info_.move_step_distance = serch_info_.move_step_distance /
  // costmap_->getResolution();
  serch_info_.wheel_base = serch_info_.wheel_base / costmap_->getResolution();
  serch_info_.serch_radius =
      serch_info_.serch_radius / costmap_->getResolution();

  // collision_checker_ =
  //     std::make_shared<GridCollisionChecker>(costmap_,
  //     serch_info_.angle_segment_size);

  vehicle_model_ptr_ =
      std::make_shared<VehicleModel::VehicleModelBicycleRearDriveFiveState>(
          serch_info_.wheel_base);

  Eigen::MatrixXd Q, Q_end, R;
  Q = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                            vehicle_model_ptr_->getDimX());
  Q_end = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                                vehicle_model_ptr_->getDimX());
  R = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimU(),
                            vehicle_model_ptr_->getDimU());

  Q(0, 0) = 1e1;
  Q(1, 1) = 1e1;
  Q(2, 2) = 1e1;
  Q_end(0, 0) = 1e4;
  Q_end(1, 1) = 1e4;
  Q_end(2, 2) = 1e4;
  R(0, 0) = 1e-2;
  R(1, 1) = 1e-2;

  ilqr_planner_ = std::make_shared<Optimizer::IlqrPathPlanner>(
      vehicle_model_ptr_, Q, Q_end, R);

  guided_hybrid_a_star_ =
      std::make_unique<GuidedHybridAStar>(costmap_, serch_info_);

  plan_publisher_ =
      node->create_publisher<nav_msgs::msg::Path>("origin_path", 1);
  this->rs_path_ptr_ = std::make_shared<RSPath>(
      serch_info_.wheel_base / tan(serch_info_.max_steer_angle));
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_activate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Activating");
  init_pose_subscriber_->activate();
  goal_pose_subscriber_->activate();
  intermeidate_pose_subscriber_->activate();
  costmap_ros_->activate();
  visualization_publisher_->activate();
  plan_publisher_->on_activate();

  timer_base_ =
      create_wall_timer(std::chrono::milliseconds(static_cast<int>(2000)),
                        std::bind(&GuidedHybridAstarFlow::timerCallback, this));

  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "DeActivating");

  init_pose_subscriber_->deactivate();
  goal_pose_subscriber_->deactivate();
  intermeidate_pose_subscriber_->deactivate();
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

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Cleaning");
  init_pose_subscriber_->cleanup();
  init_poses_.clear();
  goal_pose_subscriber_->cleanup();
  intermeidate_pose_subscriber_->cleanup();
  goal_poses_.clear();
  costmap_ros_->cleanup();
  visualization_publisher_->cleanup();
  plan_publisher_.reset();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GuidedHybridAstarFlow::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), " Shutdown");
  return nav2_util::CallbackReturn::SUCCESS;
}

void GuidedHybridAstarFlow::readData() {
  init_pose_subscriber_->parseData(init_poses_);
  goal_pose_subscriber_->parseData(goal_poses_);
}

void GuidedHybridAstarFlow::initPoseData() {
  current_init_pose = init_poses_.front();
  init_poses_.pop_front();
  current_goal_pose = goal_poses_.front();
  goal_poses_.pop_front();
}

bool GuidedHybridAstarFlow::hasValidPose() {
  return (!init_poses_.empty()) && (!goal_poses_.empty());
}

void GuidedHybridAstarFlow::waitForCostmap() {
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

void GuidedHybridAstarFlow::mapToWorld(nav2_costmap_2d::Costmap2D* costmap,
                                       const double& mx, const double& my,
                                       double& wx, double& wy) {
  wx = costmap->getOriginX() + mx * costmap->getResolution();
  wy = costmap->getOriginY() + my * costmap->getResolution();
}

void GuidedHybridAstarFlow::worldToMap(nav2_costmap_2d::Costmap2D* costmap,
                                       const double& wx, const double& wy,
                                       double& mx, double& my) {
  mx = (wx - costmap->getOriginX()) / costmap->getResolution();
  my = (wy - costmap->getOriginY()) / costmap->getResolution();
}

void GuidedHybridAstarFlow::timerCallback() {
  timer_count++;
  if (timer_count < 0) return;
  readData();
  if (hasValidPose()) {
    initPoseData();
    waitForCostmap();
    RCLCPP_INFO(get_logger(), "generateVoronoiMap......");
    guided_hybrid_a_star_->generateVoronoiMap();
    RCLCPP_INFO(get_logger(), "collision_checker_......");
    collision_checker_ = std::make_shared<GridCollisionChecker>(
        costmap_, serch_info_.angle_segment_size);
    collision_checker_->setFootprint(costmap_ros_->getRobotFootprint(), false,
                                     -1.0);
    RCLCPP_INFO(get_logger(), "createPlan......");
    nav_msgs::msg::Path plan =
        createPlan(current_init_pose->pose.pose, current_goal_pose->pose);
    smoother_->smooth(plan, costmap_, 10);
    plan_publisher_->publish(plan);
  }
}

std::string get_cur_time_str() {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
  std::tm* timeInfo = std::localtime(&currentTime);
  std::stringstream ss;
  ss << std::put_time(timeInfo, "%Y-%m-%d-%H-%M-%S") << "-" << ms.count();
  return ss.str();
}

void GuidedHybridAstarFlow::createRSPath(
    geometry_msgs::msg::PoseStamped begin_pose,
    geometry_msgs::msg::PoseStamped end_pose, nav_msgs::msg::Path& rs_path) {
  Vec3d start;
  Vec3d end;
  start.x() = begin_pose.pose.position.x;
  start.y() = begin_pose.pose.position.y;
  start.z() = utils_tool::getYawFromQuaternion(begin_pose.pose.orientation);
  end.x() = end_pose.pose.position.x;
  end.y() = end_pose.pose.position.y;
  end.z() = utils_tool::getYawFromQuaternion(end_pose.pose.orientation);
  double distance;
  auto pose_vec = rs_path_ptr_->GetRSPath(start, end, 0.05, distance);
  rs_path.poses.clear();
  for (auto p : pose_vec) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.orientation = utils_tool::createQuaternionMsgFromYaw(p.z());
    rs_path.poses.emplace_back(std::move(pose));
  }
}

nav_msgs::msg::Path GuidedHybridAstarFlow::createPlan(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal) {
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  guided_hybrid_a_star_->setCollisionChecker(collision_checker_);
  guided_hybrid_a_star_->setVisualizationToolsPtr(visualization_publisher_);
  unsigned int mx, my;
  costmap_->worldToMap(start.position.x, start.position.y, mx, my);
  guided_hybrid_a_star_->setStart(
      Vec3d(mx, my, tf2::getYaw(start.orientation)));

  costmap_->worldToMap(goal.position.x, goal.position.y, mx, my);
  guided_hybrid_a_star_->setGoal(Vec3d(mx, my, tf2::getYaw(goal.orientation)));

  std::vector<geometry_msgs::msg::PointStamped::SharedPtr>
      w_intermediate_points;
  intermeidate_pose_subscriber_->getIntermeidatePose(w_intermediate_points);
  std::vector<Vec3d> m_intermediate_points;
  m_intermediate_points.resize(w_intermediate_points.size());
  for (int i = 0; i < w_intermediate_points.size(); ++i) {
    worldToMap(costmap_, w_intermediate_points[i]->point.x,
               w_intermediate_points[i]->point.y, m_intermediate_points[i].x(),
               m_intermediate_points[i].y());
    if (i == 0) {
      double cosValue =
          Vec2d(mx, my).dot(m_intermediate_points[0].head(2)) /
          (Vec2d(mx, my).norm() * m_intermediate_points[0].head(2).norm());
      m_intermediate_points[i].z() = acos(cosValue);
    } else {
      double cosValue = m_intermediate_points[i - 1].head(2).dot(
                            m_intermediate_points[0].head(2)) /
                        (m_intermediate_points[i - 1].head(2).norm() *
                         m_intermediate_points[0].head(2).norm());
      m_intermediate_points[i].z() = acos(cosValue);
    }
  }
  guided_hybrid_a_star_->setIntermediateNodes(m_intermediate_points);
  nav_msgs::msg::Path plan;
  std::vector<nav_msgs::msg::Path> plans;
  nav_msgs::msg::Path total_plan;
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
  } catch (const std::runtime_error& e) {
    error = "invalid use: ";
    error += e.what();
  }
  std::vector<VectorVec3d> paths;
  guided_hybrid_a_star_->backtracePath(paths);
  RCLCPP_INFO_STREAM(get_logger(), "path segment size: " << paths.size());
  std::vector<VectorVec3d> forward_paths;
  for (int i = paths.size() - 1; i >= 0; --i) {
    VectorVec3d single_path;
    for (int j = paths.at(i).size() - 1; j >= 0; --j) {
      single_path.emplace_back(paths.at(i).at(j));
    }
    RCLCPP_INFO_STREAM(get_logger(), "segment size: " << single_path.size());
    forward_paths.emplace_back(std::move(single_path));
  }
  for (auto path : forward_paths) {
    nav_msgs::msg::Path single_plan;
    nav_msgs::msg::Path smooth_path;
    for (auto point : path) {
      geometry_msgs::msg::PoseStamped single_pose;
      single_pose.pose.position.x =
          costmap_->getOriginX() +
          (point.x() + 0.5) * costmap_->getResolution();
      single_pose.pose.position.y =
          costmap_->getOriginY() +
          (point.y() + 0.5) * costmap_->getResolution();
      tf2::Quaternion q;
      q.setEuler(0.0, 0.0, point.z());
      single_pose.pose.orientation = tf2::toMsg(q);

      single_plan.poses.emplace_back(std::move(single_pose));
    }
    plans.emplace_back(std::move(single_plan));
  }

  for (auto path : plans) {
    for (auto pose : path.poses) {
      plan.poses.emplace_back(std::move(pose));
    }
  }

  geometry_msgs::msg::PoseStamped begin_pose;
  geometry_msgs::msg::PoseStamped end_pose;
  nav_msgs::msg::Path opt_path;
  bool re_plan{false};
  for (int i = 0; i < plans.size(); i++) {
    if (re_plan && plans.at(i).poses.size() < 10) {
      continue;
    }
    if (re_plan && plans.at(i).poses.size() >= 10) {
      re_plan = false;
      if (plans.at(i).poses.size() > 30) {
        end_pose = plans.at(i).poses.at(10);
        // ilqr_planner_->doPlan(begin_pose.pose, end_pose.pose, opt_path);
        createRSPath(begin_pose, end_pose, opt_path);
        // plan.poses.insert(plan.poses.end(), opt_path.poses.begin(),
        //                   opt_path.poses.end());
        nav_msgs::msg::Path tmp_path;
        tmp_path.poses.insert(tmp_path.poses.end(),
                              plans.at(i).poses.begin() + 11,
                              plans.at(i).poses.end());
        std::vector<nav_msgs::msg::Path> s_paths;
        OptimizerUtils::pathSegmentation(tmp_path, 50, s_paths);
        for (auto& path : s_paths) {
          // ilqr_planner_->doPlan(path, path);
          plan.poses.insert(plan.poses.end(), path.poses.begin(),
                            path.poses.end());
        }
      } else {
        end_pose = plans.at(i).poses.back();
        // ilqr_planner_->doPlan(begin_pose.pose, end_pose.pose, opt_path);
        createRSPath(begin_pose, end_pose, opt_path);
        // plan.poses.insert(plan.poses.end(), opt_path.poses.begin(),
        //                   opt_path.poses.end());
      }
    } else if (!re_plan && i < plans.size() - 1 &&
               plans.at(i + 1).poses.size() < 10) {
      re_plan = true;
      if (plans.at(i).poses.size() > 30) {
        nav_msgs::msg::Path tmp_path;
        tmp_path.poses.insert(tmp_path.poses.end(), plans.at(i).poses.begin(),
                              plans.at(i).poses.end() - 10);
        std::vector<nav_msgs::msg::Path> s_paths;
        OptimizerUtils::pathSegmentation(tmp_path, 50, s_paths);
        for (auto& path : s_paths) {
          // ilqr_planner_->doPlan(path, path);
          plan.poses.insert(plan.poses.end(), path.poses.begin(),
                            path.poses.end());
        }
        begin_pose = plans.at(i).poses.at(plans.at(i).poses.size() - 10);
      } else {
        begin_pose = plans.at(i).poses.front();
      }
    } else {
      std::vector<nav_msgs::msg::Path> s_paths;
      OptimizerUtils::pathSegmentation(plans.at(i), 50, s_paths);
      for (auto& path : s_paths) {
        // ilqr_planner_->doPlan(path, path);
        plan.poses.insert(plan.poses.end(), path.poses.begin(),
                          path.poses.end());
      }
    }
  }

  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "guided_hybrid_planner/data/"
     << get_cur_time_str() << ".txt";

  std::ofstream file(ss.str());

  for (auto pose : plan.poses) {
    file << pose.pose << std::endl;
  }
  file.close();

  if (plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(plan);
  }
  return plan;
}

}  // namespace guided_hybrid_a_star
