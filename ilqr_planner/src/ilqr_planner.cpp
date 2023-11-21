#include "ilqr_planner.h"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <mpc_msgs/msg/detail/vehicle_state__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <utility>

#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "optimizer_utils.h"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"
#include "vehicle_state_ackermann.h"
#include "vehicle_state_interface.h"

namespace IlqrPlanner {
IlqrPlanner::IlqrPlanner(std::string name) : rclcpp::Node(name) {
  RCLCPP_INFO(get_logger(), "start ilqrplanner....");
  declareParameter();
  vehicle_model_ptr_ =
      std::make_shared<VehicleModelBicycleRearDriveFiveState>(wheel_base_);
  Eigen::MatrixXd Q, Q_end, R;
  setWeightMatrix(Q, Q_end, R);
  optimizer_ptr_ =
      std::make_shared<IlqrOptimizer>(vehicle_model_ptr_, Q, Q_end, R);
  ref_path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
      "/generate_path", rclcpp::SystemDefaultsQoS(),
      std::bind(&IlqrPlanner::refPathCallback, this, std::placeholders::_1));
  opt_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/opt_path", rclcpp::SystemDefaultsQoS());
  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());
  vehicle_control_publisher_ = create_publisher<mpc_msgs::msg::VehicleState>(
      "/vehicle_control", rclcpp::SystemDefaultsQoS());
  cur_state_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SystemDefaultsQoS(),
      std::bind(&IlqrPlanner::curStateCallback, this, std::placeholders::_1));
  vehicle_state_subscriber_ = create_subscription<mpc_msgs::msg::VehicleState>(
      "/vehicle_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&IlqrPlanner::vehicleStateCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "start ilqrplanner successfully....");
}

void IlqrPlanner::refPathCallback(
    const nav_msgs::msg::Path::SharedPtr ref_path) {
  RCLCPP_INFO(get_logger(), "refPathCallback...");
  ref_path_.poses.clear();
  if (ref_path->poses.empty()) {
    RCLCPP_WARN(get_logger(), "ref_path is empty!");
    return;
  }
  if (!get_vehicle_state_flag_) {
    RCLCPP_WARN(get_logger(), "haven't get cur state, skip optimize! ");
    return;
  }
  Eigen::VectorXd cur_state =
      Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  cur_state << state_.x, state_.y, state_.theta, state_.vel, state_.steer;
  vehicle_model_ptr_->setCurStateVec(cur_state);
  geometry_msgs::msg::Pose near_pose;
  size_t point_idx;
  double dist;
  OptimizerUtils::getNearPose(vehicle_model_ptr_, *ref_path, near_pose,
                              point_idx, dist);
  RCLCPP_INFO_STREAM(get_logger(),
                     "ref_path size: " << ref_path->poses.size()
                                       << " point_idx: " << point_idx);
  if (point_idx > ref_path->poses.size() - 3) {
    RCLCPP_WARN(get_logger(),
                "close to the end of reference path, skip optimize!");
    return;
  }
  for (size_t i = 0;
       i < max_opt_points_ && i + point_idx < ref_path->poses.size(); i++) {
    ref_path_.poses.emplace_back(std::move(ref_path->poses.at(i + point_idx)));
  }
  RCLCPP_INFO_STREAM(get_logger(),
                     "ref_path_.poses.size(): " << ref_path_.poses.size());

  std::shared_ptr<VehicleStateInterface> end_state =
      std::make_shared<VehicleStateAckermann>();
  end_state->base_state_.pose.pose = ref_path_.poses.back().pose;
  end_state->base_state_.twist.twist.linear.x = 0.0;
  vehicle_model_ptr_->setEndState(end_state);
  std::shared_ptr<VehicleStateAckermann> max_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  max_state_ptr->base_state_.twist.twist.linear.x = 2.0;
  max_state_ptr->acc_vx_ = 4.0;
  vehicle_model_ptr_->setMaxState(max_state_ptr);
  std::dynamic_pointer_cast<IlqrOptimizer>(optimizer_ptr_)
      ->setTimeInterval(opt_time_interval_);
  std::cout << "before opt: " << std::endl;
  std::vector<Eigen::VectorXd> ref_state;
  std::vector<Eigen::VectorXd> optimized_state;
  std::vector<Eigen::VectorXd> control;
  OptimizerUtils::convertToStateVec(ref_path_, ref_state);
  optimizer_ptr_->optimize(ref_state, optimized_state, control);
  opt_path_.header.stamp = get_clock()->now();
  opt_path_.header.frame_id = "map";
  OptimizerUtils::convertToMsg(optimized_state, opt_path_);
  mpc_msgs::msg::VehicleState vehicle_control;
  vehicle_control.acc = control.front()(0);
  vehicle_control.dsteer = control.front()(1);
  vehicle_control.header.stamp = get_clock()->now();
  vehicle_control.time = opt_time_interval_;
  vehicle_control_publisher_->publish(vehicle_control);
  // geometry_msgs::msg::Twist cmd_msg;
  // double time_diff = (get_clock()->now() -
  // cur_state_->header.stamp).seconds(); std::cout << "time_diff: " <<
  // time_diff << std::endl; cmd_msg.linear.x =
  //     cur_state_->twist.twist.linear.x + time_diff * control.front()(0);
  // cmd_msg.angular.z =
  //     cur_state_->twist.twist.angular.z + time_diff * control.front()(1);
  // // cmd_msg.linear.x = optimized_state.at(1)(3);
  // // cmd_msg.angular.z = optimized_state.at(1)(4);
  // // cmd_msg.linear
  // twist_publisher_->publish(cmd_msg);
  opt_path_publisher_->publish(opt_path_);
}

void IlqrPlanner::curStateCallback(
    const nav_msgs::msg::Odometry::SharedPtr cur_state) {
  if (!get_cur_state_flag_) {
    get_cur_state_flag_ = true;
  }
  cur_state_ = cur_state;
}

void IlqrPlanner::vehicleStateCallback(
    const mpc_msgs::msg::VehicleState& cur_state) {
  if (!get_vehicle_state_flag_) {
    get_vehicle_state_flag_ = true;
  }
  state_ = cur_state;
}

void IlqrPlanner::setWeightMatrix(Eigen::MatrixXd& Q, Eigen::MatrixXd& Q_end,
                                  Eigen::MatrixXd& R) {
  Q = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                            vehicle_model_ptr_->getDimX());

  Q_end = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                                vehicle_model_ptr_->getDimX());

  R = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimU(),
                            vehicle_model_ptr_->getDimU());
  for (int i = 0; i < vehicle_model_ptr_->getDimX(); i++) {
    Q(i, i) = weight_intermediate_state_.at(i);
    Q_end(i, i) = weight_end_state_.at(i);
  }
  for (int i = 0; i < vehicle_model_ptr_->getDimU(); i++) {
    R(i, i) = weight_control_.at(i);
  }
}

void IlqrPlanner::declareParameter() {
  declare_parameter("max_opt_points", 40);
  declare_parameter("opt_time_interval", 0.1);
  declare_parameter("auto_cal_time_interval", false);
  declare_parameter("auto_time_interval_coefficient", 1.0);

  declare_parameter("weight_intermediate_state", weight_intermediate_state_);
  declare_parameter("weight_end_state", weight_end_state_);
  declare_parameter("weight_control", weight_control_);

  declare_parameter("wheel_base", 0.65);

  max_opt_points_ = get_parameter("max_opt_points").as_int();
  opt_time_interval_ = get_parameter("opt_time_interval").as_double();
  auto_cal_time_interval_ = get_parameter("auto_cal_time_interval").as_bool();
  auto_time_interval_coefficient_ =
      get_parameter("auto_time_interval_coefficient").as_double();
  weight_intermediate_state_ =
      get_parameter("weight_intermediate_state").as_double_array();
  weight_end_state_ = get_parameter("weight_end_state").as_double_array();
  weight_control_ = get_parameter("weight_control").as_double_array();

  wheel_base_ = get_parameter("wheel_base").as_double();
}
}  // namespace IlqrPlanner
