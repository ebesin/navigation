/*
 * @Author       : dwayne
 * @Date         : 2023-04-06
 * @LastEditTime : 2023-04-25
 * @Description  : 关于矩阵及向量的定义
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#ifndef TYPE_HPP_
#define TYPE_HPP_

#include <Eigen/Core>
#include <ostream>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace guided_hybrid_a_star {

enum NODE_STATUS { NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2 };

enum DIRECTION { FORWARD = 0, BACKWARD = 1, NO = 3 };

/**
 * @description  : 运动原语
 * @return        {*}
 */
struct MotionPose {
  MotionPose() {}

  MotionPose(const float& x, const float& y, const float& theta)
      : _x(x), _y(y), _theta(theta) {}

  MotionPose operator-(const MotionPose& p2) {
    return MotionPose(this->_x - p2._x, this->_y - p2._y,
                      this->_theta - p2._theta);
  }

  float _x;
  float _y;
  float _theta;
};

struct SmootherParams {
  /**
   * @brief A constructor for nav2_smac_planner::SmootherParams
   */
  SmootherParams() : holonomic_(false) {}

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
           const std::string& name) {
    std::string local_name = name + std::string(".smoother.");

    // Smoother params
    nav2_util::declare_parameter_if_not_declared(node, local_name + "tolerance",
                                                 rclcpp::ParameterValue(1e-10));
    node->get_parameter(local_name + "tolerance", tolerance_);
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "max_iterations", rclcpp::ParameterValue(1000));
    node->get_parameter(local_name + "max_iterations", max_its_);
    nav2_util::declare_parameter_if_not_declared(node, local_name + "w_data",
                                                 rclcpp::ParameterValue(0.2));
    node->get_parameter(local_name + "w_data", w_data_);
    nav2_util::declare_parameter_if_not_declared(node, local_name + "w_smooth",
                                                 rclcpp::ParameterValue(0.3));
    node->get_parameter(local_name + "w_smooth", w_smooth_);
    nav2_util::declare_parameter_if_not_declared(
        node, local_name + "do_refinement", rclcpp::ParameterValue(true));
    node->get_parameter(local_name + "do_refinement", do_refinement_);
  }

  double tolerance_;
  int max_its_;
  double w_data_;
  double w_smooth_;
  bool holonomic_;
  bool do_refinement_;
};

struct SearchInfo {
  bool publish_serch_tree;
  bool show_log;

  float steering_change_penalty;
  float steering_penalty;
  float reverse_penalty;
  float change_direction_penalty;
  float rotation_penalty;

  float cost_penalty;

  bool allow_reverse_expansion;
  double max_steer_angle;
  double wheel_base;

  int angle_segment_size;
  int steer_angle_segment_size;
  double move_step_distance;
  double shot_distance;
  bool travel_unknown;
  int max_iterations;

  double serch_radius;
};

template <int dim>
using TypeVectorVecd = typename std::vector<
    Eigen::Matrix<double, dim, 1>,
    Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef TypeVectorVecd<2> VectorVec2d;
typedef TypeVectorVecd<3> VectorVec3d;
typedef TypeVectorVecd<4> VectorVec4d;

typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4d Vec4d;

typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector4i Vec4i;

namespace Color {
enum Code {
  FG_RED = 31,
  FG_GREEN = 32,
  FG_BLUE = 34,
  FG_DEFAULT = 39,
  BG_RED = 41,
  BG_GREEN = 42,
  BG_BLUE = 44,
  BG_DEFAULT = 49
};

class Modifier {
  Code code;

 public:
  Modifier(Code pCode) : code(pCode) {}

  friend std::ostream& operator<<(std::ostream& os, const Modifier& mod) {
    return os << "\033[" << mod.code << "m";
  }
};
}  // namespace Color

namespace Flag {
class Modifier {
  std::string prompt_;

 public:
  Modifier(std::string prompt) : prompt_(prompt) {}

  friend std::ostream& operator<<(std::ostream& os, const Modifier& mod) {
    return os << "[---" << mod.prompt_ << "---] ";
  }
};
}  // namespace Flag

}  // namespace guided_hybrid_a_star
#endif
