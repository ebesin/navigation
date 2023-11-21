#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <mpc_msgs/msg/detail/vehicle_state__struct.hpp>
#include <mpc_msgs/msg/detail/vehicle_status__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <vector>

#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "mpc_msgs/msg/vehicle_state.hpp"
#include "optimizer_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_four_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"
#include "vehicle_model_interface.h"
#include "vehicle_state_interface.h"

using namespace Optimizer;
using namespace VehicleModel;

namespace IlqrPlanner {
class IlqrPlanner : public rclcpp::Node {
 public:
  IlqrPlanner(std::string name);
  ~IlqrPlanner() = default;

 private:
  void setWeightMatrix(Eigen::MatrixXd& Q, Eigen::MatrixXd& Q_end,
                       Eigen::MatrixXd& R);

  void refPathCallback(const nav_msgs::msg::Path::SharedPtr ref_path);
  void curStateCallback(const nav_msgs::msg::Odometry::SharedPtr cur_state);
  void vehicleStateCallback(const mpc_msgs::msg::VehicleState& cur_state);

  void declareParameter();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_path_subscriber_;
  rclcpp::Subscription<mpc_msgs::msg::VehicleState>::SharedPtr
      vehicle_state_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<mpc_msgs::msg::VehicleState>::SharedPtr
      vehicle_control_publisher_;
  // rclcpp::Publisher<>
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      cur_state_subscriber_;
  std::shared_ptr<OptimizerInterface> optimizer_ptr_;
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;
  nav_msgs::msg::Path ref_path_;
  nav_msgs::msg::Path opt_path_;
  mpc_msgs::msg::VehicleState state_;
  bool get_vehicle_state_flag_{false};

  nav_msgs::msg::Odometry::SharedPtr cur_state_;
  bool get_cur_state_flag_{false};

  int max_opt_points_;
  double opt_time_interval_;
  bool auto_cal_time_interval_;
  double auto_time_interval_coefficient_;

  std::vector<double> weight_intermediate_state_ = {1e0, 1e0, 1e0, 1e0, 1e0};
  std::vector<double> weight_end_state_{1e3, 1e3, 1e3, 1e3, 1e3};
  std::vector<double> weight_control_{1e3, 1e3};

  double wheel_base_;
};
}  // namespace IlqrPlanner
