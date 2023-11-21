#include "ilqr_planner.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IlqrPlanner::IlqrPlanner>("ilqr_planner"));
  rclcpp::shutdown();
  return 0;
}
