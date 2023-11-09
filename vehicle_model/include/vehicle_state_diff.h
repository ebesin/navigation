#pragma once

#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include "vehicle_state_interface.h"

namespace VehicleState {
class VehicleStateDiff : public VehicleStateInterface {
 public:
  nav_msgs::msg::Odometry base_state_;

  ~VehicleStateDiff() {}
};
}  // namespace VehicleState
