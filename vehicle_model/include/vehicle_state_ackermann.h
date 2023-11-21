#pragma once

#include <ackermann_msgs/msg/detail/ackermann_drive__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include "vehicle_state_interface.h"

namespace VehicleState {
class VehicleStateAckermann : public VehicleStateInterface {
 public:
  ackermann_msgs::msg::AckermannDrive expand_state_;
};
}  // namespace VehicleState
