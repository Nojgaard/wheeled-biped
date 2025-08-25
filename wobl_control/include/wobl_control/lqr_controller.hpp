#pragma once

#include <vector>
#include <wobl_control/wobl_state.hpp>

class LqrController {
public:
  LqrController(const WoblState &state);

  const wobl_msgs::msg::JointCommand &update(double dt);
  double cmd_velocity() const { return cmd_velocity_; }
  double cmd_yaw_rate() const { return cmd_yaw_rate_; }

private:
  wobl_msgs::msg::JointCommand cmd_joints_;
  const WoblState &state_;
  double cmd_velocity_;
  double cmd_yaw_rate_;
  double velocity_integral_;
};