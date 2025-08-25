#include <wobl_control/wobl_command.hpp>

WoblCommand::WoblCommand(const DiffDriveKinematics &kinematics)
    : kinematics_(kinematics) {
  joint_command_.position.resize(4, 0.0);
  joint_command_.velocity.resize(4, 0.0);

  joint_command_.velocity[0] = 1.0;
  joint_command_.velocity[1] = 1.0;
}

const wobl_msgs::msg::JointCommand &WoblCommand::to_joint_commands() {
  auto [left_wheel_rps, right_wheel_rps] = kinematics_.inverse_kinematics(linear_velocity, yaw_rate);
  joint_command_.velocity[2] = left_wheel_rps;
  joint_command_.velocity[3] = right_wheel_rps;
  return joint_command_;
}