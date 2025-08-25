#pragma once
#include <wobl_control/diff_drive_kinematics.hpp>
#include <wobl_msgs/msg/joint_command.hpp>

struct WoblCommand {
public:
  WoblCommand(const DiffDriveKinematics &kinematics);
  const wobl_msgs::msg::JointCommand &to_joint_commands();

  double linear_velocity;
  double yaw_rate;

private:
  const DiffDriveKinematics &kinematics_;
  wobl_msgs::msg::JointCommand joint_command_;
};