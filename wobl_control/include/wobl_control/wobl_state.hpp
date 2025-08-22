#pragma once
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wobl_control/diff_drive_kinematics.hpp>
#include <wobl_control/linear_filter.hpp>
#include <wobl_msgs/msg/joint_command.hpp>

class WoblState {
public:
  WoblState(const DiffDriveKinematics &kinematics);

  bool ready() const;
  void update(sensor_msgs::msg::Imu::ConstSharedPtr imu);
  void update(sensor_msgs::msg::JointState::ConstSharedPtr joint_state);

  double pitch() const;
  double roll() const;
  double yaw() const;

  double linear_velocity() const;
  double angular_velocity() const;

private:
  const DiffDriveKinematics &kinematics_;
  LinearFilter linear_velocity_, angular_velocity_;
  double pitch_, roll_, yaw_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  sensor_msgs::msg::JointState::ConstSharedPtr joint_state_;
  wobl_msgs::msg::JointCommand joint_command_;
};