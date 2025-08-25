#pragma once
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wobl_control/diff_drive_kinematics.hpp>
#include <wobl_control/linear_filter.hpp>
#include <wobl_msgs/msg/joint_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <wobl_control/wobl_config.hpp>

class WoblState {
public:
  WoblState(const WoblConfig &config, const DiffDriveKinematics &kinematics);

  bool ready() const;
  void update(sensor_msgs::msg::Imu::ConstSharedPtr imu);
  void update(sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
  void update(geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel);

  double pitch() const;
  double roll() const;
  double yaw() const;

  double linear_velocity() const;
  double pitch_rate() const;
  double yaw_rate() const;

  double target_linear_velocity() const;
  double target_yaw_rate() const;

  const DiffDriveKinematics &kinematics() const;
  const WoblConfig &config() const { return config_; }

private:
  const WoblConfig &config_;
  const DiffDriveKinematics &kinematics_;
  LinearFilter linear_velocity_, yaw_rate_;
  LinearFilter pitch_rate_;
  double pitch_, roll_, yaw_;
  double target_linear_velocity_, target_yaw_rate_;

  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  sensor_msgs::msg::JointState::ConstSharedPtr joint_state_;
};