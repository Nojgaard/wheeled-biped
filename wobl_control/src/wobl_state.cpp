#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wobl_control/wobl_state.hpp>

WoblState::WoblState(const WoblConfig &config, const DiffDriveKinematics &kinematics)
    : config_(config), kinematics_(kinematics), linear_velocity_(0.001, 0.02), pitch_rate_(0.5), yaw_rate_(0.2) {}

bool WoblState::ready() const { return imu_ != nullptr && joint_state_ != nullptr; }

void WoblState::update(sensor_msgs::msg::Imu::ConstSharedPtr imu) {
  imu_ = imu;
  if (imu_ == nullptr)
    return;
  tf2::Quaternion quat(imu_->orientation.x, imu_->orientation.y, imu_->orientation.z, imu_->orientation.w);
  tf2::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);

  pitch_rate_.update(imu_->angular_velocity.y);
}

void WoblState::update(sensor_msgs::msg::JointState::ConstSharedPtr joint_state) {
  joint_state_ = joint_state;
  if (joint_state_ == nullptr)
    return;

  auto [linear_vel, angular_vel] = kinematics_.forward_kinematics(joint_state_->velocity[2], joint_state_->velocity[3]);
  linear_velocity_.predict_and_update(linear_vel);
  yaw_rate_.update(angular_vel);
}

void WoblState::update(geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel) {
  if (cmd_vel == nullptr)
    return;

  target_linear_velocity_ = cmd_vel->linear.x;
  target_yaw_rate_ = cmd_vel->angular.z;
}

double WoblState::target_linear_velocity() const { return target_linear_velocity_; }

double WoblState::target_yaw_rate() const { return target_yaw_rate_; }

double WoblState::pitch() const { return pitch_; }

double WoblState::roll() const { return roll_; }

double WoblState::yaw() const { return yaw_; }

double WoblState::linear_velocity() const { return linear_velocity_.state(); }

double WoblState::pitch_rate() const { return pitch_rate_.value(); }

double WoblState::yaw_rate() const { return yaw_rate_.value(); }

const DiffDriveKinematics &WoblState::kinematics() const { return kinematics_; }
