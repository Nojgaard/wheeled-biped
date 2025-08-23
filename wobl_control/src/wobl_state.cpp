#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wobl_control/wobl_state.hpp>

WoblState::WoblState(const DiffDriveKinematics &kinematics)
    : kinematics_(kinematics), linear_velocity_(0.2), angular_velocity_(0.2), pitch_(0.0), roll_(0.0), yaw_(0.0) {}

bool WoblState::ready() const { return imu_ != nullptr && joint_state_ != nullptr; }

void WoblState::update(sensor_msgs::msg::Imu::ConstSharedPtr imu) {
  imu_ = imu;
  if (imu_ == nullptr)
    return;
  tf2::Quaternion quat(imu_->orientation.x, imu_->orientation.y, imu_->orientation.z, imu_->orientation.w);
  tf2::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
}

void WoblState::update(sensor_msgs::msg::JointState::ConstSharedPtr joint_state) {
  joint_state_ = joint_state;
  if (joint_state_ == nullptr)
    return;

  auto [linear_vel, angular_vel] = kinematics_.forward_kinematics(joint_state_->velocity[2], joint_state_->velocity[3]);
  linear_velocity_.update(linear_vel);
  angular_velocity_.update(angular_vel);
}

double WoblState::pitch() const { return pitch_; }

double WoblState::roll() const { return roll_; }

double WoblState::yaw() const { return yaw_; }

double WoblState::linear_velocity() const { return linear_velocity_.value(); }

double WoblState::angular_velocity() const { return angular_velocity_.value(); }