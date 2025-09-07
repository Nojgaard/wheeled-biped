#include <wobl_control/lqr_controller.hpp>

LqrController::LqrController(const WoblState &state) : state_(state) {
  cmd_joints_.position.resize(4, 0.0);
  cmd_joints_.velocity.resize(4, 0.0);

  cmd_joints_.velocity[0] = 1.0;
  cmd_joints_.velocity[1] = 1.0;
}

const wobl_msgs::msg::JointCommand &LqrController::update(double dt) {
  const auto &config = state_.config();

  // state feedback gains
  double k_pitch = config.lqr_K[0];
  double k_pitch_rate = config.lqr_K[1];
  double k_vel = config.lqr_K[2];
  double k_pos = config.lqr_K[3];

  double pitch_error = state_.pitch() - config.offset_pitch;
  double pitch_rate_error = state_.pitch_rate();
  double vel_error = state_.target_linear_velocity() - state_.linear_velocity();

  // Reset integral if target velocity changed significantly (> 0.05 m/s)
  if (std::abs(state_.target_linear_velocity() - prev_target_velocity_) > 0.05) {
    velocity_integral_ = 0.0;
  }
  prev_target_velocity_ = state_.target_linear_velocity();

  // velocity integral with anti-windup
  velocity_integral_ += vel_error * dt;
  velocity_integral_ = std::clamp(velocity_integral_, -0.2, 0.2);

  cmd_velocity_ =
      -(k_pitch * pitch_error + k_pitch_rate * pitch_rate_error + k_vel * vel_error + k_pos * velocity_integral_);
  cmd_yaw_rate_ = state_.target_yaw_rate();

  auto [cmd_left_wheel, cmd_right_wheel] = state_.kinematics().inverse_kinematics(cmd_velocity_, cmd_yaw_rate_);
  cmd_joints_.velocity[2] = std::clamp(cmd_left_wheel, -config.max_wheel_rps, config.max_wheel_rps);
  cmd_joints_.velocity[3] = std::clamp(cmd_right_wheel, -config.max_wheel_rps, config.max_wheel_rps);

  return cmd_joints_;
}
