#include <wobl_control/pid_balance_controller.hpp>

control_toolbox::AntiWindupStrategy create_aws() {
  control_toolbox::AntiWindupStrategy aws;
  aws.set_type("conditional_integration");
  return aws;
}
PidBalanceController::PidBalanceController()
    : aws_(create_aws()), pid_vel2pitch_(0.0, 0.0, 0.0, 10.0, -10.0, aws_),
      pid_pitch2vel_(0.0, 0.0, 0.0, 10.0, -10.0, aws_) {}

void PidBalanceController::set_vel2pitch_gains(double kp, double ki, double kd, double bound) {
  pid_vel2pitch_.initialize(kp, ki, kd, bound, -bound, aws_);
}

void PidBalanceController::set_pitch2vel_gains(double kp, double ki, double kd, double bound) {
  pid_pitch2vel_.initialize(kp, ki, kd, bound, -bound, aws_);
}

void PidBalanceController::set_target_velocities(double linear, double angular) {
  target_linear_velocity_ = linear;
  target_angular_velocity_ = angular;
}

void PidBalanceController::update(double linear_velocity, double pitch, double dt) {

  cmd_pitch_ = pid_vel2pitch_.compute_command(target_linear_velocity_ - linear_velocity, dt);
  cmd_linear_velocity_ = pid_pitch2vel_.compute_command(pitch - cmd_pitch_ - pitch_offset_, dt);
  cmd_angular_velocity_ = target_angular_velocity_;
}

double PidBalanceController::cmd_pitch() const { return cmd_pitch_; }

double PidBalanceController::cmd_linear_velocity() const { return cmd_linear_velocity_; }

double PidBalanceController::cmd_angular_velocity() const { return cmd_angular_velocity_; }

void PidBalanceController::vel2pitch_errors(double& ep, double& ei, double& ed) {
  pid_vel2pitch_.get_current_pid_errors(ep, ei, ed);
}

void PidBalanceController::pitch2vel_errors(double& ep, double& ei, double& ed) {
  pid_pitch2vel_.get_current_pid_errors(ep, ei, ed);
}