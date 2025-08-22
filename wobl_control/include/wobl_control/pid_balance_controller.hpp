#pragma once
#include <control_toolbox/pid.hpp>

class PidBalanceController {
public:
  PidBalanceController();

  void set_target_velocities(double linear, double angular);
  void set_vel2pitch_gains(double kp, double ki, double kd, double bound);
  void set_pitch2vel_gains(double kp, double ki, double kd, double bound);
  
  
  void update(double linear_velocity, double pitch, double dt);

  double cmd_pitch() const;
  double cmd_linear_velocity() const;
  double cmd_angular_velocity() const;

private:
  control_toolbox::AntiWindupStrategy aws_;
  control_toolbox::Pid pid_vel2pitch_;
  control_toolbox::Pid pid_pitch2vel_;
  double cmd_pitch_, cmd_linear_velocity_, cmd_angular_velocity_;

  double target_linear_velocity_;
  double target_angular_velocity_;
  double pitch_offset_ = 0.035;
};