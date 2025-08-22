#pragma once
#include <utility>

class DiffDriveKinematics {
public:
  DiffDriveKinematics(double wheel_radius, double wheel_seperation, double wheel_max_rps)
      : wheel_radius_(wheel_radius), wheel_seperation_(wheel_seperation), wheel_max_rps_(wheel_max_rps) {}

  std::pair<double, double> inverse_kinematics(double linear_velocity, double angular_velocity) const {
    double left_wheel_velocity = (linear_velocity - (angular_velocity * wheel_seperation_ / 2.0)) / wheel_radius_;
    double right_wheel_velocity = (linear_velocity + (angular_velocity * wheel_seperation_ / 2.0)) / wheel_radius_;
    return {left_wheel_velocity, right_wheel_velocity};
  }

  std::pair<double, double> forward_kinematics(double left_wheel_velocity, double right_wheel_velocity) const {
    double linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double angular_velocity = (right_wheel_velocity - left_wheel_velocity) * wheel_radius_ / wheel_seperation_;
    return {linear_velocity, angular_velocity};
  }

  double max_linear_velocity() const {
    return wheel_max_rps_ * wheel_radius_;
  }

private:
  double wheel_radius_;
  double wheel_seperation_;
  double wheel_max_rps_;
};