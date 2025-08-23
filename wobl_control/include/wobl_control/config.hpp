#pragma once

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// Configuration structures
struct PidGains {
  double kp;
  double ki;
  double kd;

  PidGains(double p = 0.0, double i = 0.0, double d = 0.0) : kp(p), ki(i), kd(d) {}

  PidGains(const std::vector<double> &gains) {
    if (gains.size() >= 3) {
      kp = gains[0];
      ki = gains[1];
      kd = gains[2];
    } else {
      kp = ki = kd = 0.0;
    }
  }
};

struct ControllerConfig {
  // Kinematics parameters
  double wheel_radius;
  double wheel_separation;

  // Control limits
  double max_wheel_rps;
  double max_pitch;

  // Controller gains
  PidGains vel2pitch_gains;
  PidGains pitch2vel_gains;

  ControllerConfig()
      : wheel_radius(0.08), wheel_separation(0.3), max_wheel_rps(10.0), max_pitch(0.17),
        vel2pitch_gains(0.1, 0.1, 0.0), pitch2vel_gains(200.0, 0.0, 5.0) {}
};

// Parameter handler class
class ControllerParameterHandler {
public:
  explicit ControllerParameterHandler(rclcpp::Node *node) : node_(node), config_() {
    declare_parameters();
    update_config_from_parameters();
  }

  const ControllerConfig &get_config() const { return config_; }

  // Update configuration from current parameter values
  void update_config_from_parameters() {
    config_.wheel_radius = node_->get_parameter("wheel_radius").as_double();
    config_.wheel_separation = node_->get_parameter("wheel_separation").as_double();
    config_.max_wheel_rps = node_->get_parameter("max_wheel_rps").as_double();
    config_.max_pitch = node_->get_parameter("max_pitch").as_double();

    std::vector<double> vel2pitch = node_->get_parameter("vel2pitch_gains").as_double_array();
    std::vector<double> pitch2vel = node_->get_parameter("pitch2vel_gains").as_double_array();

    if (vel2pitch.size() >= 3) {
      config_.vel2pitch_gains = PidGains(vel2pitch);
    }

    if (pitch2vel.size() >= 3) {
      config_.pitch2vel_gains = PidGains(pitch2vel);
    }
  }

  // Handle parameter changes and return whether config was updated
  void handle_parameter_changes(const std::vector<rclcpp::Parameter> &parameters,
                                rcl_interfaces::msg::SetParametersResult &result) {
    
    for (const auto &param : parameters) {
      const std::string &name = param.get_name();

      if (name == "vel2pitch_gains" || name == "pitch2vel_gains") {
        // Validate array size
        if (param.as_double_array().size() != 3) {
          result.successful = false;
          result.reason = "PID gains must be an array of 3 values [kp, ki, kd]";
          return;
        }
      }
      
      // Apply parameter changes directly to our config
      if (name == "wheel_radius") {
        config_.wheel_radius = param.as_double();
      } else if (name == "wheel_separation") {
        config_.wheel_separation = param.as_double();
      } else if (name == "max_wheel_rps") {
        config_.max_wheel_rps = param.as_double();
      } else if (name == "max_pitch") {
        config_.max_pitch = param.as_double();
      } else if (name == "vel2pitch_gains") {
        config_.vel2pitch_gains = PidGains(param.as_double_array());
      } else if (name == "pitch2vel_gains") {
        config_.pitch2vel_gains = PidGains(param.as_double_array());
      }
    }
  }

private:
  void declare_parameters() {
    // Kinematics parameters
    node_->declare_parameter("wheel_radius", config_.wheel_radius);
    node_->declare_parameter("wheel_separation", config_.wheel_separation);

    // Control limits
    node_->declare_parameter("max_wheel_rps", config_.max_wheel_rps);
    node_->declare_parameter("max_pitch", config_.max_pitch);

    // Controller gains as arrays
    node_->declare_parameter(
        "vel2pitch_gains",
        std::vector<double>{config_.vel2pitch_gains.kp, config_.vel2pitch_gains.ki, config_.vel2pitch_gains.kd});

    node_->declare_parameter(
        "pitch2vel_gains",
        std::vector<double>{config_.pitch2vel_gains.kp, config_.pitch2vel_gains.ki, config_.pitch2vel_gains.kd});
  }

  rclcpp::Node *node_;
  ControllerConfig config_;
};
