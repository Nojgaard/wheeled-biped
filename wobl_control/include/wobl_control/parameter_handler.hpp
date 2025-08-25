#pragma once

#include "wobl_control/wobl_config.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

class ParameterHandler {
public:
  explicit ParameterHandler(rclcpp::Node *node) : node_(node) {
    declare_parameters();
    refresh_from_parameters();
  }

  const WoblConfig &config() const { return config_; }

  void refresh_from_parameters() {
    config_.offset_pitch = node_->get_parameter("offset_pitch").as_double();
    config_.max_pitch = node_->get_parameter("max_pitch").as_double();
    config_.max_wheel_rps = node_->get_parameter("max_wheel_rps").as_double();
    config_.pitch2vel_gains = node_->get_parameter("pitch2vel_gains").as_double_array();
    config_.vel2pitch_gains = node_->get_parameter("vel2pitch_gains").as_double_array();
    config_.lqr_K = node_->get_parameter("lqr_K").as_double_array();
    config_.wheel_radius = node_->get_parameter("wheel_radius").as_double();
    config_.wheel_separation = node_->get_parameter("wheel_separation").as_double();
  }

  rcl_interfaces::msg::SetParametersResult handle_parameter_changes(const std::vector<rclcpp::Parameter> &parameters) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters) {
      const std::string &name = param.get_name();

      // Validate parameter arrays
      if ((name == "pitch2vel_gains" || name == "vel2pitch_gains") && param.as_double_array().size() != 3) {
        result.successful = false;
        result.reason = "PID gains must be an array of 3 values [kp, ki, kd]";
        return result;
      }

      if (name == "lqr_K" && param.as_double_array().size() != 4) {
        result.successful = false;
        result.reason = "LQR_K must be an array of 4 values";
        return result;
      }

      // Apply parameter changes directly to config
      if (name == "offset_pitch")
        config_.offset_pitch = param.as_double();
      else if (name == "max_pitch")
        config_.max_pitch = param.as_double();
      else if (name == "max_wheel_rps")
        config_.max_wheel_rps = param.as_double();
      else if (name == "pitch2vel_gains")
        config_.pitch2vel_gains = param.as_double_array();
      else if (name == "vel2pitch_gains")
        config_.vel2pitch_gains = param.as_double_array();
      else if (name == "lqr_K")
        config_.lqr_K = param.as_double_array();
      else if (name == "wheel_radius")
        config_.wheel_radius = param.as_double();
      else if (name == "wheel_separation")
        config_.wheel_separation = param.as_double();
    }

    return result;
  }

private:
  void declare_parameters() {
    // Default configuration
    WoblConfig defaults;
    defaults.offset_pitch = 0.0;
    defaults.max_pitch = 0.17;
    defaults.max_wheel_rps = 10.0;
    defaults.pitch2vel_gains = {200.0, 0.0, 5.0};
    defaults.vel2pitch_gains = {0.1, 0.1, 0.0};
    defaults.lqr_K = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Default zeros for LQR gain matrix
    defaults.wheel_radius = 0.04;
    defaults.wheel_separation = 0.3;

    // Declare all parameters with defaults
    node_->declare_parameter("offset_pitch", defaults.offset_pitch);
    node_->declare_parameter("max_pitch", defaults.max_pitch);
    node_->declare_parameter("max_wheel_rps", defaults.max_wheel_rps);
    node_->declare_parameter("pitch2vel_gains", defaults.pitch2vel_gains);
    node_->declare_parameter("vel2pitch_gains", defaults.vel2pitch_gains);
    node_->declare_parameter("lqr_K", defaults.lqr_K);
    node_->declare_parameter("wheel_radius", defaults.wheel_radius);
    node_->declare_parameter("wheel_separation", defaults.wheel_separation);
  }

  rclcpp::Node *node_;
  WoblConfig config_;
};
