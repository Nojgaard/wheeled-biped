#pragma once

#include "wobl_control/wobl_config.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

class ParameterHandler {
public:
  explicit ParameterHandler(rclcpp::Node *node) : node_(node) {
    declare_parameters();
    if (!validate_required_parameters()) {
      throw std::runtime_error("Failed to initialize ParameterHandler: Required parameters not found. Please ensure all parameters are loaded from a config file.");
    }
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

    RCLCPP_INFO(node_->get_logger(), "Parameters have been updated.");

    return result;
  }

private:
  void declare_parameters() {
    // Declare parameters with explicit types but no default values - will fail if not provided from config
    node_->declare_parameter("offset_pitch", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node_->declare_parameter("max_pitch", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node_->declare_parameter("max_wheel_rps", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node_->declare_parameter("pitch2vel_gains", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    node_->declare_parameter("vel2pitch_gains", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    node_->declare_parameter("lqr_K", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    node_->declare_parameter("wheel_radius", rclcpp::ParameterType::PARAMETER_DOUBLE);
    node_->declare_parameter("wheel_separation", rclcpp::ParameterType::PARAMETER_DOUBLE);
  }

  bool validate_required_parameters() {
    // Check if all required parameters are set
    std::vector<std::string> required_params = {
      "offset_pitch", "max_pitch", "max_wheel_rps", 
      "pitch2vel_gains", "vel2pitch_gains", "lqr_K",
      "wheel_radius", "wheel_separation"
    };

    for (const auto& param_name : required_params) {
      if (!node_->has_parameter(param_name)) {
        RCLCPP_ERROR(node_->get_logger(), "Required parameter '%s' not found", param_name.c_str());
        return false;
      }
    }

    // Validate array sizes
    if (node_->get_parameter("pitch2vel_gains").as_double_array().size() != 3) {
      RCLCPP_ERROR(node_->get_logger(), "Parameter 'pitch2vel_gains' must be an array of 3 values [kp, ki, kd]");
      return false;
    }

    if (node_->get_parameter("vel2pitch_gains").as_double_array().size() != 3) {
      RCLCPP_ERROR(node_->get_logger(), "Parameter 'vel2pitch_gains' must be an array of 3 values [kp, ki, kd]");
      return false;
    }

    if (node_->get_parameter("lqr_K").as_double_array().size() != 4) {
      RCLCPP_ERROR(node_->get_logger(), "Parameter 'lqr_K' must be an array of 4 values");
      return false;
    }

    return true;
  }

  rclcpp::Node *node_;
  WoblConfig config_;
};
