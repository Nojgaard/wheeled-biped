#include "wobl_controllers/pid_balance_controller.hpp"

controller_interface::CallbackReturn PidBalanceController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "PidBalanceController: on_init");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PidBalanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "PidBalanceController: on_configure");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PidBalanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "PidBalanceController: on_activate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PidBalanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "PidBalanceController: on_deactivate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PidBalanceController::update(const rclcpp::Time & /*time*/,
                                                               const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO(get_node()->get_logger(), "PidBalanceController: update");
  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration PidBalanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  //config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  //config.names = {"wheel_left_joint/velocity", "wheel_right_joint/velocity"};
  return config;
}

controller_interface::InterfaceConfiguration PidBalanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  //config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  //config.names = {"imu/angle", "imu/angular_velocity"};
  return config;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(PidBalanceController, controller_interface::ControllerInterface)
