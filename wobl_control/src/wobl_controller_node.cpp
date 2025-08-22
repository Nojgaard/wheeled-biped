#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wobl_control/config.hpp>
#include <wobl_control/diff_drive_kinematics.hpp>
#include <wobl_control/pid_balance_controller.hpp>
#include <wobl_control/wobl_command.hpp>
#include <wobl_control/wobl_state.hpp>
#include <wobl_msgs/msg/joint_command.hpp>
#include <wobl_msgs/msg/topics.hpp>

using JointState = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;
using JointCommand = wobl_msgs::msg::JointCommand;
using Topics = wobl_msgs::msg::Topics;

class WoblControllerNode : public rclcpp::Node {
public:
  WoblControllerNode() : Node("wobl_controller"), params_(this) {
    // Initialize objects with parameters
    init_objects();

    // Set up subscribers, publishers, etc.
    joint_state_subscriber_ =
        this->create_subscription<JointState>(Topics::JOINT_STATE, rclcpp::SensorDataQoS(),
                                              [this](JointState::ConstSharedPtr msg) { wobl_state_->update(msg); });

    imu_subscriber_ = this->create_subscription<Imu>(Topics::IMU, rclcpp::SensorDataQoS(),
                                                     [this](Imu::ConstSharedPtr msg) { wobl_state_->update(msg); });

    command_publisher_ = this->create_publisher<JointCommand>(Topics::JOINT_COMMAND, 1);
    last_time_ = this->now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&WoblControllerNode::update, this));

    // Add parameter callback to handle parameter changes
    param_callback_handle_ =
        add_on_set_parameters_callback(std::bind(&WoblControllerNode::parameters_callback, this, std::placeholders::_1));
  }

private:
  void init_objects() {
    // Get current configuration
    const auto &config = params_.get_config();

    // Construct objects with proper initialization
    kinematics_ =
        std::make_unique<DiffDriveKinematics>(config.wheel_radius, config.wheel_separation, config.max_wheel_rps);
    wobl_state_ = std::make_unique<WoblState>(*kinematics_);
    wobl_command_ = std::make_unique<WoblCommand>(*kinematics_);
    pid_controller_ = std::make_unique<PidBalanceController>();

    RCLCPP_INFO(get_logger(),
                "Initialized controller with wheel radius: %.2f, wheel separation: %.2f, max wheel rps: %.2f",
                config.wheel_radius, config.wheel_separation, config.max_wheel_rps);

    // Initialize controller with current parameters
    update_controller_configuration();
  }

  void update_controller_configuration() {
    if (!pid_controller_) {
      RCLCPP_ERROR(get_logger(), "Cannot update configuration: PID controller not initialized");
      return;
    }

    const auto &config = params_.get_config();
    double max_velocity = kinematics_->max_linear_velocity();

    // Update controller gains
    pid_controller_->set_vel2pitch_gains(config.vel2pitch_gains.kp, config.vel2pitch_gains.ki,
                                         config.vel2pitch_gains.kd, config.max_pitch);

    pid_controller_->set_pitch2vel_gains(config.pitch2vel_gains.kp, config.pitch2vel_gains.ki,
                                         config.pitch2vel_gains.kd, max_velocity);

    // Log updated configuration
    RCLCPP_INFO(get_logger(), "Updated controller configuration:");
    RCLCPP_INFO(get_logger(), "Velocity to Pitch gains: kp=%.2f, ki=%.2f, kd=%.2f", config.vel2pitch_gains.kp,
                config.vel2pitch_gains.ki, config.vel2pitch_gains.kd);
    RCLCPP_INFO(get_logger(), "Pitch to Velocity gains: kp=%.2f, ki=%.2f, kd=%.2f", config.pitch2vel_gains.kp,
                config.pitch2vel_gains.ki, config.pitch2vel_gains.kd);
    RCLCPP_INFO(get_logger(), "Max velocity: %.2f, Max pitch: %.2f", max_velocity, config.max_pitch);
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    // Let parameter handler process the changes
    params_.handle_parameter_changes(parameters, result);

    if (result.successful)
      update_controller_configuration();

    return result;
  }

  void update() {
    static rclcpp::Time last_warn_time = this->now();
    rclcpp::Time current_time = this->now();
    if (!wobl_state_->ready()) {
      if ((current_time - last_warn_time).seconds() > 1.0) {
        RCLCPP_WARN(get_logger(), "WoblState not ready, waiting for IMU and JointState messages");
        last_warn_time = current_time;
      }
      last_time_ = current_time;
      return;
    }

    rclcpp::Duration dt = current_time - last_time_;
    last_time_ = current_time;
    if (dt.seconds() < 0)
      return;

    pid_controller_->update(wobl_state_->linear_velocity(), wobl_state_->pitch(), dt.seconds());
    wobl_command_->linear_velocity = pid_controller_->cmd_linear_velocity();
    wobl_command_->angular_velocity = pid_controller_->cmd_angular_velocity();

    command_publisher_->publish(wobl_command_->to_joint_commands());
  }

  // Parameter handler
  ControllerParameterHandler params_;

  // Use smart pointers to enable proper initialization after construction
  std::unique_ptr<DiffDriveKinematics> kinematics_;
  std::unique_ptr<WoblCommand> wobl_command_;
  std::unique_ptr<WoblState> wobl_state_;
  std::unique_ptr<PidBalanceController> pid_controller_;

  rclcpp::Time last_time_;

  rclcpp::Subscription<JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WoblControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}