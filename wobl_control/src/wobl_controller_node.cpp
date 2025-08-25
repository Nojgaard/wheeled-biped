#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wobl_control/diff_drive_kinematics.hpp>
#include <wobl_control/lqr_controller.hpp>
#include <wobl_control/parameter_handler.hpp>
#include <wobl_control/pid_balance_controller.hpp>
#include <wobl_control/wobl_command.hpp>
#include <wobl_control/wobl_state.hpp>
#include <wobl_msgs/msg/controller_inputs.hpp>
#include <wobl_msgs/msg/joint_command.hpp>
#include <wobl_msgs/msg/topics.hpp>

using JointState = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;
using JointCommand = wobl_msgs::msg::JointCommand;
using Topics = wobl_msgs::msg::Topics;
using ControllerInputs = wobl_msgs::msg::ControllerInputs;
using Twist = geometry_msgs::msg::Twist;

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
    velocity_command_subscriber_ = this->create_subscription<Twist>(
        Topics::VELOCITY_COMMAND, 10, [this](Twist::ConstSharedPtr msg) { wobl_state_->update(msg); });

    command_publisher_ = this->create_publisher<JointCommand>(Topics::JOINT_COMMAND, 1);
    controller_inputs_publisher_ = this->create_publisher<ControllerInputs>(Topics::CONTROLLER_INPUTS, 10);
    last_time_ = this->now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&WoblControllerNode::update, this));

    // Add parameter callback to handle parameter changes
    param_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ParameterHandler::handle_parameter_changes, &params_, std::placeholders::_1));
  }

private:
  void init_objects() {
    // Get current configuration
    const auto &config = params_.config();

    // Construct objects with proper initialization
    kinematics_ =
        std::make_unique<DiffDriveKinematics>(config.wheel_radius, config.wheel_separation, config.max_wheel_rps);
    wobl_state_ = std::make_unique<WoblState>(config, *kinematics_);
    lqr_controller_ = std::make_unique<LqrController>(*wobl_state_);

    RCLCPP_INFO(get_logger(),
                "Initialized controller with wheel radius: %.2f, wheel separation: %.2f, max wheel rps: %.2f",
                config.wheel_radius, config.wheel_separation, config.max_wheel_rps);
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
    if (dt.seconds() < 0 || dt.seconds() > 0.03)
      return;

    const auto &cmd_joints = lqr_controller_->update(dt.seconds());
    command_publisher_->publish(cmd_joints);

    controller_inputs_.cmd_linear_vel = lqr_controller_->cmd_velocity();
    //controller_inputs_.cmd_angular_vel = lqr_controller_->cmd_yaw_rate();
    controller_inputs_.est_pitch = wobl_state_->pitch();
    controller_inputs_.cmd_angular_vel = wobl_state_->pitch_rate();
    controller_inputs_.est_linear_vel = wobl_state_->linear_velocity();
    controller_inputs_.est_angular_vel = wobl_state_->yaw_rate();

    controller_inputs_publisher_->publish(controller_inputs_);
  }

  // Parameter handler
  ParameterHandler params_;
  ControllerInputs controller_inputs_;

  // Use smart pointers to enable proper initialization after construction
  std::unique_ptr<DiffDriveKinematics> kinematics_;
  std::unique_ptr<WoblState> wobl_state_;
  std::unique_ptr<LqrController> lqr_controller_;

  rclcpp::Time last_time_;

  rclcpp::Subscription<JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_;
  rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<ControllerInputs>::SharedPtr controller_inputs_publisher_;
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