#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <wobl_hardware/servo_driver.hpp>
#include <wobl_messages/msg/battery_state.hpp>
#include <wobl_messages/msg/joint_command.hpp>

using BatteryState = wobl_messages::msg::BatteryState;
using JointState = sensor_msgs::msg::JointState;
using JointCommand = wobl_messages::msg::JointCommand;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using Bool = std_msgs::msg::Bool;

enum JointToId {
  HIP_LEFT = 0,
  HIP_RIGHT = 5,
  WHEEL_LEFT = 2,
  WHEEL_RIGHT = 6,
};

class ServoNode : public rclcpp::Node {
public:
  ServoNode() : Node("servo_node") {
    servo_state_pub_ = this->create_publisher<JointState>("joint_states", rclcpp::SensorDataQoS());
    battery_pub_ = this->create_publisher<BatteryState>("battery_state", rclcpp::SensorDataQoS());
    diagnostic_pub_ = this->create_publisher<DiagnosticStatus>("servo/status", 10);

    if (!initialize_driver())
      return;

    servo_cmd_sub_ = this->create_subscription<JointCommand>("joint_commands", 1,
                                                             [this](JointCommand::SharedPtr msg) { cmd_next_ = msg; });
    servo_enabled_sub_ = this->create_subscription<Bool>(
        "servo/enabled", 1, [this](Bool::SharedPtr msg) { this->enable_servos(msg->data); });

    last_battery_pub_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { loop(); });
  }

  void enable_servos(bool enable) {
    for (u8 id : servo_ids_) {
      if (!driver_.enable_torque(id, enable)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set torque for servo with ID %d", id);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Torque %s for all servos", enable ? "enabled" : "disabled");
    is_torque_enabled_ = enable;
  }

  bool initialize_driver() {
    if (!driver_.initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo driver.");
      return false;
    }

    for (u8 id : servo_ids_) {
      if (!driver_.ping(id)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to ping servo with ID %d", id);
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Servo with ID %d is online", id);
    }

    driver_.set_mode(HIP_LEFT, ServoDriver::POSITION);
    driver_.set_mode(HIP_RIGHT, ServoDriver::POSITION);
    driver_.set_mode(WHEEL_LEFT, ServoDriver::VELOCITY);
    driver_.set_mode(WHEEL_RIGHT, ServoDriver::VELOCITY);

    enable_servos(false);

    cmd_next_ = std::make_shared<JointCommand>();
    cmd_next_->position.resize(4, 100.0);
    cmd_next_->velocity.resize(4, 0.0);

    cmd_cur_ = cmd_next_;

    RCLCPP_INFO(this->get_logger(), "Servo driver initialized successfully.");

    DiagnosticStatus msg;
    msg.level = DiagnosticStatus::OK;
    msg.name = "Servo Driver";
    msg.message = "Initialized successfully.";
    msg.hardware_id = "servo_driver";
    diagnostic_pub_->publish(msg);
    return true;
  }

  bool has_pending_command(long idx) {
    return cmd_next_->position[idx] != cmd_cur_->position[idx] || cmd_next_->velocity[idx] != cmd_cur_->velocity[idx];
  }

  void loop() {
    write_commands();
    publish_state();
  }

private:
  void write_commands() {
    for (size_t i = 0; i < servo_ids_.size(); ++i) {
      if (!has_pending_command(i)) {
        continue;
      }
      u8 id = servo_ids_[i];
      float mirror_scalar = (id == HIP_RIGHT || id == WHEEL_RIGHT) ? 1.0f : -1.0f;
      if (id == HIP_LEFT || id == HIP_RIGHT) {
        driver_.write_position(id, mirror_scalar * cmd_next_->position[i], cmd_next_->velocity[i]);
      } else if (id == WHEEL_LEFT || id == WHEEL_RIGHT) {
        driver_.write_velocity(id, mirror_scalar * cmd_next_->velocity[i], 200.0);
      }
    }
    cmd_cur_ = cmd_next_;
  }

  void publish_state() {
    auto current_time = this->now();
    JointState state_msg;
    state_msg.header.stamp = current_time;
    // state_msg.name = {"hip_left", "hip_right", "wheel_left", "wheel_right"};
    state_msg.position.resize(4);
    state_msg.velocity.resize(4);
    state_msg.effort.resize(4);

    BatteryState battery_msg;
    for (size_t i = 0; i < servo_ids_.size(); ++i) {
      auto servo_state = driver_.read_state(servo_ids_[i]);
      state_msg.position[i] = servo_state.position_rad;
      state_msg.velocity[i] = servo_state.velocity_rps;
      state_msg.effort[i] = servo_state.current_amps;
      battery_msg.voltage = servo_state.voltage_volts;
    }

    if ((current_time - last_battery_pub_time_).seconds() >= 1.0) {
      battery_pub_->publish(battery_msg);
      last_battery_pub_time_ = current_time;
    }
    servo_state_pub_->publish(state_msg);
  }

  ServoDriver driver_;
  const std::vector<uint8_t> servo_ids_ = {HIP_LEFT, HIP_RIGHT, WHEEL_LEFT, WHEEL_RIGHT};
  bool is_torque_enabled_;

  rclcpp::Time last_battery_pub_time_;
  JointCommand::SharedPtr cmd_next_;
  JointCommand::SharedPtr cmd_cur_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr servo_enabled_sub_;
  rclcpp::Subscription<JointCommand>::SharedPtr servo_cmd_sub_;
  rclcpp::Publisher<JointState>::SharedPtr servo_state_pub_;
  rclcpp::Publisher<BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoNode>();
  rclcpp::spin(node);
  node->enable_servos(false);
  rclcpp::shutdown();
  return 0;
}