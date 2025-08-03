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

enum JointToId {
  HIP_LEFT = 0,
  HIP_RIGHT = 1,
  WHEEL_LEFT = 2,
  WHEEL_RIGHT = 3,
};

class ServoNode : public rclcpp::Node {
public:
  ServoNode() : Node("servo_node") {
    if (!initialize_driver())
      return;

    cmd_sub_ = this->create_subscription<JointCommand>("joint_commands", 1,
                                                       [this](JointCommand::SharedPtr msg) { cmd_next_ = msg; });
    enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>("servo/enabled", 1,
                                                                  [this](std_msgs::msg::Bool::SharedPtr msg) { next_is_enabled_ = msg->data; });

    state_pub_ = this->create_publisher<JointState>("joint_states", rclcpp::SensorDataQoS());
    battery_pub_ = this->create_publisher<BatteryState>("battery_state", rclcpp::SensorDataQoS());
    diagnostic_pub_ = this->create_publisher<DiagnosticStatus>("servo/status", 10);

    last_battery_pub_time = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { loop(); });
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

    for (u8 id : servo_ids_) {
      if (!driver_.enable_torque(id, false)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set torque for servo with ID %d", id);
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Torque disabled for servo with ID %d", id);
    }

    cmd_next_ = std::make_shared<JointCommand>();
    cmd_next_->position.resize(4, 0.0);
    cmd_next_->velocity.resize(4, 0.0);

    cmd_prev_ = cmd_next_;

    RCLCPP_INFO(this->get_logger(), "Servo driver initialized successfully.");

    DiagnosticStatus msg;
    msg.level = DiagnosticStatus::OK;
    msg.name = "Servo Driver";
    msg.message = "Initialized successfully.";
    msg.hardware_id = "servo_driver";
    diagnostic_pub_->publish(msg);
    return true;
  }

  bool has_pending_command(u8 id) {
    return cmd_next_->position[id] != cmd_prev_->position[id] || cmd_next_->velocity[id] != cmd_prev_->velocity[id];
  }

  void loop() {
    for (u8 id : servo_ids_) {
      if (!has_pending_command(id)) {
        continue;
      }

      if (next_is_enabled_ != is_enabled_) {
        if (!driver_.enable_torque(id, next_is_enabled_)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set torque for servo with ID %d", id);
        }
      }

      if (id == HIP_LEFT || id == HIP_RIGHT) {
        driver_.write_position(id, cmd_next_->position[id], cmd_next_->velocity[id]);
      } else if (id == WHEEL_LEFT || id == WHEEL_RIGHT) {
        driver_.write_velocity(id, cmd_next_->velocity[id]);
      }
    }

    if (next_is_enabled_ != is_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Torque %s for all servos", next_is_enabled_ ? "enabled" : "disabled");
      is_enabled_ = next_is_enabled_;
    }

    cmd_prev_ = cmd_next_;

    auto current_time = this->now();
    JointState state_msg;
    state_msg.header.stamp = current_time;
    // state_msg.name = {"hip_left", "hip_right", "wheel_left", "wheel_right"};
    state_msg.position.resize(4);
    state_msg.velocity.resize(4);
    state_msg.effort.resize(4);

    BatteryState battery_msg;
    for (u8 id : servo_ids_) {
      auto servo_state = driver_.read_state(id);
      state_msg.position[id] = servo_state.position_rad;
      state_msg.velocity[id] = servo_state.velocity_rps;
      state_msg.effort[id] = servo_state.current_amps;
      battery_msg.voltage = servo_state.voltage_volts;
    }

    if ((current_time - last_battery_pub_time).seconds() >= 1.0) {
      battery_pub_->publish(battery_msg);
      rclcpp::Time last_battery_pub_time = current_time;
    }

    state_pub_->publish(state_msg);
  }

private:
  ServoDriver driver_;
  std::vector<uint8_t> servo_ids_ = {HIP_LEFT, HIP_RIGHT, WHEEL_LEFT, WHEEL_RIGHT};
  bool is_enabled_, next_is_enabled_;

  rclcpp::Time last_battery_pub_time;
  JointCommand::SharedPtr cmd_next_;
  JointCommand::SharedPtr cmd_prev_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Subscription<JointCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}