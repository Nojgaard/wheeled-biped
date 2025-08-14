#include <atomic>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <wobl_messages/msg/joint_command.hpp>

using JointState = sensor_msgs::msg::JointState;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using JointCommand = wobl_messages::msg::JointCommand;

enum JointToId { HIP_LEFT = 0, HIP_RIGHT = 1, WHEEL_LEFT = 2, WHEEL_RIGHT = 3 };

std::atomic<bool> user_quit{false};

void setup_terminal() {
  termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

void restore_terminal() {
  termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag |= (ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

bool wait_for_enter_or_quit() {
  std::cout << "Press [Enter] to continue, or 'q' to quit..." << std::endl;
  while (true) {
    if (user_quit)
      return false;
    int c = getchar();
    std::cout << c << std::endl;
    if (c == '\n')
      return true;
    if (c == 'q' || c == 'Q') {
      user_quit = true;
      return false;
    }
  }
}

class RobotTest : public rclcpp::Node {
public:
  RobotTest() : Node("robot_test") {
    joint_state_sub_ = this->create_subscription<JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [this](JointState::SharedPtr msg) { last_joint_state_ = msg; });

    imu_status_sub_ = this->create_subscription<DiagnosticStatus>(
        "imu/status", rclcpp::SensorDataQoS(), [this](DiagnosticStatus::SharedPtr msg) { imu_status_ = *msg; });

    servo_status_sub_ = this->create_subscription<DiagnosticStatus>(
        "servo/status", 10, [this](DiagnosticStatus::SharedPtr msg) { servo_status_ = *msg; });

    cmd_pub_ = this->create_publisher<JointCommand>("joint_commands", 10);
    servo_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("servo/enabled", 10);
  }

  void run() {
    // Wait for status OK from both nodes
    std::cout << "Waiting for IMU and Servo node status OK..." << std::endl;
    while (rclcpp::ok() && !user_quit) {
      rclcpp::spin_some(this->get_node_base_interface());
      if (imu_status_.level == DiagnosticStatus::OK && servo_status_.level == DiagnosticStatus::OK &&
          last_joint_state_ != nullptr) {
        std::cout << "IMU and Servo nodes are OK." << std::endl;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (user_quit)
      return;

    enable_servos(true);
    // Print joint states
    std::cout << "Current joint states:" << std::endl;
    for (int i = 0; i < 20 && rclcpp::ok() && !user_quit; ++i) {
      rclcpp::spin_some(this->get_node_base_interface());
      print_joint_states();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!wait_for_enter_or_quit())
      return;

    // Arm and set all servos to 0 position
    JointCommand zero_cmd;
    zero_cmd.position = {0.0, 0.0, 0.0, 0.0};
    zero_cmd.velocity = {0.1, 0.1, 0.0, 0.0};
    cmd_pub_->publish(zero_cmd);
    std::cout << "All servos set to 0 position." << std::endl;

    if (!wait_for_enter_or_quit())
      return;

    // Hip servos: move back, wait, then forward
    for (int id : {HIP_LEFT, HIP_RIGHT}) {
      if (user_quit)
        break;
      JointCommand cmd = zero_cmd;
      cmd.position[id] = 0.1;
      cmd_pub_->publish(cmd);
      std::cout << "Moved hip " << id << " up." << std::endl;
      if (!wait_for_enter_or_quit())
        break;

      cmd.position[id] = -0.1;
      cmd_pub_->publish(cmd);
      std::cout << "Moved hip " << id << " down." << std::endl;
      if (!wait_for_enter_or_quit())
        break;

      cmd.position[id] = 0.0;
      cmd_pub_->publish(cmd);
    }

    // Wheel servos: spin forward, wait, then backward
    for (int id : {WHEEL_LEFT, WHEEL_RIGHT}) {
      if (user_quit)
        break;
      JointCommand cmd = zero_cmd;
      cmd.velocity[id] = 1.0;
      cmd_pub_->publish(cmd);
      std::cout << "Spinning wheel " << id << " forward." << std::endl;
      if (!wait_for_enter_or_quit())
        break;

      cmd.velocity[id] = -1.0;
      cmd_pub_->publish(cmd);
      std::cout << "Spinning wheel " << id << " backward." << std::endl;
      if (!wait_for_enter_or_quit())
        break;

      cmd.velocity[id] = 0.0;
      cmd_pub_->publish(cmd);
    }

    // Disengage servos
    enable_servos(false);
  }

  void enable_servos(bool value) {
    std_msgs::msg::Bool is_enabled;
    is_enabled.data = value;
    servo_enabled_pub_->publish(is_enabled);
  }

  void print_joint_states() {
    std::cout << "Positions: ";
    for (auto p : last_joint_state_->position)
      std::cout << p << " ";
    std::cout << "| Velocities: ";
    for (auto v : last_joint_state_->velocity)
      std::cout << v << " ";
    std::cout << "| Efforts: ";
    for (auto e : last_joint_state_->effort)
      std::cout << e << " ";
    std::cout << std::endl;
  }

private:
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<DiagnosticStatus>::SharedPtr imu_status_sub_;
  rclcpp::Subscription<DiagnosticStatus>::SharedPtr servo_status_sub_;
  rclcpp::Publisher<JointCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr servo_enabled_pub_;
  JointState::SharedPtr last_joint_state_;
  DiagnosticStatus imu_status_;
  DiagnosticStatus servo_status_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  setup_terminal();

  auto node = std::make_shared<RobotTest>();
  node->run();

  restore_terminal();
  rclcpp::shutdown();
  return 0;
}