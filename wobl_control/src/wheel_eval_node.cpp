#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wobl_msgs/msg/joint_command.hpp>
#include <wobl_msgs/msg/topics.hpp>
#include <thread>

#include <chrono>
#include <string>
#include <vector>

using JointState = sensor_msgs::msg::JointState;
using JointCommand = wobl_msgs::msg::JointCommand;
using Topics = wobl_msgs::msg::Topics;

class WheelEvalNode : public rclcpp::Node {
public:
  WheelEvalNode() : Node("wheel_eval_node") {
    declare_parameter<std::vector<double>>("commands",
                                           {0.0, 0.5, 1.0, 5, 10, -5, -10, -0.5, -1.0, 0.0, 1.5, 0.5, -0.5, -1.5, 0.0});
    declare_parameter<std::vector<double>>("durations", {5, 2, 2, 2.0, 2.5, 2.0, 2, 2.0, 2.5, 2.0, 0.5, 0.5, 0.5, 0.5, 2});

    command_values_ = get_parameter("commands").as_double_array();
    durations_ = get_parameter("durations").as_double_array();

    joint_command_pub_ = create_publisher<JointCommand>(Topics::JOINT_COMMAND, 1);
  }

  void run_command_sequence() { 
    JointCommand cmd;
    cmd.position = {0, 0, 0, 0};
    
    for (size_t i = 0; i < command_values_.size(); ++i) {
      if (!rclcpp::ok())
        break;
      cmd.velocity = {0.0, 0.0, command_values_[i], command_values_[i]};
      joint_command_pub_->publish(cmd);
      RCLCPP_INFO(get_logger(), "Published command: %f", command_values_[i]);
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(durations_[i] * 1000)));
    }
  }

  std::vector<double> command_values_;
  std::vector<double> durations_;
  rclcpp::Publisher<JointCommand>::SharedPtr joint_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WheelEvalNode>();
  node->run_command_sequence();
  rclcpp::shutdown();
  return 0;
}