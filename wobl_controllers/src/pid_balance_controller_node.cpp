#include <control_toolbox/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wobl_messages/msg/joint_command.hpp>

class PidBalanceControllerNode : public rclcpp::Node {
public:
  PidBalanceControllerNode() : Node("pid_balance_controller") {
    control_toolbox::AntiWindupStrategy aw_back;
    aw_back.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;
    aw_back.i_min = -0.05;
    aw_back.i_max = 0.05;

    pid_velocity = control_toolbox::Pid();
    pid_pitch = control_toolbox::Pid();
    pid_pitch.initialize(40.0, 0.0, 5.0, 10, -10, aw_back);
    pid_velocity.initialize(0.5, 0.1, 0.0, 0.17, -0.17, aw_back);

    // pid_pitch.initialize(40.0, 0.0, 5, 10.0, -10.0, windup_strat);
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState &msg) {
          received_joint_state_data_ = true;
          joint_state_ = msg;
        });

    imu_subscriber_ =
        this->create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu &msg) {
          received_imu_data_ = true;
          imu_ = msg;
        });

    command_publisher_ = this->create_publisher<wobl_messages::msg::JointCommand>("joint_commands", 1);
    last_time_ = this->now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&PidBalanceControllerNode::update, this));
  }

private:
  std::tuple<double, double, double> quat_to_euler(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return {roll, -pitch, yaw};
  }

  double wheel_velocity() { return ((joint_state_.velocity[2] + joint_state_.velocity[3]) / 2) * 0.08; }

  void update() {
    if (!received_imu_data_ || !received_joint_state_data_ || last_time_.nanoseconds() == 0) {
      return;
    }

    rclcpp::Time current_time = this->now();
    rclcpp::Duration dt = current_time - last_time_;
    last_time_ = current_time;

    velocity_ = velocity_ * 0.9 + wheel_velocity() * 0.1;
    auto [roll, pitch, yaw] = quat_to_euler(imu_.orientation);
    double fwd_velocity = -velocity_;

    double cmd_pitch = pid_velocity.compute_command(-fwd_velocity, dt);
    double cmd_velocity = pid_pitch.compute_command(cmd_pitch + pitch, dt);

    wobl_messages::msg::JointCommand cmd_joint;
    cmd_joint.velocity = {0, 0, cmd_velocity, cmd_velocity};
    command_publisher_->publish(cmd_joint);
  }

  bool received_imu_data_;
  bool received_joint_state_data_;
  double velocity_;
  control_toolbox::Pid pid_pitch, pid_velocity;
  rclcpp::Time last_time_;

  sensor_msgs::msg::JointState joint_state_;
  sensor_msgs::msg::Imu imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<wobl_messages::msg::JointCommand>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidBalanceControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}