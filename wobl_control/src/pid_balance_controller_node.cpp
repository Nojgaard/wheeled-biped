#include <control_toolbox/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <wobl_msgs/msg/joint_command.hpp>
#include <wobl_msgs/msg/topics.hpp>

using JointState = sensor_msgs::msg::JointState;
using Imu = sensor_msgs::msg::Imu;
using JointCommand = wobl_msgs::msg::JointCommand;
using Topics = wobl_msgs::msg::Topics;

class PidBalanceController {
public:
  PidBalanceController() {
    aws.type = control_toolbox::AntiWindupStrategy::CONDITIONAL_INTEGRATION;
    aws.i_min = -0.05;
    aws.i_max = 0.05;

    pid_pitch.initialize(200.0, 0.0, 5.0, 10, -10, aws);
    pid_velocity.initialize(0.1, 0.1, 0.0, 0.17, -0.17, aws);
  }

  double extract_pitch(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return pitch;
  }

  double wheel_velocity(const JointState &joint_state) {
    return ((joint_state.velocity[2] + joint_state.velocity[3]) / 2) * 0.08;
  }

  JointCommand update(const JointState &joint_state, const Imu &imu, const rclcpp::Duration &dt) {
    velocity_ = velocity_ * 0.8 + wheel_velocity(joint_state) * 0.2;
    auto pitch = extract_pitch(imu.orientation);

    double cmd_pitch = pid_velocity.compute_command(-velocity_, dt);
    double cmd_velocity = pid_pitch.compute_command(pitch - cmd_pitch - 0.035, dt);

    wobl_msgs::msg::JointCommand cmd_joint;
    cmd_joint.position = {0, 0, 0, 0};
    cmd_joint.velocity = {1.0, 1.0, cmd_velocity, cmd_velocity};
    return cmd_joint;
  }

  void set_vel_gains(double kp, double ki, double kd) { pid_velocity.set_gains(kp, ki, kd, 0.17, -0.17, aws); }

  void set_pitch_gains(double kp, double ki, double kd) { pid_pitch.set_gains(kp, ki, kd, 10, -10, aws); }

private:
  double velocity_;
  control_toolbox::AntiWindupStrategy aws;
  control_toolbox::Pid pid_pitch;
  control_toolbox::Pid pid_velocity;
};

class PidBalanceControllerNode : public rclcpp::Node {
public:
  PidBalanceControllerNode() : Node("pid_balance_controller") {
    declare_parameter("pitch_kp", 200.0);
    declare_parameter("pitch_ki", 0.0);
    declare_parameter("pitch_kd", 5.0);

    declare_parameter("vel_kp", 0.1);
    declare_parameter("vel_ki", 0.1);
    declare_parameter("vel_kd", 0.0);
    joint_state_subscriber_ = this->create_subscription<JointState>(
        Topics::JOINT_STATE, rclcpp::SensorDataQoS(), [this](const JointState &msg) { joint_state_ = msg; });

    imu_subscriber_ =
        this->create_subscription<Imu>(Topics::IMU, rclcpp::SensorDataQoS(), [this](const Imu &msg) { imu_ = msg; });

    command_publisher_ = this->create_publisher<JointCommand>(Topics::JOINT_COMMAND, 1);
    last_time_ = this->now();
    last_update_param_time_ = this->now();
    timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&PidBalanceControllerNode::update, this));
  }

private:
  bool is_joint_state_ready() { return !joint_state_.position.empty(); }
  bool is_imu_ready() { return imu_.header.stamp.sec != 0 || imu_.header.stamp.nanosec != 0; }

  void update() {
    if (!is_imu_ready() || !is_joint_state_ready()) {
      return;
    }

    rclcpp::Time current_time = this->now();
    rclcpp::Duration dt = current_time - last_time_;
    last_time_ = current_time;
    if (dt.seconds() < 0)
      return;

    if (current_time - last_update_param_time_ > rclcpp::Duration::from_seconds(3.0)) {
      controller.set_pitch_gains(get_parameter("pitch_kp").as_double(), get_parameter("pitch_ki").as_double(),
                                 get_parameter("pitch_kd").as_double());

      controller.set_vel_gains(get_parameter("vel_kp").as_double(), get_parameter("vel_ki").as_double(),
                               get_parameter("vel_kd").as_double());
      last_update_param_time_ = current_time;
    }

    JointCommand cmd_joint = controller.update(joint_state_, imu_, dt);
    cmd_joint.header.stamp = current_time;
    command_publisher_->publish(cmd_joint);
  }

  PidBalanceController controller;
  rclcpp::Time last_time_, last_update_param_time_;

  JointState joint_state_;
  Imu imu_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_;
  rclcpp::Publisher<JointCommand>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidBalanceControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}