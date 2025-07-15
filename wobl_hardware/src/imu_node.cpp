#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <wobl_hardware/imu.h>

void print(const geometry_msgs::msg::Vector3 msg) { std::cout << msg.x << " " << msg.y << " " << msg.z << std::endl; }

class IMUNode : public rclcpp::Node {
public:
  IMUNode() : Node("imu_node") {
    imu_.initialize();
    if (imu_.status()) {
      RCLCPP_INFO(this->get_logger(), "IMU initialized successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU! Shutting down...");
      rclcpp::shutdown();
    }

    auto bias_accel = imu_.bias_linear_acceleration();
    auto bias_gyro = imu_.bias_angular_velocity();
    auto bias_compass = imu_.bias_compass();
    RCLCPP_INFO(this->get_logger(),
                "Loaded biases: Accel: [%.2f %.2f %.2f]  Gyro: [%.2f %.2f %.2f]  Compass: [%.2f %.2f %.2f]",
                bias_accel.x, bias_accel.y, bias_accel.z, bias_gyro.x, bias_gyro.y, bias_gyro.z, bias_compass.x,
                bias_compass.y, bias_compass.z);

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(12), std::bind(&IMUNode::publish_imu_data, this));
  }

private:
  geometry_msgs::msg::Vector3 quat_to_euler(const geometry_msgs::msg::Quaternion &q) {
    geometry_msgs::msg::Vector3 euler;
    // roll (x-axis rotation)
    double t0 = +2.0 * (q.w * q.x + q.y * q.z);
    double t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.x = atan2(t0, t1) * 180.0 / M_PI;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (q.w * q.y - q.x * q.z);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    euler.y = asin(t2) * 180.0 / M_PI;

    // yaw (z-axis rotation)
    double t3 = +2.0 * (q.w * q.z + q.x * q.y);
    double t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler.z = atan2(t3, t4) * 180.0 / M_PI;
    return euler;
  }

  void debug_print_imu_data(const sensor_msgs::msg::Imu &msg) {
    auto rpy = quat_to_euler(msg.orientation);
    RCLCPP_DEBUG(this->get_logger(),
                 "Orientation: [%.2f %.2f %.2f]  "
                 "Gyro: [%.2f %.2f %.2f]  "
                 "Accel: [%.2f %.2f %.2f]",
                 rpy.x, rpy.y, rpy.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                 msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  }

  void publish_imu_data() {
    sensor_msgs::msg::Imu msg = imu_.read();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    publisher_->publish(msg);
    debug_print_imu_data(msg);
  }

  IMU imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}