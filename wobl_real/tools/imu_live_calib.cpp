#include <chrono>
#include <iostream>
#include <math.h>
#include <wobl_real/imu.h>
#include <thread>

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

void print(const geometry_msgs::msg::Vector3 msg) { std::cout << msg.x << " " << msg.y << " " << msg.z << std::endl; }

int main() {
  IMU imu;
  imu.initialize();

  std::cout << std::endl;
  std::cout << "BIASES" << std::endl;
  print(imu.bias_linear_acceleration());
  print(imu.bias_angular_velocity());
  print(imu.bias_compass());

  // Check success
  if (imu.status()) {
    std::cout << "Device initialize" << std::endl;
  } else {
    std::cout << "Device not initialized" << std::endl;
    return 1;
  }

  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    auto msg = imu.read();
    auto rpy = quat_to_euler(msg.orientation);
    //std::cout << "\rAccuracy: " << imu.accuracy() << " Orientation: " << rpy.x << " " << rpy.y << " " << rpy.z << std::flush;
    std::cout << "\r Orientation: " << msg.linear_acceleration.x << " " <<msg.linear_acceleration.y << " " <<msg.linear_acceleration.z << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(12));

    auto now = std::chrono::high_resolution_clock::now();

    // Calculate elapsed time in seconds
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

    // Break the loop after 5 seconds
    if (elapsed >= 120) {
      break;
    }
  }

  std::cout << std::endl;
  std::cout << "BIASES" << std::endl;
  print(imu.bias_linear_acceleration());
  print(imu.bias_angular_velocity());
  print(imu.bias_compass());

  return 0;
}
