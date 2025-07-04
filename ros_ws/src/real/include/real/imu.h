#pragma once

#include <ICM_20948.h>
#include <array>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMU {
public:
  IMU();
  void initialize();
  const sensor_msgs::msg::Imu &read();

  bool status() const;
  double accuracy() const;

private:
  bool status_;
  double accuracy_;
  sensor_msgs::msg::Imu data_imu_;
  icm_20948_DMP_data_t data_dmp_;
  ICM_20948 icm_;
};