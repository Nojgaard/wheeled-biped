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
  bool try_read(sensor_msgs::msg::Imu &data_imu);
  geometry_msgs::msg::Vector3 bias_linear_acceleration();
  geometry_msgs::msg::Vector3 bias_angular_velocity();
  geometry_msgs::msg::Vector3 bias_compass();

  bool status() const;
  double accuracy() const;

private:
  void try_load_bias();

  bool status_;
  double accuracy_;
  icm_20948_DMP_data_t data_dmp_;
  ICM_20948 icm_;
};