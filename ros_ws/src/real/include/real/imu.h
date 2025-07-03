#pragma once

#include <ICM_20948.h>
#include <array>

struct Vector3 {
  double x, y, z;
};

struct Quaternion {
  double x, y, z, w;
};

class IMU {
public:
  IMU();
  void initialize();
  void read();

  bool status() const;
  const Quaternion& orientation() const;
  const Vector3& gyro() const;
  const Vector3& acceleration() const;
  double accuracy() const;

private:
  bool status_;
  Quaternion orientation_;
  Vector3 gyro_;
  Vector3 acceleration_;
  double accuracy_;

  icm_20948_DMP_data_t data_;
  ICM_20948 icm_;
};