#include <fstream>
#include <math.h>
#include <wobl_real/imu.h>

/*const int32_t BIAS_LINEAR_ACCELERATION[3] = {-99 * 4096, -185 * 4096, 0 * 4096};
const int32_t BIAS_ANGULAR_VELOCITY[3] = {0, 0, 0};
const int32_t BIAS_COMPASS[3] = {16 * 65536, 15 * 65536, 2 * 65536};*/

const int32_t BIAS_LINEAR_ACCELERATION[3] = {-330752, -826368, 585728};
const int32_t BIAS_ANGULAR_VELOCITY[3] = {-17440, 1344, 12768};
const int32_t BIAS_COMPASS[3] = {-230400,  static_cast<int32_t>(2.47907e+06),  static_cast<int32_t>(-3.26298e+06)};

IMU::IMU() : icm_("/dev/i2c-1", 0x69) {}

void IMU::initialize() {
  status_ = true;
  status_ &= (icm_.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy) (no live calib)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  // status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok);
  status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Accel, 1) == ICM_20948_Stat_Ok);
  status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 1) == ICM_20948_Stat_Ok);
  // status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 1) == ICM_20948_Stat_Ok); // Set to the maximum

  status_ &= (icm_.enableFIFO() == ICM_20948_Stat_Ok);
  status_ &= (icm_.enableDMP() == ICM_20948_Stat_Ok);
  status_ &= (icm_.resetDMP() == ICM_20948_Stat_Ok);
  status_ &= (icm_.resetFIFO() == ICM_20948_Stat_Ok);

  try_load_bias();
}

double get_lsb_to_dps(uint8_t gpm) {
  switch (gpm) {
  case 0:
    return 1 / 131.0;
  case 1:
    return 1 / 65.5;
  case 2:
    return 1 / 32.8;
  case 3:
    return 1 / 16.4;
  default:
    return 0;
  }
}

double get_lsb_to_g(uint8_t dps) {
  switch (dps) {
  case 0:
    return 1 / 16384.0;
  case 1:
    return 1 / 8192.0;
  case 2:
    return 1 / 4096.0;
  case 3:
    return 1 / 2048.0;
  default:
    return 0;
  }
}

bool IMU::try_read(sensor_msgs::msg::Imu &data_imu) {
  icm_.readDMPdataFromFIFO(&data_dmp_);
  bool has_data = false;

  while ((icm_.status == ICM_20948_Stat_Ok) || (icm_.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    has_data = true;
    if ((data_dmp_.header & DMP_header_bitmap_Quat9) >
        0) // We have asked for orientation data so we should receive Quat9
    {
      // Scale to +/- 1
      double q1 = ((double)data_dmp_.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data_dmp_.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data_dmp_.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = std::sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      accuracy_ = (double)data_dmp_.Quat9.Data.Accuracy;
      data_imu.orientation.x = q1;
      data_imu.orientation.y = q2;
      data_imu.orientation.z = q3;
      data_imu.orientation.w = q0;
    }
    if ((data_dmp_.header & DMP_header_bitmap_Accel) > 0) {
      double lsb2g = get_lsb_to_g(gpm4);
      double gravity = 9.80665;
      data_imu.linear_acceleration.x = data_dmp_.Raw_Accel.Data.X * lsb2g * gravity;
      data_imu.linear_acceleration.y = data_dmp_.Raw_Accel.Data.Y * lsb2g * gravity;
      data_imu.linear_acceleration.z = data_dmp_.Raw_Accel.Data.Z * lsb2g * gravity;
    }
    if ((data_dmp_.header & DMP_header_bitmap_Gyro_Calibr) > 0) {
      double lsb2dps = get_lsb_to_dps(dps2000);
      double dps2rps = 0.0174532925;
      const auto &gd = data_dmp_.Raw_Gyro.Data;
      data_imu.angular_velocity.x = gd.X * lsb2dps * dps2rps;
      data_imu.angular_velocity.y = gd.Y * lsb2dps * dps2rps;
      data_imu.angular_velocity.z = gd.Z * lsb2dps * dps2rps;
    }
    icm_.readDMPdataFromFIFO(&data_dmp_);
  }
  return has_data;
}

bool IMU::status() const { return status_; }
double IMU::accuracy() const { return accuracy_; }

geometry_msgs::msg::Vector3 IMU::bias_linear_acceleration() {
  int32_t x, y ,z;
  geometry_msgs::msg::Vector3 bias;
  status_ &= (icm_.getBias(ICM_20948::Bias::AccelX, &x) == ICM_20948_Stat_Ok);
  status_ &= (icm_.getBias(ICM_20948::Bias::AccelY, &y) == ICM_20948_Stat_Ok);
  status_ &= (icm_.getBias(ICM_20948::Bias::AccelZ, &z) == ICM_20948_Stat_Ok);
  bias.x = x;
  bias.y = y;
  bias.z = z;
  return bias;
}

geometry_msgs::msg::Vector3 IMU::bias_angular_velocity() {
  int32_t x, y ,z;
  geometry_msgs::msg::Vector3 bias;
  icm_.getBias(ICM_20948::Bias::GyroX, &x);
  icm_.getBias(ICM_20948::Bias::GyroY, &y);
  icm_.getBias(ICM_20948::Bias::GyroZ, &z);
  bias.x = x;
  bias.y = y;
  bias.z = z;
  return bias;
}

geometry_msgs::msg::Vector3 IMU::bias_compass() {
  int32_t x, y ,z;
  geometry_msgs::msg::Vector3 bias;
  icm_.getBias(ICM_20948::Bias::MagX, &x);
  icm_.getBias(ICM_20948::Bias::MagY, &y);
  icm_.getBias(ICM_20948::Bias::MagZ, &z);
  bias.x = x;
  bias.y = y;
  bias.z = z;
  return bias;
}

void IMU::try_load_bias() {
  icm_.setBias(ICM_20948::Bias::AccelX, BIAS_LINEAR_ACCELERATION[0]);
  icm_.setBias(ICM_20948::Bias::AccelY, BIAS_LINEAR_ACCELERATION[1]);
  icm_.setBias(ICM_20948::Bias::AccelZ, BIAS_LINEAR_ACCELERATION[2]);

  icm_.setBias(ICM_20948::Bias::GyroX, BIAS_ANGULAR_VELOCITY[0]);
  icm_.setBias(ICM_20948::Bias::GyroY, BIAS_ANGULAR_VELOCITY[1]);
  icm_.setBias(ICM_20948::Bias::GyroZ, BIAS_ANGULAR_VELOCITY[2]);

  icm_.setBias(ICM_20948::Bias::MagX, BIAS_COMPASS[0]);
  icm_.setBias(ICM_20948::Bias::MagY, BIAS_COMPASS[1]);
  icm_.setBias(ICM_20948::Bias::MagZ, BIAS_COMPASS[2]);
}
