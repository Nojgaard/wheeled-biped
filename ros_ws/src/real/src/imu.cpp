#include <math.h>
#include <real/imu.h>

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
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  // status_ &= (icm_.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Geomag, 3) == ICM_20948_Stat_Ok);
  status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Accel, 3) == ICM_20948_Stat_Ok);
  status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 3) == ICM_20948_Stat_Ok);
  // status_ &= (icm_.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 1) == ICM_20948_Stat_Ok); // Set to the maximum

  status_ &= (icm_.enableFIFO() == ICM_20948_Stat_Ok);
  status_ &= (icm_.enableDMP() == ICM_20948_Stat_Ok);
  status_ &= (icm_.resetDMP() == ICM_20948_Stat_Ok);
  status_ &= (icm_.resetFIFO() == ICM_20948_Stat_Ok);
}

double lsbToDpsScalar(const ICM_20948 &icm) {
  switch (icm.agmt.fss.g) {
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

double lsbToGScalar(const ICM_20948 &icm) {
  switch (icm.agmt.fss.a) {
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

const sensor_msgs::msg::Imu &IMU::read() {
  icm_.readDMPdataFromFIFO(&data_dmp_);

  while ((icm_.status == ICM_20948_Stat_Ok) || (icm_.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data_dmp_.header & DMP_header_bitmap_Geomag) >
        0) // We have asked for orientation data so we should receive Quat9
    {
      // Scale to +/- 1
      double q1 = ((double)data_dmp_.Geomag.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data_dmp_.Geomag.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data_dmp_.Geomag.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = std::sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      accuracy_ = (double)data_dmp_.Geomag.Data.Accuracy;
      data_imu_.orientation.x = q1;
      data_imu_.orientation.y = q2;
      data_imu_.orientation.z = -q3;
      data_imu_.orientation.w = q0;
    } else if ((data_dmp_.header & DMP_header_bitmap_Accel) > 0) {
      double lsb2g = lsbToGScalar(icm_);
      double gravity = 9.80665;
      data_imu_.linear_acceleration.x = data_dmp_.Raw_Accel.Data.X * lsb2g * gravity;
      data_imu_.linear_acceleration.y = data_dmp_.Raw_Accel.Data.Y * lsb2g * gravity;
      data_imu_.linear_acceleration.z = data_dmp_.Raw_Accel.Data.Z * lsb2g * gravity;
    } else if ((data_dmp_.header & DMP_header_bitmap_Gyro_Calibr) > 0) {
      double lsb2dps = lsbToDpsScalar(icm_);
      double dps2rps = 0.0174532925;
      data_imu_.angular_velocity.x = (data_dmp_.Gyro_Calibr.Data.X / 32768) * lsb2dps * dps2rps;
      data_imu_.angular_velocity.y = (data_dmp_.Gyro_Calibr.Data.Y / 32768) * lsb2dps * dps2rps;
      data_imu_.angular_velocity.z = (data_dmp_.Gyro_Calibr.Data.Z / 32768) * lsb2dps * dps2rps;
    }
  }
  return data_imu_;
}

bool IMU::status() const { return status_; }
double IMU::accuracy() const { return accuracy_; }