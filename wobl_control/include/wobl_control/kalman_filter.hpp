#pragma once

class KalmanFilter {
public:
  KalmanFilter(double process_noise, double measurement_noise)
      : q_(process_noise), r_(measurement_noise), x_(0.0), p_(1.0) {}

  void predict(double dt = 1.0) {
    // State prediction (assuming constant velocity model)
    // x_k = x_{k-1} (no state transition for position-only model)
    
    // Error covariance prediction
    p_ = p_ + q_ * dt;
  }

  double update(double measurement) {
    // Kalman gain
    double k = p_ / (p_ + r_);
    
    // State update
    x_ = x_ + k * (measurement - x_);
    
    // Error covariance update
    p_ = (1.0 - k) * p_;
    
    return x_;
  }

  double predict_and_update(double measurement, double dt = 1.0) {
    predict(dt);
    return update(measurement);
  }

  double state() const { return x_; }
  double covariance() const { return p_; }
  
  void reset(double initial_state = 0.0, double initial_covariance = 1.0) {
    x_ = initial_state;
    p_ = initial_covariance;
  }

private:
  double q_;  // Process noise variance
  double r_;  // Measurement noise variance
  double x_;  // State estimate
  double p_;  // Error covariance
};
