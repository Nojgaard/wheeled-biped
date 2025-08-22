#pragma once

class LinearFilter {
public:
  LinearFilter(double alpha) : alpha_(alpha), filtered_value_(0.0) {}

  double update(double new_value) {
    filtered_value_ = alpha_ * new_value + (1.0 - alpha_) * filtered_value_;
    return filtered_value_;
  }

  double value() const { return filtered_value_; }

private:
  double alpha_;
  double filtered_value_;
};
