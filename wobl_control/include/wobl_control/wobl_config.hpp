#pragma once
#include <vector>

struct WoblConfig {
    double offset_pitch;
    double max_pitch;
    double max_wheel_rps;
    std::vector<double> pitch2vel_gains;
    std::vector<double> vel2pitch_gains;
    std::vector<double> lqr_K;
    double wheel_radius;
    double wheel_separation;
};
