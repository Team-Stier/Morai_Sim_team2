// Extracted from Closedteam2 longitudinal_control_node.cpp, ff8eb42458518379.
#pragma once
#include <algorithm>
#include <cmath>
#include <utility>
namespace longitudinal_control {
inline double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
class BoundedPiController {
 public:
  BoundedPiController(double kp, double ki, double brake_kp,
                      double integrator_limit, double stop_brake)
      : kp_(kp),
        ki_(ki),
        brake_kp_(brake_kp),
        integrator_limit_(integrator_limit),
        stop_brake_(stop_brake) {}

  std::pair<double, double> calculate(double target_kph, double speed_kph,
                                      double dt_sec, bool stop_required) {
    if (!std::isfinite(target_kph) || !std::isfinite(speed_kph) ||
        !std::isfinite(dt_sec) || dt_sec <= 0.0 ||
        !std::isfinite(kp_) || kp_ < 0.0 ||
        !std::isfinite(ki_) || ki_ < 0.0 ||
        !std::isfinite(brake_kp_) || brake_kp_ < 0.0 ||
        !std::isfinite(integrator_limit_) || integrator_limit_ < 0.0 ||
        !std::isfinite(stop_brake_) || stop_brake_ < 0.0) {
      return {0.0, 1.0};
    }

    const double error_kph = target_kph - speed_kph;
    if (stop_required) {
      reset();
      const double brake =
          std::max(Clamp(stop_brake_, 0.0, 1.0),
                   Clamp(brake_kp_ * std::max(0.0, speed_kph), 0.0, 1.0));
      return {0.0, brake};
    }
    if (error_kph > 0.1) {
      integral_ =
          Clamp(integral_ + error_kph * dt_sec, -integrator_limit_,
                integrator_limit_);
      return {Clamp(kp_ * error_kph + ki_ * integral_, 0.0, 1.0), 0.0};
    }
    if (error_kph < -0.1) {
      reset();
      return {0.0, Clamp(brake_kp_ * -error_kph, 0.0, 1.0)};
    }
    return {0.0, 0.0};
  }

  void reset() { integral_ = 0.0; }

 private:
  double kp_;
  double ki_;
  double brake_kp_;
  double integrator_limit_;
  double stop_brake_;
  double integral_{0.0};
};

}  // namespace longitudinal_control
