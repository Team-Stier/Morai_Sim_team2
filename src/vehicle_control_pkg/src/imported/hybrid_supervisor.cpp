#include "hybrid_path_tracking/hybrid_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace hybrid_path_tracking {
namespace {

constexpr double kMaxSteeringRad =
    40.0 * 3.14159265358979323846 / 180.0;
constexpr double kFailSafeMaxStepRad = 0.03;

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

bool validConfig(const SupervisorConfig& config) {
  return std::isfinite(config.curvature_enter_stanley) &&
         std::isfinite(config.curvature_exit_stanley) &&
         std::isfinite(config.cross_track_enter_stanley_m) &&
         std::isfinite(config.cross_track_exit_stanley_m) &&
         std::isfinite(config.minimum_dwell_sec) &&
         std::isfinite(config.blend_duration_sec) &&
         std::isfinite(config.max_step_rad) &&
         config.curvature_enter_stanley >= 0.06 &&
         config.curvature_exit_stanley >= 0.0 &&
         config.curvature_exit_stanley <= 0.04 &&
         config.curvature_enter_stanley >
             config.curvature_exit_stanley &&
         config.cross_track_enter_stanley_m >= 0.8 &&
         config.cross_track_exit_stanley_m >= 0.0 &&
         config.cross_track_exit_stanley_m <= 0.4 &&
         config.cross_track_enter_stanley_m >
             config.cross_track_exit_stanley_m &&
         config.confirmation_samples >= 3 &&
         config.minimum_dwell_sec >= 1.0 &&
         config.blend_duration_sec >= 0.5 && config.max_step_rad > 0.0 &&
         config.max_step_rad <= kFailSafeMaxStepRad;
}

bool isPpRoute(HybridMode mode) {
  return mode == HybridMode::PP_GLOBAL || mode == HybridMode::PP_VISION ||
         mode == HybridMode::PP_AVOIDANCE;
}

bool isStanleyRoute(HybridMode mode) {
  return mode == HybridMode::STANLEY_GLOBAL;
}

bool routeMatchesPreference(HybridMode route, HybridMode preference) {
  return (preference == HybridMode::PP_GLOBAL && isPpRoute(route)) ||
         (preference == HybridMode::STANLEY_GLOBAL &&
          isStanleyRoute(route));
}

}  // namespace

HybridSupervisor::HybridSupervisor(SupervisorConfig config)
    : config_(config), config_valid_(validConfig(config)) {}

double HybridSupervisor::select(const ControllerCandidate& pure_pursuit,
                                const ControllerCandidate& stanley,
                                const SupervisorContext& context) {
  if (!config_valid_ || !context.path_valid ||
      !std::isfinite(context.now_sec) ||
      (has_last_time_ && context.now_sec < last_time_sec_)) {
    return controlledStop();
  }

  has_last_time_ = true;
  last_time_sec_ = context.now_sec;

  const bool had_path_source = has_path_source_;
  const PathSource previous_path_source = last_path_source_;
  const bool source_changed =
      !has_path_source_ || context.path_source != last_path_source_;
  has_path_source_ = true;
  last_path_source_ = context.path_source;

  if (context.path_source == PathSource::VISION) {
    if (had_path_source && previous_path_source == PathSource::GLOBAL &&
        source_changed) {
      recordGlobalControllerTransfer(context.now_sec);
    }
    entry_confirmation_count_ = 0;
    exit_confirmation_count_ = 0;
    preferred_global_mode_ = HybridMode::PP_GLOBAL;
    if (!candidateValid(pure_pursuit)) {
      return controlledStop();
    }
    return command(pure_pursuit.steering_angle_rad, HybridMode::PP_VISION,
                   HybridMode::PP_VISION, context.now_sec);
  }

  if (context.path_source == PathSource::AVOIDANCE) {
    if (had_path_source && previous_path_source == PathSource::GLOBAL &&
        source_changed) {
      recordGlobalControllerTransfer(context.now_sec);
    }
    entry_confirmation_count_ = 0;
    exit_confirmation_count_ = 0;
    preferred_global_mode_ = HybridMode::PP_GLOBAL;
    if (!candidateValid(pure_pursuit)) {
      return controlledStop();
    }
    return command(pure_pursuit.steering_angle_rad,
                   HybridMode::PP_AVOIDANCE, HybridMode::PP_AVOIDANCE,
                   context.now_sec);
  }

  if (context.path_source != PathSource::GLOBAL) {
    return controlledStop();
  }

  if (source_changed) {
    preferred_global_mode_ = HybridMode::PP_GLOBAL;
    entry_confirmation_count_ = 0;
    exit_confirmation_count_ = 0;
  }

  const bool pp_valid = candidateValid(pure_pursuit);
  const bool stanley_valid = candidateValid(stanley);
  const bool has_actual_global_controller =
      isPpRoute(command_route_) || isStanleyRoute(command_route_);

  if (has_actual_global_controller &&
      !routeMatchesPreference(command_route_, preferred_global_mode_)) {
    entry_confirmation_count_ = 0;
    exit_confirmation_count_ = 0;
    if (dwellSatisfied(context.now_sec)) {
      if (preferred_global_mode_ == HybridMode::STANLEY_GLOBAL &&
          stanley_valid) {
        return command(stanley.steering_angle_rad,
                       HybridMode::STANLEY_GLOBAL,
                       HybridMode::STANLEY_GLOBAL, context.now_sec);
      }
      if (preferred_global_mode_ == HybridMode::PP_GLOBAL && pp_valid) {
        return command(pure_pursuit.steering_angle_rad,
                       HybridMode::PP_GLOBAL, HybridMode::PP_GLOBAL,
                       context.now_sec);
      }
    }

    if (isPpRoute(command_route_) && pp_valid) {
      return command(pure_pursuit.steering_angle_rad, HybridMode::DEGRADED,
                     HybridMode::PP_GLOBAL, context.now_sec);
    }
    if (isStanleyRoute(command_route_) && stanley_valid) {
      return command(stanley.steering_angle_rad, HybridMode::DEGRADED,
                     HybridMode::STANLEY_GLOBAL, context.now_sec);
    }
    if (preferred_global_mode_ == HybridMode::STANLEY_GLOBAL &&
        stanley_valid) {
      return command(stanley.steering_angle_rad,
                     HybridMode::STANLEY_GLOBAL,
                     HybridMode::STANLEY_GLOBAL, context.now_sec);
    }
    if (preferred_global_mode_ == HybridMode::PP_GLOBAL && pp_valid) {
      return command(pure_pursuit.steering_angle_rad,
                     HybridMode::PP_GLOBAL, HybridMode::PP_GLOBAL,
                     context.now_sec);
    }
    return controlledStop();
  }

  updateGlobalPreference(pure_pursuit, stanley, context.now_sec);
  if (preferred_global_mode_ == HybridMode::STANLEY_GLOBAL) {
    if (stanley_valid) {
      return command(stanley.steering_angle_rad, HybridMode::STANLEY_GLOBAL,
                     HybridMode::STANLEY_GLOBAL, context.now_sec);
    }
    if (pp_valid) {
      recordGlobalControllerTransfer(context.now_sec);
      return command(pure_pursuit.steering_angle_rad, HybridMode::DEGRADED,
                     HybridMode::PP_GLOBAL, context.now_sec);
    }
    return controlledStop();
  }

  if (pp_valid) {
    return command(pure_pursuit.steering_angle_rad, HybridMode::PP_GLOBAL,
                   HybridMode::PP_GLOBAL, context.now_sec);
  }
  if (stanley_valid) {
    recordGlobalControllerTransfer(context.now_sec);
    return command(stanley.steering_angle_rad, HybridMode::DEGRADED,
                   HybridMode::STANLEY_GLOBAL, context.now_sec);
  }
  return controlledStop();
}

HybridMode HybridSupervisor::mode() const { return mode_; }

void HybridSupervisor::seedSteering(double steering_angle_rad) {
  last_steering_rad_ =
      std::isfinite(steering_angle_rad)
          ? clamp(steering_angle_rad, -kMaxSteeringRad, kMaxSteeringRad)
          : 0.0;
  transition_active_ = false;
  command_route_ = HybridMode::STOP;
}

bool HybridSupervisor::candidateValid(
    const ControllerCandidate& candidate) const {
  return candidate.valid && std::isfinite(candidate.steering_angle_rad) &&
         std::isfinite(candidate.cross_track_error_m) &&
         std::isfinite(candidate.heading_error_rad) &&
         std::isfinite(candidate.path_curvature) &&
         std::abs(candidate.steering_angle_rad) <= kMaxSteeringRad;
}

bool HybridSupervisor::dwellSatisfied(double now_sec) const {
  return !has_global_controller_transfer_time_ ||
         now_sec - global_controller_transfer_time_sec_ >=
             config_.minimum_dwell_sec;
}

void HybridSupervisor::recordGlobalControllerTransfer(double now_sec) {
  has_global_controller_transfer_time_ = true;
  global_controller_transfer_time_sec_ = now_sec;
}

void HybridSupervisor::updateGlobalPreference(
    const ControllerCandidate& pure_pursuit,
    const ControllerCandidate& stanley, double now_sec) {
  const bool pp_valid = candidateValid(pure_pursuit);
  const bool stanley_valid = candidateValid(stanley);

  if (preferred_global_mode_ == HybridMode::PP_GLOBAL) {
    exit_confirmation_count_ = 0;
    const bool enter =
        pp_valid && stanley_valid &&
        (std::abs(pure_pursuit.path_curvature) >
             config_.curvature_enter_stanley ||
         std::abs(pure_pursuit.cross_track_error_m) >
             config_.cross_track_enter_stanley_m);
    if (enter) {
      if (entry_confirmation_count_ < config_.confirmation_samples) {
        ++entry_confirmation_count_;
      }
    } else {
      entry_confirmation_count_ = 0;
    }
    if (entry_confirmation_count_ >= config_.confirmation_samples &&
        dwellSatisfied(now_sec)) {
      preferred_global_mode_ = HybridMode::STANLEY_GLOBAL;
      entry_confirmation_count_ = 0;
    }
    return;
  }

  entry_confirmation_count_ = 0;
  const bool exit =
      pp_valid && stanley_valid &&
      std::abs(stanley.path_curvature) < config_.curvature_exit_stanley &&
      std::abs(stanley.cross_track_error_m) <
          config_.cross_track_exit_stanley_m;
  if (exit) {
    if (exit_confirmation_count_ < config_.confirmation_samples) {
      ++exit_confirmation_count_;
    }
  } else {
    exit_confirmation_count_ = 0;
  }
  if (exit_confirmation_count_ >= config_.confirmation_samples &&
      dwellSatisfied(now_sec)) {
    preferred_global_mode_ = HybridMode::PP_GLOBAL;
    exit_confirmation_count_ = 0;
  }
}

double HybridSupervisor::command(double target_steering_rad,
                                 HybridMode reported_mode,
                                 HybridMode command_route,
                                 double now_sec) {
  if (command_route != command_route_) {
    if ((isPpRoute(command_route_) && isStanleyRoute(command_route)) ||
        (isStanleyRoute(command_route_) && isPpRoute(command_route))) {
      recordGlobalControllerTransfer(now_sec);
    }
    command_route_ = command_route;
    transition_active_ = true;
    transition_start_time_sec_ = now_sec;
    transition_start_steering_rad_ = last_steering_rad_;
    transition_target_steering_rad_ =
        clamp(target_steering_rad, -kMaxSteeringRad, kMaxSteeringRad);
  }

  double target = clamp(target_steering_rad, -kMaxSteeringRad,
                        kMaxSteeringRad);
  if (transition_active_) {
    const double alpha =
        config_.blend_duration_sec == 0.0
            ? 1.0
            : clamp((now_sec - transition_start_time_sec_) /
                        config_.blend_duration_sec,
                    0.0, 1.0);
    target = transition_start_steering_rad_ +
             alpha * (transition_target_steering_rad_ -
                      transition_start_steering_rad_);
    transition_active_ = alpha < 1.0;
  }

  mode_ = reported_mode;
  return stepToward(target);
}

double HybridSupervisor::controlledStop() {
  mode_ = HybridMode::STOP;
  command_route_ = HybridMode::STOP;
  transition_active_ = false;
  entry_confirmation_count_ = 0;
  exit_confirmation_count_ = 0;
  return stepToward(0.0);
}

double HybridSupervisor::stepToward(double target_steering_rad) {
  if (!std::isfinite(last_steering_rad_)) {
    last_steering_rad_ = 0.0;
  }
  const double target =
      std::isfinite(target_steering_rad)
          ? clamp(target_steering_rad, -kMaxSteeringRad, kMaxSteeringRad)
          : 0.0;
  const double configured_step =
      std::isfinite(config_.max_step_rad) && config_.max_step_rad > 0.0
          ? config_.max_step_rad
          : kFailSafeMaxStepRad;
  const double step = std::min(configured_step, kFailSafeMaxStepRad);
  const double delta =
      clamp(target - last_steering_rad_, -step, step);
  last_steering_rad_ =
      clamp(last_steering_rad_ + delta, -kMaxSteeringRad, kMaxSteeringRad);
  return last_steering_rad_;
}

}  // namespace hybrid_path_tracking
