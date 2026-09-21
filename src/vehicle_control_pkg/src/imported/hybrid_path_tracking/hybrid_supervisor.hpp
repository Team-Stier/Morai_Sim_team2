#ifndef HYBRID_PATH_TRACKING_HYBRID_SUPERVISOR_HPP_
#define HYBRID_PATH_TRACKING_HYBRID_SUPERVISOR_HPP_

#include "hybrid_path_tracking/control_types.hpp"
#include "hybrid_path_tracking/path_source_manager.hpp"

namespace hybrid_path_tracking {

enum class HybridMode {
  PP_GLOBAL,
  PP_VISION,
  PP_AVOIDANCE,
  STANLEY_GLOBAL,
  DEGRADED,
  STOP
};

struct SupervisorConfig {
  double curvature_enter_stanley{0.06};
  double curvature_exit_stanley{0.04};
  double cross_track_enter_stanley_m{0.8};
  double cross_track_exit_stanley_m{0.4};
  int confirmation_samples{3};
  double minimum_dwell_sec{1.0};
  double blend_duration_sec{0.5};
  double max_step_rad{0.03};
};

struct SupervisorContext {
  PathSource path_source;
  double now_sec;
  bool path_valid{false};
};

class HybridSupervisor {
 public:
  explicit HybridSupervisor(SupervisorConfig config);

  double select(const ControllerCandidate& pure_pursuit,
                const ControllerCandidate& stanley,
                const SupervisorContext& context);
  HybridMode mode() const;
  void seedSteering(double steering_angle_rad);

 private:
  bool candidateValid(const ControllerCandidate& candidate) const;
  bool dwellSatisfied(double now_sec) const;
  void recordGlobalControllerTransfer(double now_sec);
  void updateGlobalPreference(const ControllerCandidate& pure_pursuit,
                              const ControllerCandidate& stanley,
                              double now_sec);
  double command(double target_steering_rad, HybridMode reported_mode,
                 HybridMode command_route, double now_sec);
  double controlledStop();
  double stepToward(double target_steering_rad);

  SupervisorConfig config_;
  bool config_valid_{false};

  // A new supervisor has no trusted timing history or active controller.
  // It starts in STOP. Equal timestamps are accepted as additional samples;
  // strictly decreasing or non-finite timestamps fail safe to STOP.
  HybridMode mode_{HybridMode::STOP};
  HybridMode preferred_global_mode_{HybridMode::PP_GLOBAL};
  HybridMode command_route_{HybridMode::STOP};
  bool has_path_source_{false};
  PathSource last_path_source_{PathSource::GLOBAL};
  bool has_last_time_{false};
  double last_time_sec_{0.0};

  // Initial path acquisition is dwell-exempt. Dwell is measured from an actual
  // PP/Stanley command transfer or from leaving GLOBAL for a PP-only source.
  bool has_global_controller_transfer_time_{false};
  double global_controller_transfer_time_sec_{0.0};
  int entry_confirmation_count_{0};
  int exit_confirmation_count_{0};

  bool transition_active_{false};
  double transition_start_time_sec_{0.0};
  double transition_start_steering_rad_{0.0};
  double transition_target_steering_rad_{0.0};
  double last_steering_rad_{0.0};
};

}  // namespace hybrid_path_tracking

#endif  // HYBRID_PATH_TRACKING_HYBRID_SUPERVISOR_HPP_
