#include "hybrid_path_tracking/path_source_manager.hpp"

#include <cmath>

namespace hybrid_path_tracking {
namespace {

bool finitePath(const Path2D& path) {
  if (!std::isfinite(path.confidence) || path.points.size() < 2U) {
    return false;
  }
  for (const auto& point : path.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return false;
    }
  }
  return true;
}

bool validConfig(const PathSourceConfig& config) {
  return std::isfinite(config.min_confidence) &&
         config.min_confidence >= 0.7 && config.min_confidence <= 1.0 &&
         std::isfinite(config.min_forward_length_m) &&
         config.min_forward_length_m >= 10.0 &&
         std::isfinite(config.stale_sec) && config.stale_sec >= 0.0 &&
         config.stale_sec <= 0.2;
}

}  // namespace

PathSourceManager::PathSourceManager(PathSourceConfig config)
    : config_(config), config_valid_(validConfig(config)) {}

void PathSourceManager::updateGlobal(const TimedPath& value) {
  global_ = value;
  has_global_ = true;
}

void PathSourceManager::updateVision(const TimedPath& value) {
  vision_ = value;
  has_vision_ = true;
}

void PathSourceManager::updateAvoidance(const TimedPath& value) {
  avoidance_ = value;
  has_avoidance_ = true;
}

SelectedPath PathSourceManager::select(
    const PathSelectionContext& context) const {
  if (!config_valid_ || !std::isfinite(context.now_sec)) {
    return invalid();
  }

  if (context.avoidance_active && has_avoidance_ &&
      isValid(avoidance_, context.now_sec)) {
    return selected(avoidance_, PathSource::AVOIDANCE);
  }

  if (!context.gps_blackout) {
    if (has_global_ && isValid(global_, context.now_sec)) {
      return selected(global_, PathSource::GLOBAL);
    }
    return invalid();
  }

  if (has_vision_ && isValid(vision_, context.now_sec)) {
    return selected(vision_, PathSource::VISION);
  }
  return invalid();
}

bool PathSourceManager::isValid(const TimedPath& value,
                                double now_sec) const {
  if (!std::isfinite(value.stamp_sec) ||
      !std::isfinite(value.forward_length_m) || !finitePath(value.path)) {
    return false;
  }

  const double age_sec = now_sec - value.stamp_sec;
  return age_sec >= 0.0 && age_sec <= config_.stale_sec &&
         value.path.confidence >= config_.min_confidence &&
         value.forward_length_m >= config_.min_forward_length_m;
}

SelectedPath PathSourceManager::selected(const TimedPath& value,
                                         PathSource source) const {
  return {true, source, value.path};
}

SelectedPath PathSourceManager::invalid() const {
  return {false, PathSource::GLOBAL, Path2D{{}, 0.0, 0}};
}

}  // namespace hybrid_path_tracking
