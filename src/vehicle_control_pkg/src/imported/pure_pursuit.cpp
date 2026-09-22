#include "hybrid_path_tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hybrid_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

struct LocalPoint {
  double x;
  double y;
};

bool isFinite(double value) { return std::isfinite(value); }

bool isFinitePoint(const Point2D& point) {
  return isFinite(point.x) && isFinite(point.y);
}

bool isFiniteState(const VehicleState2D& state) {
  return isFinite(state.x) && isFinite(state.y) && isFinite(state.yaw) &&
         isFinite(state.speed_kph) && state.speed_kph >= 0.0;
}

bool hasValidConfig(const PurePursuitConfig& config) {
  return isFinite(config.wheelbase_m) && config.wheelbase_m > 0.0 &&
         isFinite(config.max_steering_rad) && config.max_steering_rad > 0.0 &&
         isFinite(config.min_lookahead_m) && config.min_lookahead_m > 0.0 &&
         isFinite(config.max_lookahead_m) &&
         config.max_lookahead_m >= config.min_lookahead_m &&
         isFinite(config.lookahead_time_sec) &&
         config.lookahead_time_sec >= 0.0 &&
         isFinite(config.min_forward_path_m) && config.min_forward_path_m > 0.0;
}

ControllerCandidate invalidCandidate() {
  return {false, 0.0, 0.0, 0.0, 0.0, false, "pure_pursuit"};
}

double normalizeAngle(double angle) {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

bool toVehicleFrame(const VehicleState2D& state, const Point2D& point,
                    LocalPoint* local) {
  const double dx = point.x - state.x;
  const double dy = point.y - state.y;
  const double cosine = std::cos(state.yaw);
  const double sine = std::sin(state.yaw);
  local->x = cosine * dx + sine * dy;
  local->y = -sine * dx + cosine * dy;
  return isFinite(local->x) && isFinite(local->y);
}

bool findLookaheadTarget(const std::vector<LocalPoint>& points,
                         double lookahead_m, LocalPoint* target) {
  for (std::size_t index = 1; index < points.size(); ++index) {
    const LocalPoint& a = points[index - 1];
    const LocalPoint& b = points[index];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_squared = dx * dx + dy * dy;
    if (!isFinite(segment_squared) || segment_squared <= kEpsilon) {
      continue;
    }

    const double linear = 2.0 * (a.x * dx + a.y * dy);
    const double constant = a.x * a.x + a.y * a.y -
                            lookahead_m * lookahead_m;
    const double discriminant = linear * linear - 4.0 * segment_squared * constant;
    if (!isFinite(discriminant) || discriminant < 0.0) {
      continue;
    }

    const double root = std::sqrt(std::max(0.0, discriminant));
    const double roots[2] = {(-linear - root) / (2.0 * segment_squared),
                             (-linear + root) / (2.0 * segment_squared)};
    for (double fraction : roots) {
      if (fraction < -kEpsilon || fraction > 1.0 + kEpsilon) {
        continue;
      }
      fraction = std::max(0.0, std::min(1.0, fraction));
      const LocalPoint candidate = {a.x + fraction * dx, a.y + fraction * dy};
      if (candidate.x > kEpsilon && isFinite(candidate.x) &&
          isFinite(candidate.y)) {
        *target = candidate;
        return true;
      }
    }
  }
  return false;
}

bool findClosestForwardPoint(const std::vector<LocalPoint>& points,
                             LocalPoint* closest, double* heading_rad) {
  double best_squared = std::numeric_limits<double>::infinity();
  bool found = false;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const LocalPoint& a = points[index - 1];
    const LocalPoint& b = points[index];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_squared = dx * dx + dy * dy;
    if (!isFinite(segment_squared) || segment_squared <= kEpsilon) {
      continue;
    }
    const double fraction = std::max(
        0.0, std::min(1.0, -(a.x * dx + a.y * dy) / segment_squared));
    const LocalPoint projection = {a.x + fraction * dx, a.y + fraction * dy};
    if (projection.x < -kEpsilon) {
      continue;
    }
    const double squared = projection.x * projection.x + projection.y * projection.y;
    if (isFinite(squared) && squared < best_squared) {
      best_squared = squared;
      *closest = projection;
      *heading_rad = std::atan2(dy, dx);
      found = isFinite(*heading_rad);
    }
  }
  return found;
}

}  // namespace

PurePursuit::PurePursuit(PurePursuitConfig config) : config_(config) {}

ControllerCandidate PurePursuit::calculate(const VehicleState2D& state,
                                            const Path2D& path) const {
  if (!hasValidConfig(config_) || !isFiniteState(state) ||
      !isFinite(path.confidence) || path.points.size() < 2) {
    return invalidCandidate();
  }

  std::vector<LocalPoint> points;
  points.reserve(path.points.size());
  double maximum_forward_m = -std::numeric_limits<double>::infinity();
  for (const Point2D& point : path.points) {
    if (!isFinitePoint(point)) {
      return invalidCandidate();
    }
    LocalPoint local;
    if (!toVehicleFrame(state, point, &local)) {
      return invalidCandidate();
    }
    maximum_forward_m = std::max(maximum_forward_m, local.x);
    points.push_back(local);
  }
  if (!isFinite(maximum_forward_m) ||
      maximum_forward_m < config_.min_forward_path_m) {
    return invalidCandidate();
  }

  const double lookahead_m = std::max(
      config_.min_lookahead_m,
      std::min(config_.max_lookahead_m,
               config_.min_lookahead_m + config_.lookahead_time_sec *
                                             state.speed_kph / 3.6));
  if (!isFinite(lookahead_m)) {
    return invalidCandidate();
  }

  LocalPoint target;
  LocalPoint closest;
  double path_heading_rad = 0.0;
  if (!findLookaheadTarget(points, lookahead_m, &target) ||
      !findClosestForwardPoint(points, &closest, &path_heading_rad)) {
    return invalidCandidate();
  }

  const double alpha = normalizeAngle(std::atan2(target.y, target.x));
  const double raw_steering_rad = std::atan2(
      2.0 * config_.wheelbase_m * std::sin(alpha), lookahead_m);
  if (!isFinite(alpha) || !isFinite(raw_steering_rad)) {
    return invalidCandidate();
  }

  const double steering_rad = std::max(
      -config_.max_steering_rad,
      std::min(config_.max_steering_rad, raw_steering_rad));
  const double curvature = 2.0 * std::sin(alpha) / lookahead_m;
  const double heading_error = normalizeAngle(path_heading_rad);
  if (!isFinite(steering_rad) || !isFinite(curvature) ||
      !isFinite(heading_error) || !isFinite(closest.y)) {
    return invalidCandidate();
  }

  return {true,
          steering_rad,
          closest.y,
          heading_error,
          curvature,
          std::abs(raw_steering_rad) > config_.max_steering_rad,
          "pure_pursuit"};
}

}  // namespace hybrid_path_tracking
