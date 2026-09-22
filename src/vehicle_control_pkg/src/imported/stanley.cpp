#include "hybrid_path_tracking/stanley.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hybrid_path_tracking/path_geometry.hpp"

namespace hybrid_path_tracking {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;
constexpr double kTieTolerance = 1e-9;

struct NearestProjection {
  Point2D point;
  double tangent_x;
  double tangent_y;
  double distance_squared;
  std::size_t segment_index;
  double fraction;
};

bool isFinite(double value) { return std::isfinite(value); }

bool isFinitePoint(const Point2D& point) {
  return isFinite(point.x) && isFinite(point.y);
}

bool isFiniteState(const VehicleState2D& state) {
  return isFinite(state.x) && isFinite(state.y) && isFinite(state.yaw) &&
         isFinite(state.speed_kph);
}

bool hasValidConfig(const StanleyConfig& config) {
  return isFinite(config.wheelbase_m) && config.wheelbase_m > 0.0 &&
         isFinite(config.heading_gain) && config.heading_gain >= 0.0 &&
         isFinite(config.cross_track_gain) && config.cross_track_gain >= 0.0 &&
         isFinite(config.softening_mps) && config.softening_mps >= 0.0 &&
         isFinite(config.max_steering_rad) && config.max_steering_rad > 0.0 &&
         isFinite(config.min_path_length_m) && config.min_path_length_m > 0.0;
}

ControllerCandidate invalidCandidate() {
  return {false, 0.0, 0.0, 0.0, 0.0, false, "stanley"};
}

double normalizeAngle(double angle) {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

bool nearlyEqual(double a, double b) {
  return std::abs(a - b) <=
         kTieTolerance * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

bool pointsNearlyEqual(const Point2D& a, const Point2D& b) {
  return nearlyEqual(a.x, b.x) && nearlyEqual(a.y, b.y);
}

bool hasSameDirectedTangent(const NearestProjection& a,
                            const NearestProjection& b) {
  const double cross = a.tangent_x * b.tangent_y -
                       a.tangent_y * b.tangent_x;
  const double dot = a.tangent_x * b.tangent_x + a.tangent_y * b.tangent_y;
  return std::abs(cross) <= kTieTolerance &&
         dot >= 1.0 - kTieTolerance;
}

bool hasReverseTangent(const NearestProjection& a,
                       const NearestProjection& b) {
  const double cross = a.tangent_x * b.tangent_y -
                       a.tangent_y * b.tangent_x;
  const double dot = a.tangent_x * b.tangent_x + a.tangent_y * b.tangent_y;
  return std::abs(cross) <= kTieTolerance &&
         dot <= -1.0 + kTieTolerance;
}

bool isOutgoingSharedVertex(const NearestProjection& prior,
                            const NearestProjection& next,
                            const std::vector<Point2D>& points) {
  return next.segment_index == prior.segment_index + 1 &&
         prior.fraction >= 1.0 - kTieTolerance &&
         next.fraction <= kTieTolerance &&
         pointsNearlyEqual(points[prior.segment_index + 1],
                           points[next.segment_index]);
}

bool findNearestProjection(const Point2D& front_axle,
                           const std::vector<Point2D>& points,
                           NearestProjection* nearest) {
  double best_distance_squared = std::numeric_limits<double>::infinity();
  bool found = false;
  bool ambiguous = false;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const Point2D& a = points[index - 1];
    const Point2D& b = points[index];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_squared = dx * dx + dy * dy;
    if (!isFinite(segment_squared) || segment_squared <= kEpsilon) {
      continue;
    }

    const double fraction = std::max(
        0.0, std::min(1.0, ((front_axle.x - a.x) * dx +
                             (front_axle.y - a.y) * dy) / segment_squared));
    const Point2D projection{a.x + fraction * dx, a.y + fraction * dy};
    const double error_x = front_axle.x - projection.x;
    const double error_y = front_axle.y - projection.y;
    const double distance_squared = error_x * error_x + error_y * error_y;
    if (!isFinitePoint(projection) || !isFinite(distance_squared)) {
      continue;
    }

    const double segment_length = std::sqrt(segment_squared);
    const NearestProjection candidate{projection,
                                      dx / segment_length,
                                      dy / segment_length,
                                      distance_squared,
                                      index - 1,
                                      fraction};
    if (!found ||
        distance_squared < best_distance_squared -
                               kTieTolerance *
                                   std::max(1.0, best_distance_squared)) {
      *nearest = candidate;
      best_distance_squared = distance_squared;
      found = true;
      ambiguous = false;
      continue;
    }
    if (!nearlyEqual(distance_squared, best_distance_squared)) {
      continue;
    }

    if (isOutgoingSharedVertex(*nearest, candidate, points)) {
      if (hasReverseTangent(*nearest, candidate)) {
        ambiguous = true;
        continue;
      }
      *nearest = candidate;
      best_distance_squared = distance_squared;
      continue;
    }
    if (!hasSameDirectedTangent(*nearest, candidate)) {
      ambiguous = true;
    }
  }
  return found && !ambiguous;
}

}  // namespace

StanleyConfig defaultStanleyConfig() { return StanleyConfig{}; }

Stanley::Stanley(StanleyConfig config) : config_(config) {}

ControllerCandidate Stanley::calculate(const VehicleState2D& state,
                                       const Path2D& path) const {
  if (!hasValidConfig(config_) || !isFiniteState(state) ||
      !isFinite(path.confidence) || path.points.size() < 2) {
    return invalidCandidate();
  }
  const CurvatureResult path_curvature =
      maximumAbsoluteCurvature(path.points);
  if (!path_curvature.valid) {
    return invalidCandidate();
  }

  const double cosine = std::cos(state.yaw);
  const double sine = std::sin(state.yaw);
  const Point2D front_axle{state.x + config_.wheelbase_m * cosine,
                            state.y + config_.wheelbase_m * sine};
  if (!isFinite(cosine) || !isFinite(sine) || !isFinitePoint(front_axle)) {
    return invalidCandidate();
  }

  double maximum_forward_m = -std::numeric_limits<double>::infinity();
  for (const Point2D& point : path.points) {
    if (!isFinitePoint(point)) {
      return invalidCandidate();
    }
    const double dx = point.x - state.x;
    const double dy = point.y - state.y;
    const double forward_m = cosine * dx + sine * dy;
    if (!isFinite(forward_m)) {
      return invalidCandidate();
    }
    maximum_forward_m = std::max(maximum_forward_m, forward_m);
  }
  if (maximum_forward_m < config_.min_path_length_m) {
    return invalidCandidate();
  }

  NearestProjection nearest;
  if (!findNearestProjection(front_axle, path.points, &nearest)) {
    return invalidCandidate();
  }

  const double error_x = front_axle.x - nearest.point.x;
  const double error_y = front_axle.y - nearest.point.y;
  // cross(front_axle - projection, tangent) is positive on the right of a
  // directed path.  With y left and positive steering left, that produces a
  // positive correction toward the path.
  const double side = error_x * nearest.tangent_y -
                      error_y * nearest.tangent_x;
  const double distance_m = std::sqrt(nearest.distance_squared);
  const double signed_cross_track_error =
      side > 0.0 ? distance_m : (side < 0.0 ? -distance_m : 0.0);
  const double path_yaw = std::atan2(nearest.tangent_y, nearest.tangent_x);
  const double heading_error = normalizeAngle(path_yaw - state.yaw);
  const double speed_mps = std::max(0.0, state.speed_kph / 3.6);
  const double cross_track_term = std::atan2(
      config_.cross_track_gain * signed_cross_track_error,
      speed_mps + config_.softening_mps);
  const double raw_steering =
      config_.heading_gain * heading_error + cross_track_term;
  if (!isFinite(signed_cross_track_error) || !isFinite(path_yaw) ||
      !isFinite(heading_error) || !isFinite(speed_mps) ||
      !isFinite(cross_track_term) || !isFinite(raw_steering)) {
    return invalidCandidate();
  }

  const double steering = std::max(
      -config_.max_steering_rad,
      std::min(config_.max_steering_rad, raw_steering));
  if (!isFinite(steering)) {
    return invalidCandidate();
  }

  return {true,
          steering,
          signed_cross_track_error,
          heading_error,
          path_curvature.max_abs_curvature,
          std::abs(raw_steering) > config_.max_steering_rad,
          "stanley"};
}

}  // namespace hybrid_path_tracking
