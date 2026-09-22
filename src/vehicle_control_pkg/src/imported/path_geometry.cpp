#include "hybrid_path_tracking/path_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hybrid_path_tracking {
namespace {

constexpr double kMinimumSegmentSquared = 1e-18;

bool FinitePoint(const Point2D& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double SquaredDistance(const Point2D& a, const Point2D& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return dx * dx + dy * dy;
}

}  // namespace

CurvatureResult maximumAbsoluteCurvature(
    const std::vector<Point2D>& points) {
  std::vector<Point2D> distinct_points;
  distinct_points.reserve(points.size());
  for (const Point2D& point : points) {
    if (!FinitePoint(point)) {
      return {false, 0.0};
    }
    if (!distinct_points.empty()) {
      const double separation_squared =
          SquaredDistance(distinct_points.back(), point);
      if (!std::isfinite(separation_squared)) {
        return {false, 0.0};
      }
      if (separation_squared <= kMinimumSegmentSquared) {
        continue;
      }
    }
    distinct_points.push_back(point);
  }

  if (distinct_points.size() < 2U) {
    return {false, 0.0};
  }

  double maximum = 0.0;
  for (std::size_t index = 2; index < distinct_points.size(); ++index) {
    const Point2D& a = distinct_points[index - 2];
    const Point2D& b = distinct_points[index - 1];
    const Point2D& c = distinct_points[index];
    const double ab_squared = SquaredDistance(a, b);
    const double bc_squared = SquaredDistance(b, c);
    const double ac_squared = SquaredDistance(a, c);
    if (!std::isfinite(ab_squared) || !std::isfinite(bc_squared) ||
        !std::isfinite(ac_squared) ||
        ab_squared <= kMinimumSegmentSquared ||
        bc_squared <= kMinimumSegmentSquared ||
        ac_squared <= kMinimumSegmentSquared) {
      return {false, 0.0};
    }

    const double cross =
        (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
    const double denominator =
        std::sqrt(ab_squared * bc_squared * ac_squared);
    if (!std::isfinite(cross) || !std::isfinite(denominator) ||
        denominator <= 0.0) {
      return {false, 0.0};
    }
    const double curvature = 2.0 * std::abs(cross) / denominator;
    if (!std::isfinite(curvature)) {
      return {false, 0.0};
    }
    maximum = std::max(maximum, curvature);
  }

  return {true, maximum};
}

}  // namespace hybrid_path_tracking
