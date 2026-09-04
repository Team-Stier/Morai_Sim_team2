"""ROS-message-shaped validation helpers without a ROS runtime dependency."""

from __future__ import annotations

import math

from .geometry import angular_distance
from .models import Point2D


def finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def yaw_from_quaternion(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not finite(*values):
        raise ValueError("non_finite_orientation")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        raise ValueError("invalid_orientation")
    x_value, y_value, z_value, w_value = (value / norm for value in values)
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


def observation_reason(message, now_sec, timeout_sec, frame_id):
    if message is None:
        return "missing_input"
    if message.header.frame_id != frame_id:
        return "frame_mismatch"
    stamp_sec = message.header.stamp.to_sec()
    if not finite(stamp_sec, now_sec, timeout_sec) or stamp_sec <= 0.0:
        return "invalid_timestamp"
    age = now_sec - stamp_sec
    if age < 0.0:
        return "future_timestamp"
    if age > timeout_sec:
        return "stale_input"
    return ""


def odometry_payload_reason(odometry):
    """Validate every numeric payload carried by ``nav_msgs/Odometry``.

    The central contract rejects *any* non-finite odometry value.  Keeping
    this helper ROS-independent makes that rule directly regression-testable
    instead of relying on downstream pose conversion to fail by accident.
    """

    position = odometry.pose.pose.position
    twist = odometry.twist.twist
    if not finite(position.x, position.y, position.z):
        return "non_finite_ego_position"
    try:
        yaw_from_quaternion(odometry.pose.pose.orientation)
    except ValueError as error:
        return str(error)
    if not finite(
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    ):
        return "non_finite_ego_twist"
    pose_covariance = tuple(odometry.pose.covariance)
    twist_covariance = tuple(odometry.twist.covariance)
    if len(pose_covariance) != 36 or not finite(*pose_covariance):
        return "invalid_pose_covariance"
    if len(twist_covariance) != 36 or not finite(*twist_covariance):
        return "invalid_twist_covariance"
    return ""


def aligned_path_index(
    path,
    ego,
    maximum_position_error_m,
    maximum_yaw_error_rad,
    minimum_remaining_distance_m,
):
    """Gate a snapshot path against current tracking error and useful horizon.

    The returned index is diagnostic only.  Callers must retain the complete
    snapshot path: trimming it would implicitly create an unverified motion
    from the newest ego pose to the selected sample.  Controller latency and
    nearest-point tracking remain outside this planner's collision proof.
    """

    if not path or not finite(
        ego.x,
        ego.y,
        ego.yaw,
        maximum_position_error_m,
        maximum_yaw_error_rad,
        minimum_remaining_distance_m,
    ):
        return None
    if (
        maximum_position_error_m < 0.0
        or maximum_yaw_error_rad < 0.0
        or minimum_remaining_distance_m < 0.0
    ):
        return None
    best_index = min(
        range(len(path)),
        key=lambda index: math.hypot(
            path[index].pose.x - ego.x,
            path[index].pose.y - ego.y,
        ),
    )
    best = path[best_index]
    if (
        math.hypot(best.pose.x - ego.x, best.pose.y - ego.y)
        > maximum_position_error_m
        or angular_distance(best.pose.yaw, ego.yaw) > maximum_yaw_error_rad
        or path[-1].distance_from_start_m - best.distance_from_start_m
        < minimum_remaining_distance_m
    ):
        return None
    return best_index


def select_forward_valid_goal(
    lanes,
    start_progress_m,
    preferred_distance_m,
    minimum_distance_m,
    final_end_margin_m,
    scan_step_m,
    pose_is_valid,
):
    """Choose a map-centerline goal without stopping on an unsafe seam.

    MGeo link endpoints can lie inside a conservative vehicle footprint near
    a turn bridge.  The preferred lookahead is tried first, then farther
    points, then shorter fallbacks.  Every returned pose is checked by the
    caller's complete corridor/obstacle predicate.
    """

    lanes = tuple(lanes)
    if not lanes or not finite(
        start_progress_m,
        preferred_distance_m,
        minimum_distance_m,
        final_end_margin_m,
        scan_step_m,
    ):
        raise ValueError("invalid forward-goal inputs")
    if (
        start_progress_m < 0.0
        or preferred_distance_m <= 0.0
        or minimum_distance_m <= 0.0
        or final_end_margin_m < 0.0
        or scan_step_m <= 0.0
    ):
        raise ValueError("invalid forward-goal limits")
    if start_progress_m > lanes[0].length_m:
        raise ValueError("start progress exceeds source lane")

    available = lanes[0].length_m - start_progress_m
    available += sum(lane.length_m for lane in lanes[1:])
    maximum_distance = available - final_end_margin_m
    if maximum_distance < minimum_distance_m:
        raise ValueError("route horizon is too short for a safe local goal")
    preferred = min(preferred_distance_m, maximum_distance)

    def pose_at(distance_m):
        remaining = distance_m
        for index, lane in enumerate(lanes):
            origin = start_progress_m if index == 0 else 0.0
            lane_available = lane.length_m - origin
            if remaining <= lane_available or index == len(lanes) - 1:
                progress = min(lane.length_m, origin + remaining)
                return lane.pose_at(progress), lane.link_id
            remaining -= lane_available
        raise AssertionError("forward route distance was not resolved")

    candidates = [preferred]
    value = preferred + scan_step_m
    while value < maximum_distance:
        candidates.append(value)
        value += scan_step_m
    if candidates[-1] < maximum_distance:
        candidates.append(maximum_distance)
    value = preferred - scan_step_m
    while value > minimum_distance_m:
        candidates.append(value)
        value -= scan_step_m
    if minimum_distance_m not in candidates:
        candidates.append(minimum_distance_m)

    for distance_m in candidates:
        pose, lane_id = pose_at(distance_m)
        if pose_is_valid(pose):
            return pose, lane_id
    raise ValueError("no safe centerline goal in route horizon")


def lead_reason(
    lead,
    now_sec,
    timeout_sec,
    frame_id,
    route_link_id,
    minimum_confidence,
):
    reason = observation_reason(lead, now_sec, timeout_sec, frame_id)
    if reason:
        return reason
    position = lead.pose.position
    orientation = lead.pose.orientation
    twist = lead.twist
    if not finite(
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
        lead.length_m,
        lead.width_m,
        lead.longitudinal_distance_m,
        lead.confidence,
        minimum_confidence,
    ):
        return "non_finite_lead"
    if not lead.valid:
        return "lead_not_valid"
    try:
        yaw_from_quaternion(lead.pose.orientation)
    except ValueError as error:
        return str(error)
    if lead.length_m <= 0.0 or lead.width_m <= 0.0:
        return "invalid_lead_dimensions"
    if lead.longitudinal_distance_m < 0.0:
        return "invalid_lead_gap"
    if not 0.0 <= lead.confidence <= 1.0:
        return "invalid_lead_confidence"
    if lead.confidence < minimum_confidence:
        return "low_lead_confidence"
    if lead.lane_link_id != route_link_id:
        return "lead_lane_mismatch"
    return ""


def lead_fault_reason(
    lead,
    now_sec,
    timeout_sec,
    frame_id,
    route_link_id,
    minimum_confidence,
):
    """Return faults while allowing only an explicit fresh no-lead state."""

    reason = lead_reason(
        lead,
        now_sec,
        timeout_sec,
        frame_id,
        route_link_id,
        minimum_confidence,
    )
    if reason == "lead_not_valid":
        return ""
    return reason


def same_route_snapshot(first, latest):
    return bool(
        first is not None
        and latest is not None
        and latest.valid
        and first.current_link_id == latest.current_link_id
        and tuple(first.horizon_link_ids) == tuple(latest.horizon_link_ids)
        and bool(first.speed_limit_exempt_zone)
        == bool(latest.speed_limit_exempt_zone)
    )


def lane_change_reference(start, goal):
    """Return a smooth cubic guide between two rear-axle poses."""

    direct = math.hypot(goal.x - start.x, goal.y - start.y)
    sample_count = max(3, int(math.ceil(direct)) + 1)
    tangent = max(3.0, direct)
    points = []
    for index in range(sample_count):
        ratio = float(index) / float(sample_count - 1)
        ratio_squared = ratio * ratio
        ratio_cubed = ratio_squared * ratio
        h00 = 2.0 * ratio_cubed - 3.0 * ratio_squared + 1.0
        h10 = ratio_cubed - 2.0 * ratio_squared + ratio
        h01 = -2.0 * ratio_cubed + 3.0 * ratio_squared
        h11 = ratio_cubed - ratio_squared
        points.append(
            Point2D(
                h00 * start.x
                + h10 * tangent * math.cos(start.yaw)
                + h01 * goal.x
                + h11 * tangent * math.cos(goal.yaw),
                h00 * start.y
                + h10 * tangent * math.sin(start.yaw)
                + h01 * goal.y
                + h11 * tangent * math.sin(goal.yaw),
            )
        )
    return tuple(points)


def nearest_lane_id(point, lanes, fallback=""):
    best_id = str(fallback)
    best_distance = math.inf
    for lane in lanes:
        _progress, lateral_distance = lane.nearest_progress(point)
        if lateral_distance < best_distance:
            best_distance = lateral_distance
            best_id = lane.link_id
    return best_id
