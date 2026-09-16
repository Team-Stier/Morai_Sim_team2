#!/usr/bin/env python3
"""Publish only centrally approved development GPS/IMU/LiDAR static transforms."""

import math
from pathlib import Path

import yaml


DEVELOPMENT_CHILDREN = {"gps_link", "imu_link", "lidar_link"}


def approved_static_poses(frames, extrinsics):
    """Validate both gates before returning any transform (all-or-nothing)."""
    frame_names = [frame["name"] for frame in frames["frames"]]
    if len(frame_names) != len(set(frame_names)):
        raise ValueError("duplicate frame name")
    parents = {}
    for transform in frames["transforms"]:
        parent, child = transform["parent"], transform["child"]
        if parent not in frame_names or child not in frame_names or child in parents:
            raise ValueError("unknown frame or multiple parents")
        parents[child] = parent
    for child in parents:
        chain = set()
        current = child
        while current in parents:
            if current in chain:
                raise ValueError("TF cycle")
            chain.add(current)
            current = parents[current]
        if current != frames["root_frame"]:
            raise ValueError("TF is not reachable from the central root")

    mounts = {mount["child_frame"]: mount for mount in extrinsics["sensor_mounts"]}
    results = []
    for transform in frames["transforms"]:
        if transform["type"] != "static":
            continue
        child = transform["child"]
        mount = mounts.get(child)
        if mount is not None and bool(transform["publish_enabled"]) != bool(mount["publish_enabled"]):
            raise ValueError("frame and extrinsic gates disagree: " + child)
        if not transform["publish_enabled"]:
            continue
        if child not in DEVELOPMENT_CHILDREN or mount is None:
            raise ValueError("static transform is outside the development scope: " + child)
        if transform["parent"] != "base_link" or transform["runtime_publisher_owner"] != "system_bringup_pkg":
            raise ValueError("static publisher ownership or parent mismatch")
        for entry in (transform, mount):
            if entry.get("activation_scope") != "development_only":
                raise ValueError("missing explicit development approval")
            if entry.get("physical_alignment_verified") is not False:
                raise ValueError("development publisher must not claim physical verification")
        evidence = extrinsics["source_evidence"][mount["source_evidence"]]
        if mount["key"] not in evidence["scope"] or not evidence.get("user_confirmed_mount_position"):
            raise ValueError("mount has no scoped user-approved evidence")
        if child == "lidar_link" and not (
                evidence.get("user_confirmed_coordinate_alignment") is True
                and evidence.get("runtime_activation_authorized") is True):
            raise ValueError("LiDAR requires explicit coordinate and activation approval")
        pose = mount["candidate_ros_pose"]
        xyz, rpy = pose["translation_m"], pose["rotation_rpy_rad"]
        if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
            raise ValueError("nonfinite or malformed static pose")
        roll, pitch, yaw = (angle / 2 for angle in rpy)
        sr, cr, sp, cp, sy, cy = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
        quaternion = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        results.append((transform["parent"], child, xyz, quaternion))
    return results


def main():
    import rospkg
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped

    rospy.init_node("sensor_tf_publisher", anonymous=False)
    config = Path(rospkg.RosPack().get_path("ros_architecture_pkg")) / "config" / "tf"
    with (config / "frame_contract.yaml").open() as stream:
        frames = yaml.safe_load(stream)
    with (config / "sensor_extrinsics.yaml").open() as stream:
        extrinsics = yaml.safe_load(stream)
    try:
        poses = approved_static_poses(frames, extrinsics)
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Central static TF contract rejected: %s", error)
        raise
    messages = []
    for parent, child, xyz, quaternion in poses:
        message = TransformStamped()
        message.header.stamp = rospy.Time(0)
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
        (message.transform.rotation.x, message.transform.rotation.y,
         message.transform.rotation.z, message.transform.rotation.w) = quaternion
        messages.append(message)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    if messages:
        broadcaster.sendTransform(messages)
    rospy.logwarn("Development static TF active for %s; physical alignment remains unverified",
                  ", ".join(message.child_frame_id for message in messages))
    rospy.spin()


if __name__ == "__main__":
    main()
