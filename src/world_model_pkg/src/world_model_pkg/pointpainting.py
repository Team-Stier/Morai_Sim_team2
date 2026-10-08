"""PointPainting projection and ROS serialization, using NumPy and ros_numpy."""
import math
from pathlib import Path
import numpy as np
import yaml
from tf.transformations import euler_matrix
from ros_numpy.point_cloud2 import fields_to_dtype, array_to_pointcloud2


def camera_geometry(config_root, width, height):
    """Read the central development calibration; never publish candidate TF."""
    root = Path(config_root)
    extrinsics = yaml.safe_load((root / 'tf/sensor_extrinsics.yaml').read_text())
    calibration = yaml.safe_load((root / 'messages/pointpainting.yaml').read_text())
    mounts = {entry['key']: entry['candidate_ros_pose'] for entry in extrinsics['sensor_mounts']}

    def pose(key):
        matrix = euler_matrix(*mounts[key]['rotation_rpy_rad'])
        matrix[:3, 3] = mounts[key]['translation_m']
        return matrix

    optical = euler_matrix(*extrinsics['camera_optical_convention']['rotation_rpy_rad'])
    lidar_to_camera = np.linalg.inv(pose('camera_front') @ optical) @ pose('lidar')
    focal = width / (2 * math.tan(math.radians(calibration['front_camera']['horizontal_fov_deg']) / 2))
    intrinsic = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])
    return lidar_to_camera, intrinsic


def paint(xyz, scores, lidar_to_camera, intrinsic):
    """Keep all points; append six probabilities and an observed/not-observed flag."""
    if scores.ndim != 3 or scores.shape[2] != 6 or not np.isfinite(scores).all():
        raise ValueError('expected finite HxWx6 probabilities')
    camera = xyz @ lidar_to_camera[:3, :3].T + lidar_to_camera[:3, 3]
    projected = camera @ intrinsic.T
    valid = np.isfinite(camera).all(1)
    valid[valid] &= camera[valid, 2] > 0
    uv = np.zeros((len(xyz), 2), np.float64)
    uv[valid] = projected[valid, :2] / projected[valid, 2:3]
    height, width = scores.shape[:2]
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    result = np.zeros((len(xyz), 6), np.float32)
    pixels = np.floor(uv[valid]).astype(np.int64)
    result[valid] = scores[pixels[:, 1], pixels[:, 0]]
    return result, valid


def cloud_records(cloud):
    # ros_numpy supplies field decoding; strides handle organized row padding and
    # dtype byte order handles big-endian sources which upstream numpify assumes away.
    dtype = np.dtype(fields_to_dtype(sorted(cloud.fields, key=lambda f: f.offset), cloud.point_step))
    dtype = dtype.newbyteorder('>' if cloud.is_bigendian else '<')
    return np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                      strides=(cloud.row_step, cloud.point_step)).reshape(-1)


def painted_cloud(cloud, score_image, scores, transform, intrinsic):
    records = cloud_records(cloud)
    xyz = np.column_stack([records[name] for name in ('x', 'y', 'z')])
    probabilities, observed = paint(xyz, scores, transform, intrinsic)
    original = [(field.name, records.dtype.fields[field.name][0].newbyteorder('<')) for field in cloud.fields]
    extras = [('score_' + str(i), '<f4') for i in range(6)]
    extras += [('painted', 'u1'), ('image_stamp_sec', '<u4'), ('image_stamp_nsec', '<u4')]
    if set(name for name, _ in original) & set(name for name, _ in extras):
        raise ValueError('cloud already contains painting fields')
    output = np.zeros(len(records), dtype=original + extras)
    for name, _ in original:
        output[name] = records[name]
    for i in range(6):
        output['score_' + str(i)] = probabilities[:, i]
    output['painted'] = observed
    output['image_stamp_sec'] = score_image.header.stamp.secs
    output['image_stamp_nsec'] = score_image.header.stamp.nsecs
    message = array_to_pointcloud2(output)
    message.header = cloud.header
    return message
