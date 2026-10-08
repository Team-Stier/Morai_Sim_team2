import unittest
from pathlib import Path
import numpy as np
import rospy
from cv_bridge import CvBridge
from ros_numpy.point_cloud2 import array_to_pointcloud2
from world_model_pkg.pointpainting import paint, camera_geometry, cloud_records, painted_cloud

ROOT = Path(__file__).resolve().parents[2] / 'ros_architecture_pkg/config'


class PointPaintingTest(unittest.TestCase):
    def test_projection_and_missing_observation(self):
        scores = np.zeros((3, 4, 6), np.float32)
        scores[:, :, 0] = 0.2
        scores[:, :, 1] = 0.8
        scores[1, 2] = [0.1, 0.2, 0.3, 0.15, 0.15, 0.1]
        xyz = np.array([[2.9, 1.1, 1], [1, 1, -1], [4, 1, 1], [np.nan, 0, 1]])
        result, valid = paint(xyz, scores, np.eye(4), np.eye(3))
        np.testing.assert_array_equal(valid, [True, False, False, False])
        np.testing.assert_allclose(result[0], scores[1, 2])
        self.assertEqual(np.count_nonzero(result[1:]), 0)

    def test_central_mount_projection_roundtrip(self):
        transform, intrinsic = camera_geometry(ROOT, 1280, 720)
        camera = np.array([[0., 0., 10.], [1., 0., 10.]])
        lidar = (camera - transform[:3, 3]) @ transform[:3, :3]
        scores = np.zeros((720, 1280, 6), np.float32)
        scores[360, 640, 2] = 1
        scores[360, 704, 4] = 1
        result, valid = paint(lidar, scores, transform, intrinsic)
        np.testing.assert_array_equal(valid, [True, True])
        np.testing.assert_allclose(intrinsic.diagonal(), [640, 640, 1])
        np.testing.assert_allclose(result.sum(1), [1, 1])

    def test_padded_big_endian_cloud_preserves_fields_and_both_stamps(self):
        records = np.zeros((2, 2), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('ring', '<u2')])
        records['x'] = [[1, 2], [3, 4]]; records['y'] = 1; records['z'] = 1
        records['ring'] = [[1, 2], [3, 4]]
        cloud = array_to_pointcloud2(records)
        cloud.is_bigendian = True
        big = records.astype(records.dtype.newbyteorder('>'))
        cloud.row_step += 7
        cloud.data = b''.join(row.tobytes() + b'\0' * 7 for row in big)
        cloud.header.stamp = rospy.Time(123, 45); cloud.header.frame_id = 'lidar_link'
        scores = np.full((8, 8, 6), 1 / 6, np.float32)
        image = CvBridge().cv2_to_imgmsg(scores, encoding='passthrough')
        image.header.stamp = rospy.Time(123, 678)
        output = painted_cloud(cloud, image, scores, np.eye(4), np.eye(3))
        decoded = cloud_records(output)
        for name in records.dtype.names:
            np.testing.assert_array_equal(decoded[name], records[name].ravel())
        self.assertEqual(output.header, cloud.header)
        np.testing.assert_array_equal(decoded['image_stamp_nsec'], 678)
        np.testing.assert_allclose(sum(decoded['score_' + str(i)] for i in range(6)), 1, atol=1e-6)

    def test_nan_score_map_is_rejected(self):
        with self.assertRaises(ValueError):
            paint(np.ones((1, 3)), np.full((1, 1, 6), np.nan), np.eye(4), np.eye(3))


if __name__ == '__main__':
    unittest.main()
