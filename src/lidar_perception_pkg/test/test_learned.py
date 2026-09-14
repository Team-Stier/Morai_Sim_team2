#!/usr/bin/env python3
import copy
import unittest

import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs.point_cloud2 import create_cloud
from common_msgs_pkg.msg import LidarObservationArray
from common_msgs_pkg.lidar_validation import validate_lidar
from lidar_perception_pkg.learned import read_xyzi, observations_from_predictions


def cloud(points):
    return create_cloud(Header(frame_id='lidar_link'),
                        [PointField(n,i*4,PointField.FLOAT32,1) for i,n in enumerate(('x','y','z','intensity'))], points)


class LearnedTest(unittest.TestCase):
    def test_xyzi_and_row_padding(self):
        msg = cloud([(1,2,3,200), (4,5,6,20)])
        msg.height, msg.width, msg.row_step = 2, 1, 20
        msg.data = msg.data[:16]+b'xxxx'+msg.data[16:]+b'yyyy'
        np.testing.assert_array_equal(read_xyzi(msg), [[1,2,3,200],[4,5,6,20]])

    def test_reject_bad_layout_missing_intensity_and_nonfinite(self):
        original = cloud([(1,2,3,200)])
        mutations = [lambda m: setattr(m,'data',b''),
                     lambda m: m.fields.pop(),
                     lambda m: setattr(m.fields[1],'offset',0),
                     lambda m: setattr(m,'is_bigendian',True)]
        for mutate in mutations:
            msg = copy.deepcopy(original); mutate(msg)
            with self.assertRaises(ValueError): read_xyzi(msg)
        for point in [(float('nan'),0,0,0), (0,0,0,256), (0,0,0,float('nan'))]:
            with self.assertRaises(ValueError): read_xyzi(cloud([point]))

    def test_mapping_support_and_enclosing_box(self):
        points = np.array([[5,0,0,10], [5,1.5,0,10], [6.5,0,0,10]], dtype=float)
        boxes = np.array([[5,0,0,4,2,2,np.pi/2]]*3)
        objects = observations_from_predictions(points, boxes, [.9,.8,.7], [1,2,3],
                                                 ['pedestrian','bus','barrier'],
                                                 np.array([0,-15,-1.5,40,15,1]), .3)
        self.assertEqual([o.semantic_class for o in objects], [1,2,3])
        self.assertEqual([o.point_count for o in objects], [2,2,2])
        self.assertAlmostEqual(objects[0].size.x,2)
        self.assertAlmostEqual(objects[0].size.y,4)
        msg = LidarObservationArray(objects=objects, objects_valid=True,
                                    calibration_id='test', timestamp_provenance='ingress_fallback')
        msg.header.frame_id, msg.header.stamp.secs = 'lidar_link', 10
        validate_lidar(msg)
        with self.assertRaises(ValueError): validate_lidar(msg,for_fusion=True)
        objects[0].semantic_class = 2
        with self.assertRaises(ValueError): validate_lidar(msg)
        objects[0].semantic_class = 1; objects[0].confidence = .9
        with self.assertRaises(ValueError): validate_lidar(msg)

    def test_no_fabricated_boxes_from_empty_support_or_low_score(self):
        bounds = np.array([0,-15,-1.5,40,15,1])
        points = np.array([[5,0,0,10]])
        boxes = np.array([[20,0,0,4,2,2,0], [5,0,0,4,2,2,0]])
        self.assertEqual(observations_from_predictions(points,boxes,[.9,.1],[1,1],['car'],bounds,.3),[])
        with self.assertRaises(ValueError):
            observations_from_predictions(points,boxes,[.9,float('nan')],[1,1],['car'],bounds,.3)

if __name__ == '__main__':
    unittest.main()
