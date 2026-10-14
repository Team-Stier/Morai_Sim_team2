import unittest
import numpy as np
import rospy
from ros_numpy.point_cloud2 import array_to_pointcloud2
from world_model_pkg.pointpainting import cloud_records
from world_model_pkg.semantic_objects import foreground_mask, foreground_cloud, semantic_groups

POLICY = {'classes': [1, 2, 3, 4, 5], 'minimum_probability': 0.6}
CONFIG = {'cell_size_m': 0.4, 'minimum_points': 3, 'maximum_points': 100}


def records():
    dtype = [(n, '<f4') for n in ('x', 'y', 'z', 'intensity')]
    dtype += [('painted', 'u1'), ('class_id', 'u1'), ('confidence', '<f4')]
    dtype += [('score_' + str(i), '<f4') for i in range(6)]
    result = np.zeros(9, dtype=dtype)
    result['x'] = [5, 5.1, 5.2, 5, 5.1, 5.2, 5, 5, 5]
    result['intensity'] = np.arange(9)
    result['painted'] = 1
    result['class_id'][:3] = 1; result['class_id'][3:6] = 4
    result['confidence'] = 1
    result['score_1'][:3] = 1; result['score_4'][3:6] = 1
    result['score_0'][6] = 1  # background
    result['painted'][7] = 0; result['class_id'][7] = 255; result['confidence'][7] = 0
    result['score_1'][8] = .55; result['score_0'][8] = .45
    result['class_id'][8] = 1; result['confidence'][8] = .55
    return result


class SemanticObjectsTest(unittest.TestCase):
    def test_selection_and_original_stamp_records(self):
        data = records()
        cloud = array_to_pointcloud2(data, stamp=rospy.Time(12, 34), frame_id='lidar_link')
        result = foreground_cloud(cloud, POLICY)
        self.assertEqual(result.header, cloud.header)
        selected = cloud_records(result)
        np.testing.assert_array_equal(selected, data[:6])
        self.assertEqual(len(semantic_groups(data, POLICY, CONFIG)), 2)

    def test_empty_foreground_is_valid_and_nan_geometry_is_excluded(self):
        data = records()[6:]
        self.assertFalse(foreground_mask(data, POLICY).any())
        self.assertEqual(semantic_groups(data, POLICY, CONFIG), [])
        data = records(); data['x'][0] = np.nan
        self.assertFalse(foreground_mask(data, POLICY)[0])
        self.assertEqual(len(semantic_groups(data, POLICY, CONFIG)), 1)
        self.assertEqual(semantic_groups(data[:0], POLICY, CONFIG), [])

    def test_malformed_scores_and_budgets_are_rejected(self):
        data = records(); data['score_1'][0] = np.nan
        with self.assertRaises(ValueError): foreground_mask(data, POLICY)
        data = records(); data['confidence'][0] = .7
        with self.assertRaises(ValueError): foreground_mask(data, POLICY)
        with self.assertRaises(ValueError): semantic_groups(records(), POLICY, dict(CONFIG, maximum_points=5))
