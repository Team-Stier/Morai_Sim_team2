import unittest
from pathlib import Path
import numpy as np
import yaml
from cv_bridge import CvBridge

ROOT = Path(__file__).resolve().parents[2]


class SemanticContractTest(unittest.TestCase):
    def test_six_channel_transport_and_class_order(self):
        bridge = CvBridge()
        scores = np.arange(36, dtype=np.float32).reshape(2, 3, 6)
        message = bridge.cv2_to_imgmsg(scores, encoding='passthrough')
        self.assertEqual(message.encoding, '32FC6')
        np.testing.assert_array_equal(bridge.imgmsg_to_cv2(message), scores)
        definition = yaml.safe_load((ROOT / 'ros_architecture_pkg/config/messages/pointpainting.yaml').read_text())
        self.assertEqual(definition['classes'], ['background', 'person', 'animal', 'box', 'bollard', 'barrier'])

    def test_producer_consumer_boundary_and_clock_contract(self):
        config = ROOT / 'ros_architecture_pkg/config'
        contract = yaml.safe_load((config / 'interface_contract.yaml').read_text())
        topics = {topic['name']: topic for topic in contract['topics']}
        scores = topics['/molit/perception/camera/front/semantic_scores']
        self.assertEqual(scores['producers'], ['camera_perception_node'])
        self.assertEqual(scores['consumers'], ['pointpainting_node'])
        self.assertEqual(scores['data_type'], 'sensor_msgs/Image')
        painted = topics['/molit/world_model/painted_points']
        self.assertEqual(painted['owner_package'], 'world_model_pkg')
        self.assertEqual(painted['frame'], 'lidar_link')
        registry = yaml.safe_load((config / 'timestamp/timestamp_contract.yaml').read_text())
        for topic in [scores, painted]:
            self.assertIn(topic['name'], registry['timestamp_source_registry'][topic['timestamp_source']]['allowed_topics'])
        extrinsics = yaml.safe_load((config / 'tf/sensor_extrinsics.yaml').read_text())
        self.assertFalse(extrinsics['camera_optical_convention']['publish_enabled'])


if __name__ == '__main__':
    unittest.main()
