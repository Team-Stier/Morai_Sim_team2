"""Producer -> ROS1 bytes -> consumer checks, without a ROS master.

Uses native genpy after catkin build. On a non-ROS host, the optional rosbags
Noetic typestore checks ROS1 wire layout; it does NOT prove catkin builds.
"""
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fixtures import component, ego, localization

PACKAGE = Path(__file__).resolve().parents[1]
BACKEND = None
try:
    from common_msgs_pkg.msg import ComponentStatus, EgoState, LocalizationStatus
    NATIVE_TYPES = dict(ComponentStatus=ComponentStatus, EgoState=EgoState,
                        LocalizationStatus=LocalizationStatus)
    BACKEND = 'genpy'
except ImportError:
    try:
        import numpy as np
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
        STORE = get_typestore(Stores.ROS1_NOETIC)
        for name in ('ComponentStatus', 'EgoState', 'LocalizationStatus'):
            STORE.register(get_types_from_msg((PACKAGE / 'msg' / (name + '.msg')).read_text(),
                                             'common_msgs_pkg/msg/' + name))
        BACKEND = 'rosbags ROS1_NOETIC'
    except ImportError:
        pass


def fill_native(target, source):
    for field in target.__slots__:
        current, value = getattr(target, field), getattr(source, field)
        if hasattr(current, '__slots__'):
            fill_native(current, value)
        else:
            setattr(target, field, value)
    return target


def typed_fixture(typename, source):
    values = {}
    for field, descriptor in STORE.fielddefs[typename][1]:
        value = getattr(source, field)
        kind, detail = descriptor
        if kind.name == 'NAME':
            value = typed_fixture(detail, value)
        elif kind.name == 'ARRAY':
            value = np.asarray(value, dtype=bool if detail[0][1][0] == 'bool' else np.float64)
        values[field] = value
    return STORE.types[typename](**values)


def roundtrip(name, source):
    if BACKEND == 'genpy':
        message = fill_native(NATIVE_TYPES[name](), source)
        buffer = io.BytesIO()
        message.serialize(buffer)
        return NATIVE_TYPES[name]().deserialize(buffer.getvalue())
    typename = 'common_msgs_pkg/msg/' + name
    message = typed_fixture(typename, source)
    return STORE.deserialize_ros1(STORE.serialize_ros1(message, typename), typename)


@unittest.skipUnless(BACKEND, 'Build catkin/genpy or install optional rosbags to verify ROS1 wire format')
class SerializationTest(unittest.TestCase):
    def test_component_fields_survive(self):
        value = roundtrip('ComponentStatus', component())
        self.assertEqual(value.component, 'path_planning_pkg')
        self.assertEqual(value.processed_count, 7)
        self.assertEqual(value.data_age_sec, 1.)

    def test_ego_masks_and_covariance_survive(self):
        value = roundtrip('EgoState', ego())
        self.assertEqual(list(value.pose_valid), [True, True, False, False, False, True])
        self.assertFalse(value.twist_valid[1])
        self.assertEqual(value.twist.twist.linear.x, -2.)
        self.assertEqual(value.pose.covariance[7], 0.5)
        self.assertEqual(value.reset_id, 2)

    def test_blackout_is_not_lost(self):
        value = roundtrip('LocalizationStatus', localization())
        self.assertEqual(value.mode, 3)
        self.assertFalse(value.gps_fix_valid)
        self.assertTrue(value.local_odometry_valid)
        self.assertFalse(value.stop_required)

    def test_unavailable_sentinel_survives(self):
        message = localization()
        message.gps_age_sec = -1.
        self.assertEqual(roundtrip('LocalizationStatus', message).gps_age_sec, -1.)


if __name__ == '__main__':
    print('Serialization backend:', BACKEND or 'unavailable')
    unittest.main()
