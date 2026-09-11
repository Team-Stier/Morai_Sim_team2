import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/common_msgs_pkg/src'))
from common_msgs_pkg.lidar_validation import validate_lidar


def fixture():
    return NS(header=NS(frame_id='lidar_link',stamp=NS(secs=10,nsecs=0)),
              calibration_id='saved-profile',calibration_verified=False,
              timestamp_provenance='ingress_fallback',freshness_verified=False,
              objects_valid=True,objects=[],ground_valid=False,free_space_valid=False,
              occupancy_valid=False,velocity_valid=False)


class LidarContract(unittest.TestCase):
    def test_runtime_timeouts_are_separate_and_status_period_matches(self):
        config=ROOT/'src/ros_architecture_pkg/config'
        runtime=yaml.safe_load((config/'messages/lidar_runtime.yaml').read_text(encoding='utf-8'))
        contract=yaml.safe_load((config/'interface_contract.yaml').read_text(encoding='utf-8'))
        self.assertEqual(runtime['max_scan_age_sec'],0)
        self.assertGreater(runtime['development_watchdog_sec'],0)
        status=next(t for t in contract['topics'] if t['name']=='/molit/perception/lidar/status')
        self.assertEqual(runtime['status_period_sec'],1/status['expected_rate_hz'])

    def test_exact_wire_and_registry(self):
        config=ROOT/'src/ros_architecture_pkg/config'
        schema=yaml.safe_load((config/'messages/lidar_messages.yaml').read_text(encoding='utf-8'))
        contract=yaml.safe_load((config/'interface_contract.yaml').read_text(encoding='utf-8'))
        module=contract['contract_modules']['lidar_messages']
        self.assertEqual(module['path'], 'messages/lidar_messages.yaml')
        self.assertEqual(module['required_contract_version'], schema['contract_version'])
        for name,entry in schema['messages'].items():
            self.assertEqual((ROOT/'src/common_msgs_pkg/msg'/f'{name}.msg').read_text(encoding='utf-8'),entry['wire_definition'])
            self.assertTrue(any(m['type']=='common_msgs_pkg/'+name and 'schema' in m for m in contract['messages']))

    def test_consumer_rejects_unverified_geometry(self):
        msg=fixture(); validate_lidar(msg)
        with self.assertRaises(ValueError): validate_lidar(msg,for_fusion=True)
        msg.objects_valid=False
        msg.objects=[NS(scan_local_id=0,point_count=1,center=NS(x=1,y=0,z=0),size=NS(x=1,y=1,z=1),confidence=-1)]
        with self.assertRaises(ValueError): validate_lidar(msg)

    def test_geometry_and_layers(self):
        msg=fixture()
        obj=NS(scan_local_id=0,point_count=1,center=NS(x=1.,y=0.,z=0.),size=NS(x=1.,y=1.,z=1.),confidence=-1.)
        msg.objects=[obj]; validate_lidar(msg)
        obj.size.x=-1
        with self.assertRaises(ValueError): validate_lidar(msg)
        obj.size.x=1; msg.ground_valid=True
        with self.assertRaises(ValueError): validate_lidar(msg)

    def test_ros1_roundtrip_variable_length(self):
        try:
            import io
            from common_msgs_pkg.msg import LidarObservationArray, LidarObjectObservation
            msg=LidarObservationArray()
            msg.header.frame_id='lidar_link'; msg.header.stamp.secs=10; msg.header.stamp.nsecs=20
            msg.objects=[LidarObjectObservation(scan_local_id=i,point_count=32,confidence=-1) for i in range(121)]
            stream=io.BytesIO(); msg.serialize(stream)
            output=LidarObservationArray().deserialize(stream.getvalue())
            self.assertEqual(len(output.objects),121)
            self.assertEqual(output.header.stamp.nsecs,20)
            return
        except ImportError:
            pass
        try:
            from rosbags.typesys import Stores,get_typestore,get_types_from_msg
        except ImportError:
            self.skipTest('optional rosbags required for non-ROS wire check')
        store=get_typestore(Stores.ROS1_NOETIC)
        for name in ('LidarObjectObservation','LidarObservationArray'):
            store.register(get_types_from_msg((ROOT/'src/common_msgs_pkg/msg'/f'{name}.msg').read_text(encoding='utf-8'),'common_msgs_pkg/msg/'+name))
        t=store.types
        obj=t['common_msgs_pkg/msg/LidarObjectObservation'](0,t['geometry_msgs/msg/Point'](2.,0.,0.),t['geometry_msgs/msg/Vector3'](1.,1.,1.),32,-1.)
        header=t['std_msgs/msg/Header'](0,t['builtin_interfaces/msg/Time'](10,20),'lidar_link')
        name='common_msgs_pkg/msg/LidarObservationArray'
        msg=t[name](header,'saved-profile',False,'ingress_fallback',False,True,[obj]*121,False,False,False,False)
        output=store.deserialize_ros1(store.serialize_ros1(msg,name),name)
        self.assertEqual(len(output.objects),121)
        self.assertEqual(output.header.stamp.nanosec,20)
        self.assertEqual(output.objects[0].point_count,32)

if __name__=='__main__': unittest.main()
