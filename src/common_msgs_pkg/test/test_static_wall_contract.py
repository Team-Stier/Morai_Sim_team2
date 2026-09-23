import unittest
from pathlib import Path
import yaml
import rospy
from geometry_msgs.msg import Polygon, Point32
from common_msgs_pkg.msg import StaticWallMap
from common_msgs_pkg.static_wall_validation import validate_static_walls


class StaticWallContractTest(unittest.TestCase):
    def test_wire_matches_central_authority_and_validates_roundtrip(self):
        root=Path(__file__).resolve().parents[3]
        schema=yaml.safe_load((root/'src/ros_architecture_pkg/config/messages/static_wall_messages.yaml').read_text())
        self.assertEqual((root/'src/common_msgs_pkg/msg/StaticWallMap.msg').read_text(),schema['messages']['StaticWallMap']['wire_definition'])
        m=StaticWallMap(map_id='test',source_sha256='a'*64,wall_ids=['left','right'],horizontal_stddev_m=.4)
        m.header.frame_id='map';m.header.stamp=rospy.Time(1)
        m.baselines=[Polygon(points=[Point32(0,y,0),Point32(10,y,0)]) for y in (-6,6)]
        validate_static_walls(m)
        import io
        stream=io.BytesIO();m.serialize(stream);copy=StaticWallMap().deserialize(stream.getvalue())
        validate_static_walls(copy);self.assertEqual(copy,m)
        m.baselines[0].points[1].x=float('nan')
        with self.assertRaises(ValueError):validate_static_walls(m)

    def test_producer_consumer_use_exact_approved_boundary(self):
        root=Path(__file__).resolve().parents[3]
        contract=yaml.safe_load((root/'src/ros_architecture_pkg/config/interface_contract.yaml').read_text())
        topic=next(x for x in contract['topics'] if x['name']=='/molit/map/static_walls')
        self.assertEqual(topic['producers'],['hd_map_server_node'])
        self.assertEqual(topic['consumers'],['localization_node'])
        for package,direction in [('hd_map_pkg','outputs'),('localization_pkg','inputs')]:
            self.assertIn(topic['name'],contract['package_boundaries'][package][direction])
        self.assertEqual(topic['data_type'],'common_msgs_pkg/StaticWallMap')
