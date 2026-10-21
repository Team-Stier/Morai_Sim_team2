import io
from pathlib import Path
import unittest

import yaml
from common_msgs_pkg.msg import HdMap, StaticMapObstacle
from geometry_msgs.msg import Point


class StaticMapContractTest(unittest.TestCase):
    def test_schema_registry_and_wire_roundtrip(self):
        root = Path(__file__).resolve().parents[3]
        central = root/'src/ros_architecture_pkg/config'
        contract = yaml.safe_load((central/'interface_contract.yaml').read_text())
        module = contract['contract_modules']['rddf_planning_messages']
        schema = yaml.safe_load((central/module['path']).read_text())
        self.assertEqual(module['required_contract_version'], schema['contract_version'])
        registry = {row['type'] for row in contract['messages']}
        for name, row in schema['messages'].items():
            self.assertEqual((root/'src/common_msgs_pkg/msg'/(name+'.msg')).read_text(),
                             row['wire_definition'])
        for name in ('HdMap', 'StaticMapObstacle'):
            self.assertIn('common_msgs_pkg/'+name, registry)
        message = HdMap(map_id='static-version', static_obstacles=[
            StaticMapObstacle(id='source-id', kind='wall', points=[Point(1.,2.,28.)])])
        message.header.frame_id = 'map'
        wire = io.BytesIO()
        message.serialize(wire)
        self.assertEqual(HdMap().deserialize(wire.getvalue()), message)
