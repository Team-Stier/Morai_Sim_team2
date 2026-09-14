import unittest
from unittest.mock import Mock,patch
import rospy
from sensor_msgs.msg import PointField
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_pkg.cluster_points_display import ClusterPointsDisplay,decode_clusters
from visualization_pkg.vehicle_display import DisplayConfig


class ClusterDisplayTest(unittest.TestCase):
    def setUp(self):
        self.tf,self.pub=Mock(),Mock();self.tf.can_transform.return_value=True
        self.display=ClusterPointsDisplay(DisplayConfig(),self.tf,self.pub)
        patch('rospy.logwarn_throttle').start();self.addCleanup(patch.stopall)

    def cloud(self,seconds=10):
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32 if i<4 else PointField.UINT32,count=1)
                for i,name in enumerate(('x','y','z','intensity','cluster_id','source_index'))]
        return create_cloud(Header(stamp=rospy.Time.from_sec(seconds),frame_id='lidar_link'),fields,
                            [(8.1,-2.2,-.9,42.,0,3),(8.2,-2.1,-.8,21.,0,6),(12.1,3.2,-.6,20.,1,9)])

    def test_points_exactly_preserved_without_pose_or_height_offset(self):
        msg=self.cloud();expected=decode_clusters(msg)
        self.display.ingest(msg,rospy.Time(10),1)
        self.tf.can_transform.assert_called_with('map','lidar_link',msg.header.stamp,rospy.Duration(0))
        markers=self.pub.publish.call_args[0][0].markers
        self.assertEqual(markers[0].action,Marker.DELETEALL)
        for m in markers[1:]:
            self.assertEqual(m.type,Marker.POINTS);self.assertEqual(m.header,msg.header)
            self.assertEqual(m.points,expected[m.id]);self.assertFalse(m.frame_locked)
            self.assertEqual((m.pose.position.x,m.pose.position.y,m.pose.position.z),(0,0,0))
            self.assertEqual(m.pose.orientation.w,1)

    def test_missing_scan_tf_stale_pause_and_reset_delete(self):
        self.tf.can_transform.return_value=False
        self.display.ingest(self.cloud(),rospy.Time(10),1)
        self.pub.publish.assert_not_called()
        self.tf.can_transform.return_value=True
        self.display.update(rospy.Time.from_sec(10.1),1.1)
        self.assertTrue(self.display.visible)
        self.display.update(rospy.Time.from_sec(10.1),2)
        self.assertFalse(self.display.visible)
        self.display.update(rospy.Time(5),3)
        self.display.ingest(self.cloud(5),rospy.Time(5),3)
        self.assertTrue(self.display.visible)
        self.display.reset_epoch(rospy.Time.from_sec(5.2))
        self.display.ingest(self.cloud(5.1),rospy.Time.from_sec(5.3),3.3)
        self.assertFalse(self.display.visible)

    def test_malformed_and_empty_cloud_clear_previous_points(self):
        for mutation in ('frame','future','layout','empty'):
            self.display=ClusterPointsDisplay(DisplayConfig(),self.tf,self.pub)
            self.display.ingest(self.cloud(),rospy.Time(10),1)
            msg=self.cloud(10.1)
            if mutation=='frame':msg.header.frame_id='map'
            if mutation=='future':msg.header.stamp=rospy.Time(12)
            if mutation=='layout':msg.fields[-1].offset=msg.point_step+1
            if mutation=='empty':msg.width=0;msg.row_step=0;msg.data=b''
            self.display.ingest(msg,rospy.Time.from_sec(10.1),1.1)
            self.assertFalse(self.display.visible)


if __name__=='__main__':unittest.main()
