#!/usr/bin/env python3
"""Raw bytes -> accepted cluster -> POINTS marker -> independent map TF check."""
import struct
import time
import unittest
import numpy as np
import rospy
import rostest
import tf2_ros
from tf.transformations import quaternion_from_euler,quaternion_matrix
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header,Bool
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs.point_cloud2 import create_cloud,read_points
from visualization_msgs.msg import Marker,MarkerArray
from common_msgs_pkg.msg import EgoState


class ClusterPipelineTest(unittest.TestCase):
    def test_raw_records_scan_time_tf_and_marker_coordinates(self):
        clouds,markers=[],[]
        subs=[rospy.Subscriber('/molit/perception/lidar/cluster_points',PointCloud2,clouds.append),
              rospy.Subscriber('/molit/internal/visualization/lidar_markers',MarkerArray,markers.append)]
        pub=rospy.Publisher('/molit/sensors/lidar/points',PointCloud2,queue_size=1)
        ego_pub=rospy.Publisher('/molit/localization/ego_state',EgoState,queue_size=5)
        transport=rospy.Publisher('/molit/sensors/lidar/status',Bool,queue_size=1,latch=True)
        static=tf2_ros.StaticTransformBroadcaster();dynamic=tf2_ros.TransformBroadcaster()
        buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
        mount=TransformStamped();mount.header.frame_id='base_link';mount.child_frame_id='lidar_link'
        mount.transform.translation.x=2.;mount.transform.translation.z=1.5;mount.transform.rotation.w=1.
        static.sendTransform(mount)
        end=time.monotonic()+5
        while time.monotonic()<end and (pub.get_num_connections()==0 or ego_pub.get_num_connections()==0):time.sleep(.02)
        transport.publish(Bool(True));time.sleep(.2)
        stamp=rospy.Time.now()-rospy.Duration(.1)
        q=quaternion_from_euler(.1,.05,.7)
        ego=EgoState();ego.header=Header(stamp=stamp,frame_id='map');ego.child_frame_id='base_link';ego.pose_valid=[True]*6
        ego.pose.pose.orientation.x,ego.pose.pose.orientation.y,ego.pose.pose.orientation.z,ego.pose.pose.orientation.w=q
        ego_pub.publish(ego);time.sleep(.02)
        fields=[PointField(n,i*4,PointField.FLOAT32,1) for i,n in enumerate(('x','y','z','intensity'))]
        blob=[(6+i*.05,2+j*.05,-.5+k*.05,10+i+j) for i in range(4) for j in range(4) for k in range(2)]
        raw=create_cloud(Header(stamp=stamp,frame_id='lidar_link'),fields,blob)
        pub.publish(raw)
        end=time.monotonic()+.6
        while not clouds and time.monotonic()<end:time.sleep(.01)
        self.assertTrue(clouds);cloud=clouds[-1]
        self.assertGreater(cloud.width,0)
        self.assertFalse(any(m.type==Marker.POINTS and m.action==Marker.ADD for a in markers for m in a.markers))
        map_odom=TransformStamped();map_odom.header.frame_id='map';map_odom.child_frame_id='odom'
        map_odom.transform.translation.x=100.;map_odom.transform.translation.y=200.;map_odom.transform.rotation.w=1.
        static.sendTransform([mount,map_odom])
        transforms=[]
        for seconds,x in [(-.02,20.),(.02,22.)]:
            t=TransformStamped();t.header=Header(stamp=stamp+rospy.Duration(seconds),frame_id='odom');t.child_frame_id='base_link'
            t.transform.translation.x=x;t.transform.translation.y=4.
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=q
            transforms.append(t)
        dynamic.sendTransform(transforms)
        end=time.monotonic()+.8;shown=[]
        while time.monotonic()<end and not shown:
            shown=[m for a in markers for m in a.markers if m.type==Marker.POINTS and m.action==Marker.ADD]
            time.sleep(.01)
        self.assertTrue(shown)
        point_markers=[m for m in shown if m.header.stamp==stamp]
        self.assertTrue(point_markers)
        decoded=list(read_points(cloud,field_names=('x','y','z','intensity','cluster_id','source_index')))
        observed=[]
        for m in point_markers:
            self.assertEqual(m.header.frame_id,'lidar_link');self.assertFalse(m.frame_locked)
            self.assertEqual(m.pose.position.x,0);self.assertEqual(m.pose.position.z,0)
            observed.extend((m.id,p.x,p.y,p.z) for p in m.points)
        self.assertEqual(sorted(observed),sorted((cid,x,y,z) for x,y,z,intensity,cid,index in decoded))
        for x,y,z,intensity,cid,index in decoded:
            self.assertEqual(struct.pack('<ffff',x,y,z,intensity),raw.data[index*raw.point_step:(index+1)*raw.point_step])
        transform=buffer.lookup_transform('map','lidar_link',stamp,rospy.Duration(0))
        tq=transform.transform.rotation;tr=transform.transform.translation
        tf_rotation=quaternion_matrix([tq.x,tq.y,tq.z,tq.w])[:3,:3]
        body_rotation=quaternion_matrix(q)[:3,:3]
        for _,x,y,z in observed:
            actual=tf_rotation@np.array([x,y,z])+[tr.x,tr.y,tr.z]
            expected=np.array([121,204,0])+body_rotation@(np.array([2,0,1.5])+[x,y,z])
            np.testing.assert_allclose(actual,expected,atol=1e-6,rtol=0)
        latest=buffer.lookup_transform('map','lidar_link',rospy.Time(0),rospy.Duration(0))
        self.assertAlmostEqual(latest.transform.translation.x-tr.x,1.,places=6)
        time.sleep(2.2)
        self.assertTrue(all(m.action==Marker.DELETEALL for m in markers[-1].markers))


if __name__=='__main__':
    rospy.init_node('cluster_points_pipeline_test')
    rostest.rosrun('visualization_pkg','cluster_points_pipeline',ClusterPipelineTest)
