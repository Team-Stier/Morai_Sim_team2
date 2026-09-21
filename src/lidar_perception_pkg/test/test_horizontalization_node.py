#!/usr/bin/env python3
"""EgoState producer -> scan-time leveling -> existing lidar_link consumer."""
import time
import json
import unittest
import numpy as np
import rospy
import rostest
import tf2_ros
from tf.transformations import quaternion_from_euler, quaternion_matrix
from std_msgs.msg import Bool, Header, String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points
from geometry_msgs.msg import TransformStamped
from common_msgs_pkg.msg import EgoState, LidarObservationArray


class HorizontalizationTest(unittest.TestCase):
    def test_alignment_geometry_failure_and_reset(self):
        outputs, filtered, audits, cluster_clouds = [], [], [], []
        obs_sub = rospy.Subscriber('/molit/perception/lidar/observations', LidarObservationArray, outputs.append)
        roi_sub = rospy.Subscriber('/lidar_perception_node/filtered_points', PointCloud2, filtered.append)
        audit_sub = rospy.Subscriber('/lidar_perception_node/horizontalization_audit', String, lambda m: audits.append(json.loads(m.data)))
        cluster_sub = rospy.Subscriber('/molit/perception/lidar/cluster_points',PointCloud2,cluster_clouds.append)
        scan_pub = rospy.Publisher('/molit/sensors/lidar/points', PointCloud2, queue_size=5)
        ego_pub = rospy.Publisher('/molit/localization/ego_state', EgoState, queue_size=10)
        transport = rospy.Publisher('/molit/sensors/lidar/status', Bool, queue_size=1, latch=True)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_link', 'lidar_link'
        mount.transform.translation.x, mount.transform.translation.z = 2., 1.5
        mount.transform.rotation.w = 1.
        broadcaster.sendTransform(mount)
        deadline = time.monotonic()+5
        while time.monotonic()<deadline and (scan_pub.get_num_connections()==0 or ego_pub.get_num_connections()==0):
            time.sleep(.02)
        self.assertGreater(ego_pub.get_num_connections(), 0)
        time.sleep(.2)
        q = quaternion_from_euler(.15, .20, 1.1)
        rotation = quaternion_matrix(quaternion_from_euler(.15,.20,0))[:3,:3]
        # At x=10, this blob lies above raw ROI z_max=1; leveling must recover it.
        level = np.array([(10+i*.05,j*.05,k*.05) for i in range(4) for j in range(4) for k in range(2)])
        raw = level @ rotation
        self.assertTrue(np.all(raw[:,2]>1))

        def ego(stamp, epoch=0, valid=True, quaternion=q):
            m=EgoState(); m.header=Header(stamp=stamp,frame_id='map'); m.child_frame_id='base_link'
            m.pose_valid=[valid]*6; m.reset_id=epoch
            m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=quaternion
            ego_pub.publish(m); time.sleep(.025)

        def scan(stamp, xyz=raw):
            transport.publish(Bool(data=True)); time.sleep(.03)
            scan_pub.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='lidar_link'),xyz))

        def result(stamp):
            end=time.monotonic()+2
            while time.monotonic()<end:
                matches=[m for m in outputs if m.header.stamp==stamp]
                if matches:return matches[-1]
                time.sleep(.01)
            self.fail('no output at original scan stamp')

        # No orientation: fail closed instead of silently running raw detection.
        stamp=rospy.Time.now(); scan(stamp)
        self.assertFalse(result(stamp).objects_valid)
        # Scan arrives before the later localization sample (real estimator delay).
        stamp=rospy.Time.now()-rospy.Duration(.08)
        ego(stamp-rospy.Duration(.02)); scan(stamp)
        time.sleep(.03)
        self.assertFalse(any(m.header.stamp==stamp for m in outputs))
        ego(stamp+rospy.Duration(.02))
        out=result(stamp)
        self.assertTrue(out.objects_valid); self.assertEqual(len(out.objects),1)
        self.assertEqual(out.header.frame_id,'lidar_link')
        self.assertFalse(out.objects[0].learned_box)
        expected=raw.mean(axis=0)
        actual=out.objects[0].center
        np.testing.assert_allclose([actual.x,actual.y,actual.z],expected,atol=.003)
        # Debug coordinates return to lidar_link, including original tilted z.
        time.sleep(.05)
        debug=[m for m in filtered if m.header.stamp==stamp][-1]
        xyz=np.array(list(read_points(debug,field_names=('x','y','z'),skip_nans=True)))
        self.assertTrue(np.all(xyz[:,2]>1))
        self.assertEqual(debug.header.frame_id,'lidar_link')
        audit=[a for a in audits if int(a['stamp_ns'])==stamp.to_nsec()][-1]
        self.assertTrue(audit['leveling_enabled'])
        np.testing.assert_allclose(np.array(audit['rotation']).reshape(3,3),rotation,atol=1e-12)
        self.assertGreaterEqual(audit['processing_ms'],0)
        clustered=[m for m in cluster_clouds if m.header.stamp==stamp][-1]
        self.assertEqual(clustered.header,out.header)
        self.assertEqual(clustered.width,len(raw))
        entries=list(read_points(clustered,field_names=('x','y','z','cluster_id','source_index')))
        for x,y,z,cluster,index in entries:
            self.assertEqual(cluster,0)
            np.testing.assert_array_equal(np.array([x,y,z],dtype=np.float32),raw[index].astype(np.float32))

        # Missing future bracket cannot reuse the latest attitude indefinitely.
        time.sleep(.1); stamp=rospy.Time.now(); scan(stamp)
        self.assertFalse(result(stamp).objects_valid)
        # A reset must not interpolate between different localization epochs.
        stamp=rospy.Time.now()-rospy.Duration(.03)
        ego(stamp-rospy.Duration(.02)); scan(stamp)
        ego(stamp+rospy.Duration(.02),epoch=1)
        self.assertFalse(result(stamp).objects_valid)
        # Invalid pose flag also discards a waiting scan.
        stamp=rospy.Time.now()-rospy.Duration(.03)
        ego(stamp-rospy.Duration(.02),epoch=1); scan(stamp)
        ego(stamp+rospy.Duration(.02),epoch=1,valid=False)
        self.assertFalse(result(stamp).objects_valid)
        # Recover only with a fresh pair in the new epoch.
        time.sleep(.1); stamp=rospy.Time.now()-rospy.Duration(.03)
        ego(stamp-rospy.Duration(.02),epoch=1); scan(stamp)
        ego(stamp+rospy.Duration(.02),epoch=1)
        self.assertTrue(result(stamp).objects_valid)
        # Two samples exist, but a large bracket gap must still fail.
        time.sleep(.2); stamp=rospy.Time.now()-rospy.Duration(.08)
        ego(stamp-rospy.Duration(.07),epoch=1); scan(stamp)
        ego(stamp+rospy.Duration(.07),epoch=1)
        self.assertFalse(result(stamp).objects_valid)

        # Braking attitude: road enters raw z ROI, but is removed before DBSCAN.
        # Retained obstacle records still carry original sensor XYZ and indices.
        time.sleep(.2)
        ground=np.array([(10+i*.05,j*.05,-1.65) for i in range(4) for j in range(4)])
        # These body points stay in sensor coordinates even under roll/pitch.
        body=np.array([(-1.8+i*.05,-.1+j*.05,-.35) for i in range(4) for j in range(4)])
        mixed=np.vstack((ground @ rotation,body,level @ rotation))
        self.assertTrue(np.all(mixed[:len(ground),2]>-1.5))
        self.assertTrue(np.all(mixed[:len(ground),2]<1.0))
        stamp=rospy.Time.now()-rospy.Duration(.03)
        ego(stamp-rospy.Duration(.02),epoch=1); scan(stamp,mixed)
        ego(stamp+rospy.Duration(.02),epoch=1)
        out=result(stamp)
        self.assertTrue(out.objects_valid)
        self.assertEqual(len(out.objects),1)
        time.sleep(.05)
        clustered=[m for m in cluster_clouds if m.header.stamp==stamp][-1]
        self.assertEqual(clustered.header.frame_id,'lidar_link')
        entries=list(read_points(clustered,field_names=('x','y','z','cluster_id','source_index')))
        self.assertEqual(len(entries),len(level))
        for x,y,z,cluster,index in entries:
            self.assertGreaterEqual(index,len(ground)+len(body))
            self.assertEqual(cluster,0)
            np.testing.assert_array_equal(np.array([x,y,z],dtype=np.float32),mixed[index].astype(np.float32))


        # Real sloped ground still rises in the gravity-aligned frame. Remove it
        # before the ROI, retaining a 10 cm obstacle and raw record provenance.
        time.sleep(.2)
        road=np.array([(x*.5,y*.5,-1.9+.11*x*.5)
                       for x in range(-20,61) for y in range(-20,21)])
        low=np.array([(6+i*.05,2+j*.05,-1.9+.11*(6+i*.05)+.10)
                      for i in range(5) for j in range(5)])
        mixed=np.vstack((road,low)) @ rotation
        stamp=rospy.Time.now()-rospy.Duration(.03)
        ego(stamp-rospy.Duration(.02),epoch=1); scan(stamp,mixed)
        ego(stamp+rospy.Duration(.02),epoch=1)
        self.assertTrue(result(stamp).objects_valid)
        time.sleep(.1)
        clustered=[m for m in cluster_clouds if m.header.stamp==stamp][-1]
        entries=list(read_points(clustered,field_names=('x','y','z','source_index')))
        kept={int(v[3]) for v in entries}
        self.assertTrue(set(range(len(road),len(mixed))).issubset(kept))
        self.assertFalse(any(i<len(road) and 2<=road[i,0]<=12 and abs(road[i,1])<5 for i in kept))
        for x,y,z,i in entries:
            np.testing.assert_array_equal(np.array([x,y,z],np.float32),mixed[int(i)].astype(np.float32))
        audit=[a for a in audits if int(a['stamp_ns'])==stamp.to_nsec()][-1]
        self.assertGreater(audit['ground_removed'],1000)
        self.assertGreater(audit['ground_supported_cells'],0)


if __name__=='__main__':
    rospy.init_node('horizontalization_test')
    rostest.rosrun('lidar_perception_pkg','lidar_horizontalization_contract',HorizontalizationTest)
