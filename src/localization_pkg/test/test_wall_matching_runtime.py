#!/usr/bin/env python3
"""Isolated-master producer/consumer test; never run against a live MORAI master."""
import time
import unittest
import numpy as np
import rospy
import rostest
from pyproj import Transformer
from sensor_msgs.msg import Imu, NavSatFix, PointField
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Polygon
from common_msgs_pkg.msg import StaticWallMap, LocalizationStatus, EgoState


class WallRuntimeTest(unittest.TestCase):
    def test_map_and_measurement_time_cloud_reach_filter(self):
        rospy.init_node('wall_runtime_test', anonymous=True)
        received=[]; poses=[]
        subscriptions=[rospy.Subscriber('/molit/localization/status',LocalizationStatus,received.append),
                       rospy.Subscriber('/molit/localization/ego_state',EgoState,poses.append)]
        imu_pub=rospy.Publisher('/molit/sensors/imu/data',Imu,queue_size=100)
        gps_pub=rospy.Publisher('/molit/sensors/gps/fix',NavSatFix,queue_size=10)
        from sensor_msgs.msg import PointCloud2
        lidar_pub=rospy.Publisher('/molit/sensors/lidar/points',PointCloud2,queue_size=3)
        map_pub=rospy.Publisher('/molit/map/static_walls',StaticWallMap,queue_size=1,latch=True)
        deadline=time.monotonic()+8
        while not all(p.get_num_connections() for p in (imu_pub,gps_pub,lidar_pub,map_pub)):
            self.assertLess(time.monotonic(),deadline);time.sleep(.02)
        wall=StaticWallMap(map_id='synthetic-walls',source_sha256='a'*64,
                          wall_ids=['a','b'],horizontal_stddev_m=.4)
        wall.header=Header(stamp=rospy.Time.now(),frame_id='map')
        wall.baselines=[Polygon(points=[Point32(0,y,0),Point32(100,y,0)]) for y in (-6,6)]
        map_pub.publish(wall)
        points=[[x-2,y,z-1.5] for y in (-6,6) for x in np.linspace(-25,25,121) for z in (2,2.5,3)]
        lon,lat=Transformer.from_crs(32652,4326,always_xy=True).transform(302645,4124145.8)
        scan_stamps=set()
        for i in range(200):
            stamp=rospy.Time.now()
            msg=Imu(header=Header(stamp=stamp,frame_id='imu_link'))
            msg.orientation.w=1;msg.linear_acceleration.z=9.80665
            imu_pub.publish(msg)
            if i < 50 and i % 10 == 0:
                gps=NavSatFix(header=Header(stamp=stamp,frame_id='gps_link'),latitude=lat,longitude=lon,altitude=1.3)
                gps.status.status=0;gps_pub.publish(gps)
            if i%5 == 0:
                cloud=create_cloud_xyz32(Header(stamp=stamp,frame_id='lidar_link'),points)
                lidar_pub.publish(cloud);scan_stamps.add(stamp.to_nsec())
            time.sleep(.02)
        self.assertTrue(any('wall matched' in m.reason for m in received),[m.reason for m in received[-3:]])
        self.assertTrue(any(m.mode==m.DEAD_RECKONING and m.map_pose_valid for m in received))
        self.assertTrue(poses)
        self.assertLess(abs(poses[-1].pose.pose.position.y),.25)
        self.assertAlmostEqual(poses[-1].pose.pose.position.x,50,delta=.05)
        self.assertTrue(all(m.stop_required for m in received))
        # Loss of cloud support does not stamp-refresh the accepted match.
        time.sleep(.7)
        self.assertFalse(received[-1].map_pose_valid)


if __name__=='__main__':
    rostest.rosrun('localization_pkg','wall_matching_runtime',WallRuntimeTest)
