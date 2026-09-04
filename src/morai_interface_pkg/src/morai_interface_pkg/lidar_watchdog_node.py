# -*- coding: utf-8 -*-
"""
lidar_watchdog_node.py
- 역할: LiDAR는 자체 파서가 없고 velodyne_driver+pointcloud가 담당하므로,
        출력 토픽(/sensors/lidar/points) 수신 여부만 감시해 상태를 노출한다.
- 주요 클래스: MoraiLidarWatchdog
인터페이스
- sub ~input_topic (기본 /sensors/lidar/points): sensor_msgs/PointCloud2
- pub ~status_topic (기본 /sensors/lidar/status): std_msgs/Bool (True=EXTERNAL_OK)

상태(디버깅 인터페이스용): EXTERNAL_OK(points 수신) / NO_POINTS(미수신).
points가 timeout 동안 안 오면 경고 로그 하나(throttle)를 남긴다. velodyne 내부는 관측 불가.
"""

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from morai_interface_pkg.watchdog_util import is_stale


class MoraiLidarWatchdog(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic",
                                           "/sensors/lidar/points")
        self.status_topic = rospy.get_param("~status_topic",
                                            "/sensors/lidar/status")
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 1.0))
        self.check_period_sec = float(rospy.get_param("~check_period_sec", 0.5))
        self.publish_status = bool(rospy.get_param("~publish_status", True))

        self._start_time = rospy.Time.now()
        self._last_recv_time = None
        self._ok = None  # 상태 전이 로깅용(EXTERNAL_OK/NO_POINTS 최초 판정 전 None)

        self._sub = rospy.Subscriber(self.input_topic, PointCloud2,
                                     self._callback, queue_size=1)
        if self.publish_status:
            self._status_pub = rospy.Publisher(self.status_topic, Bool,
                                               queue_size=1, latch=True)
        else:
            self._status_pub = None
        rospy.Timer(rospy.Duration(self.check_period_sec), self._check)

        rospy.loginfo("[morai_lidar_watchdog] watch '%s' (timeout=%.1fs) -> status '%s'",
                      self.input_topic, self.timeout_sec, self.status_topic)

    def _callback(self, _msg):
        self._last_recv_time = rospy.Time.now()

    # 함수이름: _check
    # 기능: 주기적으로 points 수신 staleness를 판정해 상태/로그를 갱신한다.
    def _check(self, _event):
        now = rospy.Time.now()
        reference = self._last_recv_time if self._last_recv_time is not None \
            else self._start_time
        stale = is_stale(now.to_sec(), reference.to_sec(), self.timeout_sec)
        ok = not stale

        if ok != self._ok:  # 상태 전이 시 한 번 로그
            if ok:
                rospy.loginfo("[morai_lidar_watchdog] EXTERNAL_OK: LiDAR points 수신")
            else:
                rospy.logwarn("[morai_lidar_watchdog] NO_POINTS: %s 미수신 -> "
                              "velodyne driver/port/rpm/모드 확인", self.input_topic)
            self._ok = ok
        elif not ok:  # NO_POINTS 지속 시 주기적으로 한 줄만
            rospy.logwarn_throttle(
                5.0, "[morai_lidar_watchdog] NO_POINTS 지속: %s 미수신" % self.input_topic)

        if self._status_pub is not None:
            self._status_pub.publish(Bool(data=ok))


def main():
    rospy.init_node("morai_lidar_watchdog")
    MoraiLidarWatchdog()
    rospy.spin()


if __name__ == "__main__":
    main()


