"""Original clustered returns, without boxes or coordinate relabeling."""
import colorsys
import copy
import math
import struct

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker
from visualization_pkg.lidar_display import LidarDisplay


def decode_clusters(message, max_points=100000):
    count=message.width*message.height
    if message.is_bigendian or message.point_step<=0 or count>max_points or message.row_step<message.width*message.point_step or len(message.data)!=message.row_step*message.height:
        raise ValueError('invalid cluster point layout or size')
    offsets={}
    for name,kind in [('x',PointField.FLOAT32),('y',PointField.FLOAT32),('z',PointField.FLOAT32),
                      ('cluster_id',PointField.UINT32),('source_index',PointField.UINT32)]:
        fields=[f for f in message.fields if f.name==name]
        if len(fields)!=1 or fields[0].datatype!=kind or fields[0].count!=1 or fields[0].offset+4>message.point_step:
            raise ValueError('invalid cluster field '+name)
        offsets[name]=fields[0].offset
    groups={};seen=set()
    for row in range(message.height):
        for col in range(message.width):
            base=row*message.row_step+col*message.point_step
            xyz=[struct.unpack_from('<f',message.data,base+offsets[name])[0] for name in ('x','y','z')]
            cluster=struct.unpack_from('<I',message.data,base+offsets['cluster_id'])[0]
            source=struct.unpack_from('<I',message.data,base+offsets['source_index'])[0]
            if not all(math.isfinite(v) for v in xyz) or cluster>2147483647 or source in seen:
                raise ValueError('invalid coordinate, cluster id or duplicate source index')
            seen.add(source)
            groups.setdefault(cluster,[]).append(Point(*xyz))
    return groups


class ClusterPointsDisplay(LidarDisplay):
    def __init__(self,config,tf_buffer,publisher,point_size=.06,max_points=100000):
        super().__init__(config,tf_buffer,publisher)
        if not math.isfinite(point_size) or point_size<=0 or max_points<1:
            raise ValueError('invalid point display settings')
        self.point_size,self.max_points=point_size,max_points
        self.groups={}
        self.reject_before=None

    def reset_epoch(self,stamp):
        self.clear()
        self.reject_before=stamp

    def update(self,now,wall):
        if self.last_clock is not None and now<self.last_clock:
            self.reject_before=None
        super().update(now,wall)

    def ingest(self,message,now,wall):
        self.update(now,wall)
        try:
            if message.header.frame_id!='lidar_link' or message.header.stamp.is_zero():
                raise ValueError('invalid original scan frame/stamp')
            if self.reject_before is not None and message.header.stamp<self.reject_before:
                raise ValueError('scan predates localization reset')
            if self.last_stamp is not None and message.header.stamp<=self.last_stamp:
                raise ValueError('duplicate or regressing scan')
            if not 0<=(now-message.header.stamp).to_sec()<=self.config.display_timeout_sec:
                raise ValueError('stale or future cluster points')
            groups=decode_clusters(message,self.max_points)
        except (ValueError,TypeError,AttributeError,struct.error) as error:
            self.clear();rospy.logwarn_throttle(2,'Cluster point display rejected input: %s',error)
            return
        self.last_stamp=message.header.stamp
        if not groups:
            self.clear();return
        self.groups=groups
        self.pending,self.received=message,wall
        self.update(now,wall)

    def make_markers(self,message):
        markers=[Marker(action=Marker.DELETEALL)]
        for cluster,points in sorted(self.groups.items()):
            m=Marker();m.header=copy.deepcopy(message.header)
            m.ns='lidar_cluster_raw_points';m.id=cluster
            m.type=Marker.POINTS;m.action=Marker.ADD;m.pose.orientation.w=1.
            m.points=points;m.scale.x=m.scale.y=self.point_size
            m.color.r,m.color.g,m.color.b=colorsys.hsv_to_rgb((cluster*.61803398875)%1,.75,1.)
            m.color.a=1.;m.lifetime=rospy.Duration(self.config.display_timeout_sec)
            m.frame_locked=False
            markers.append(m)
        return markers
