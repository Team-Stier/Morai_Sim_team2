"""Measurement-time map association and temporal confirmation of camera HSV lamps."""
from collections import deque
import math
import numpy as np


class SignalFusion:
    def __init__(self, policy, config):
        self.policy, self.config = policy, config
        self.poses = deque(maxlen=500)
        self.route = None
        self.latest = None
        self.confirmation = None
        self.reset_id = None

    def add_pose(self, ego):
        if self.reset_id != ego.reset_id or (self.poses and ego.header.stamp < self.poses[-1].header.stamp):
            self.poses.clear()
            self.latest = self.confirmation = None
        self.reset_id = ego.reset_id
        if all(ego.pose_valid[i] for i in (0, 1, 5)):
            self.poses.append(ego)

    def target(self, now):
        route = self.route
        if (route is None or route.reference_sha256 != self.policy['reference_sha256'] or
                route.header.frame_id != 'map' or route.route_complete or not math.isfinite(route.progress) or
                not 0 <= now-route.header.stamp.to_sec() <= self.policy['context_timeout_sec']):
            return None
        return next((t for t in self.policy['targets'] if
                     0 <= t['stop_station_m']-route.progress <= self.policy['approach_distance_m']), None)

    def pose_at(self, stamp):
        if not self.poses:
            return None
        nearest = min(self.poses, key=lambda p:abs(p.header.stamp.to_sec()-stamp))
        if abs(nearest.header.stamp.to_sec()-stamp) > self.config['maximum_pose_delta_sec']:
            return None
        before = [p for p in self.poses if p.header.stamp.to_sec() <= stamp]
        after = [p for p in self.poses if p.header.stamp.to_sec() >= stamp]
        if before and after and after[0].header.stamp != before[-1].header.stamp:
            a, b = before[-1], after[0]
            fraction = (stamp-a.header.stamp.to_sec())/(b.header.stamp-a.header.stamp).to_sec()
            xyz = lambda p:np.array([p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z])
            quat = lambda p:np.array([p.pose.pose.orientation.x,p.pose.pose.orientation.y,p.pose.pose.orientation.z,p.pose.pose.orientation.w])
            qa, qb = quat(a), quat(b)
            if np.dot(qa,qb) < 0:qb = -qb
            return xyz(a)+(xyz(b)-xyz(a))*fraction, qa+(qb-qa)*fraction
        p = nearest.pose.pose
        return np.array([p.position.x,p.position.y,p.position.z]), np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])

    def project(self, point, pose, width, height):
        position, q = pose
        norm = np.linalg.norm(q)
        if not np.isfinite(norm) or norm < .9:
            return None
        x,y,z,w = q/norm
        rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        camera = self.policy['camera_projection']
        v = rotation.T.dot(np.asarray(point)-position)-camera['translation_m']
        pitch = camera['pitch_rad'];c,s = math.cos(pitch),math.sin(pitch)
        forward,left,up = c*v[0]-s*v[2],v[1],s*v[0]+c*v[2]
        if forward <= .5:
            return None
        focal = width/(2*math.tan(math.radians(camera['horizontal_fov_deg'])/2))
        return np.array([width/2-focal*left/forward,height/2-focal*up/forward])

    def ingest(self, message, now):
        from common_msgs_pkg.msg import TrafficSignal
        target = self.target(now)
        if target is None:
            self.latest = self.confirmation = None
            return
        stamp = message.header.stamp.to_sec()
        if (message.header.frame_id != 'camera_signal_optical_frame' or
                not 0 <= now-stamp <= self.config['maximum_observation_age_sec'] or
                self.latest is not None and stamp <= self.latest.header.stamp.to_sec()):
            return
        output = TrafficSignal(header=message.header, stop_line_id=target['stop_line_id'],
                               valid=False, reason='no_matching_visible_lamp')
        output.header = type(message.header)(seq=message.header.seq,stamp=message.header.stamp,frame_id='map')
        output.position.x,output.position.y,output.position.z = target['heads'][0]['map_position']
        pose = self.pose_at(stamp)
        matches = []
        if pose is not None and message.image_width and message.image_height:
            for head in target['heads']:
                projected = self.project(head['map_position'],pose,message.image_width,message.image_height)
                if projected is None:
                    continue
                candidates=[]
                for observation in message.signals:
                    # One box may carry both HSV layout hypotheses. Only the
                    # map's layout participates; choose at most one per head.
                    if (not observation.valid or observation.layout != head['layout'] or
                            not math.isfinite(observation.confidence) or
                            observation.confidence < self.config['minimum_detection_confidence'] or
                            not observation.lamp_mask or observation.lamp_mask & ~15):
                        continue
                    box=np.asarray(observation.bbox)
                    if not np.isfinite(box).all() or np.any(box[2:]<=box[:2]):
                        continue
                    distance=np.linalg.norm((box[:2]+box[2:])/2-projected)
                    if distance <= message.image_width*self.config['maximum_projection_error_fraction']:
                        candidates.append((distance,observation))
                if candidates:
                    matches.append((head,min(candidates,key=lambda row:row[0])[1]))
        masks={observation.lamp_mask for _,observation in matches}
        common_mask=15
        for mask in masks:common_mask &= mask
        if masks and common_mask:
            output.lamp_mask=common_mask
            output.signal_ids=[head['id'] for head,_ in matches]
            output.confidence=min(observation.confidence for _,observation in matches)
            key=(target['stop_line_id'],output.lamp_mask)
            previous=self.confirmation
            if previous is None or previous[0]!=key or stamp-previous[2]>self.config['maximum_observation_age_sec']:
                self.confirmation=(key,stamp,stamp,1)
            else:
                self.confirmation=(key,previous[1],stamp,previous[3]+1)
            # Red/yellow is immediate. A release always requires multiple measurements.
            output.valid=(bool(output.lamp_mask & 3 and not output.lamp_mask & 12) or
                self.confirmation[3]>=self.config['green_confirmation_frames'] and
                stamp-self.confirmation[1]>=self.config['green_confirmation_sec'])
            output.reason='confirmed_hsv_map_association' if output.valid else 'confirming_green'
        else:
            self.confirmation=None
            output.reason='conflicting_heads' if masks else 'no_matching_visible_lamp'
        self.latest=output

    def states(self, now):
        target=self.target(now)
        if (target is None or self.latest is None or
                target['stop_line_id']!=self.latest.stop_line_id or
                not 0<=now-self.latest.header.stamp.to_sec()<=self.config['maximum_observation_age_sec']):
            return []
        return [self.latest]
