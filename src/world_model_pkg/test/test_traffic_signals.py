import unittest
import rospy
from common_msgs_pkg.msg import EgoState,RouteContext,CameraSignalArray,TrafficSignal
from world_model_pkg.traffic_signals import SignalFusion
class FusionTest(unittest.TestCase):
 def setUp(self):
  self.policy=dict(reference_sha256='route',context_timeout_sec=.5,approach_distance_m=30.,camera_projection=dict(translation_m=[0,0,0],pitch_rad=0.,horizontal_fov_deg=90),targets=[dict(number=1,stop_line_id='bar',stop_station_m=100.,signal_ids=['head'],heads=[dict(id='head',map_position=[20,0,5],layout=3)])])
  self.f=SignalFusion(self.policy,dict(maximum_pose_delta_sec=.15,maximum_observation_age_sec=.5,minimum_detection_confidence=.35,maximum_projection_error_fraction=.07,green_confirmation_frames=3,green_confirmation_sec=.2))
 def sample(self,t,mask=4,box=None,signals=None):
  p=EgoState();p.header.stamp=rospy.Time.from_sec(t);p.pose.pose.orientation.w=1;p.pose_valid=[True]*6;self.f.add_pose(p)
  r=RouteContext(progress=80,reference_sha256='route');r.header.stamp=p.header.stamp;r.header.frame_id='map';self.f.route=r
  m=CameraSignalArray(image_width=1000,image_height=750);m.header.stamp=p.header.stamp;m.header.frame_id='camera_signal_optical_frame'
  # Pinhole projection: (500,250).
  m.signals=signals if signals is not None else [TrafficSignal(valid=True,layout=3,confidence=.9,lamp_mask=mask,bbox=box or [475,240,525,260])]
  self.f.ingest(m,t);return self.f.states(t)[0]
 def test_map_association_and_green_temporal_confirmation(self):
  self.assertFalse(self.sample(10).valid);self.assertFalse(self.sample(10.1).valid)
  s=self.sample(10.3);self.assertTrue(s.valid);self.assertEqual(s.signal_ids,['head']);self.assertEqual(s.header.stamp,rospy.Time.from_sec(10.3))
  self.assertEqual(self.f.states(11),[])
 def test_red_immediate_wrong_roi_unknown_reset_clears(self):
  self.assertTrue(self.sample(10,1).valid)
  self.assertFalse(self.sample(10.1,4,[100,100,120,120]).valid)
  p=EgoState(reset_id=1);p.header.stamp=rospy.Time(11);self.f.add_pose(p);self.assertIsNone(self.f.latest)
 def test_duplicate_and_gapped_frames_cannot_confirm_permission(self):
  self.sample(10);self.sample(10);self.sample(10)
  self.assertEqual(self.f.confirmation[3],1)
  self.assertFalse(self.sample(10.6).valid)
  self.assertFalse(self.sample(10.7).valid)
  self.assertTrue(self.sample(10.9).valid)
 def hypotheses(self,slot,layout=3,box=None):
  import cv2,numpy as np,yaml
  from pathlib import Path
  from camera_perception_pkg.signal_hsv import classify_layout_hypotheses
  config=yaml.safe_load((Path(__file__).resolve().parents[2]/
      'camera_perception_pkg/config/signal_hsv.yaml').read_text())
  image=np.zeros((750,1000,3),np.uint8);box=box or [440,230,560,270]
  x=int(box[0]+(box[2]-box[0])*(slot+.5)/layout)
  cv2.circle(image,(x,250),4,(0,0,255) if slot==0 else (0,255,0),-1)
  return [TrafficSignal(confidence=.9,bbox=box,**h) for h in
          classify_layout_hypotheses(image,box,config)]
 def test_three_lamp_map_accepts_red_despite_four_lamp_yolo_class(self):
  # The producer no longer gates the 3-lamp hypothesis on YOLO class_id.
  state=self.sample(10,signals=self.hypotheses(0))
  self.assertTrue(state.valid);self.assertEqual(state.lamp_mask,1)
  self.assertEqual(state.signal_ids,['head'])
  wrong=self.sample(10.1,signals=self.hypotheses(0,box=[100,230,220,270]))
  self.assertFalse(wrong.valid)
 def test_alternative_layouts_do_not_count_as_extra_confirmation_frames(self):
  signals=self.hypotheses(2)
  self.assertFalse(self.sample(10,signals=signals).valid)
  self.assertEqual(self.f.confirmation[3],1)
  self.assertFalse(self.sample(10.1,signals=signals).valid)
  self.assertTrue(self.sample(10.3,signals=signals).valid)
  self.assertEqual(self.f.latest.lamp_mask,4)
 def test_four_lamp_arrow_cannot_release_three_lamp_map_target(self):
  signals=self.hypotheses(2,layout=4)
  self.assertFalse(self.sample(10,signals=signals).valid)
  self.assertFalse(self.sample(10.3,signals=signals).valid)
  self.policy['targets'][0]['heads'][0]['layout']=4
  self.assertFalse(self.sample(10.4,signals=signals).valid)
  self.assertFalse(self.sample(10.5,signals=signals).valid)
  state=self.sample(10.7,signals=signals)
  self.assertTrue(state.valid);self.assertEqual(state.lamp_mask,8)
 def test_heads_must_share_a_permissive_lamp(self):
  self.policy['targets'][0]['heads'].append(dict(id='other',map_position=[20,-2,5],layout=3))
  def pair(t,first,second):
   self.sample(t,first)
   m=CameraSignalArray(image_width=1000,image_height=750)
   m.header.stamp=rospy.Time.from_sec(t+.01);m.header.frame_id='camera_signal_optical_frame'
   m.signals=[TrafficSignal(valid=True,layout=3,confidence=.9,lamp_mask=first,bbox=[475,240,525,260]),
              TrafficSignal(valid=True,layout=3,confidence=.9,lamp_mask=second,bbox=[525,240,575,260])]
   self.f.ingest(m,t+.01);return self.f.states(t+.01)[0]
  self.assertFalse(pair(10,1,4).valid)
  pair(10.1,4,12);pair(10.2,4,12)
  state=pair(10.4,4,12)
  self.assertTrue(state.valid);self.assertEqual(state.lamp_mask,4)
  self.assertEqual(state.signal_ids,['head','other'])
if __name__=='__main__':unittest.main()
