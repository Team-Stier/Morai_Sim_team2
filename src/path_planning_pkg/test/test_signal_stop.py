import unittest,math
import numpy as np,rospy
from common_msgs_pkg.msg import WorldModel,TrafficSignal,RouteContext
from path_planning_pkg.signal_stop import SignalStopPolicy
class StopTest(unittest.TestCase):
 def setUp(self):
  self.p=dict(reference_sha256='route',context_timeout_sec=.5,approach_distance_m=30.,targets=[dict(number=1,stop_line_id='bar',stop_station_m=100.,signal_ids=['head'],maneuver='straight')])
  self.c=dict(signal_stop_buffer_m=8.,signal_commit_distance_m=8.,signal_left_stop_buffer_m=14.,signal_left_commit_distance_m=12.,signal_deceleration_mps2=1.8,signal_observation_timeout_sec=.5)
  self.f=SignalStopPolicy(self.p,self.c);self.r=RouteContext(progress=80.,reference_sha256='route');self.r.header.stamp=rospy.Time(10)
  self.w=WorldModel();self.s=TrafficSignal(valid=True,lamp_mask=1,stop_line_id='bar',signal_ids=['head']);self.s.header.stamp=rospy.Time(10);self.w.traffic_signals=[self.s]
 def cap(self,now=10):return self.f.caps(np.array([80.,90.,92.,95.,100.]),self.r,self.w,now,0)
 def test_red_unknown_stale_stop_green_releases(self):
  cap=self.cap();self.assertGreater(cap[0],0);self.assertEqual(list(cap[2:]),[0,0,0])
  self.s.lamp_mask=4;self.assertTrue(np.isinf(self.cap()).all())
  self.assertTrue(np.isfinite(self.cap(11)).all())
  self.s.valid=False;self.assertTrue(np.isfinite(self.cap()).all())
 def test_unknown_simulator_bypass_keeps_known_stops_and_no_commit(self):
  self.c['signal_unknown_pass_enabled']=True
  for mask in (1,2):
   self.s.lamp_mask=mask;self.assertTrue(np.isfinite(self.cap()).all())
  self.s.valid=False;self.r.progress=94.
  self.assertTrue(np.isinf(self.cap()).all())
  self.assertIn('signal_action=PASS_UNKNOWN',self.f.reason)
  self.assertFalse(self.f.committed)
  self.s.valid=True;self.s.lamp_mask=1
  self.assertTrue(np.isfinite(self.cap()).all())
 def test_unknown_bypass_missing_stale_and_wrong_route(self):
  self.c['signal_unknown_pass_enabled']=True
  self.w.traffic_signals=[];self.assertTrue(np.isinf(self.cap()).all())
  self.w.traffic_signals=[self.s];self.s.header.stamp=rospy.Time(9)
  self.assertTrue(np.isinf(self.cap()).all())
  self.r.reference_sha256='wrong';self.assertTrue(np.isfinite(self.cap()).all())
  self.r.reference_sha256='route';self.r.header.stamp=rospy.Time(9)
  self.assertTrue(np.isfinite(self.cap()).all())
 def test_left_arrow_required_and_commit_only_on_fresh_permission(self):
  self.p['targets'][0]['maneuver']='left';self.s.lamp_mask=4;self.assertTrue(np.isfinite(self.cap()).all());self.assertEqual(self.cap()[1],0.)
  self.s.lamp_mask=9;self.assertTrue(np.isinf(self.cap()).all())
  self.r.progress=88.5;self.cap();self.s.valid=False;self.assertTrue(np.isinf(self.cap()).all())
  self.f.caps([80.],self.r,self.w,10.,1);self.assertTrue(np.isfinite(self.cap()).all())
 def test_new_lap_clears_commit_and_stale_route_cannot_release(self):
  self.s.lamp_mask=4;self.r.progress=93.;self.cap()
  self.assertIn('bar',self.f.committed)
  self.r.progress=30.;self.cap()
  self.assertFalse(self.f.committed)
  self.r.progress=80.;self.r.header.stamp=rospy.Time(9)
  self.assertTrue(np.isfinite(self.cap()).all())
if __name__=='__main__':unittest.main()
