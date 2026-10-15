from pathlib import Path
import unittest
import cv2,numpy as np,yaml
from camera_perception_pkg.signal_hsv import classify_lamps
ROOT=Path(__file__).resolve().parents[1]
class HSVTest(unittest.TestCase):
 def setUp(self):self.c=yaml.safe_load((ROOT/'config/signal_hsv.yaml').read_text())
 def read(self,layout,lamps):
  im=np.zeros((100,240,3),np.uint8);box=[20,20,220,80]
  for slot,color in lamps:cv2.circle(im,(int(20+200*(slot+.5)/layout),50),12,color,-1)
  return classify_lamps(im,box,layout,self.c)
 def test_color_slots_and_arrow_are_distinct(self):
  self.assertEqual(self.read(3,[(0,(0,0,255))])[:2],(1,True))
  self.assertEqual(self.read(3,[(1,(0,255,255))])[:2],(2,True))
  self.assertEqual(self.read(3,[(2,(0,255,0))])[:2],(4,True))
  self.assertEqual(self.read(4,[(2,(0,255,0))])[:2],(8,True))
  self.assertEqual(self.read(4,[(0,(0,0,255)),(2,(0,255,0))])[:2],(9,True))
 def test_dark_clipped_noise_and_conflict_are_unknown(self):
  self.assertFalse(self.read(4,[])[1])
  self.assertFalse(self.read(3,[(0,(0,0,255)),(2,(0,255,0))])[1])
  self.assertFalse(classify_lamps(np.zeros((100,240,3),np.uint8),[0,0,240,20],3,self.c)[1])
if __name__=='__main__':unittest.main()
