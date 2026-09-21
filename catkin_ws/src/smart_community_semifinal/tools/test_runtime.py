#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Meaningful runtime contracts, executable on stock Python 2.7 and Python 3."""
from __future__ import division,print_function
import io,json,math,os,sys,tempfile,unittest
import cv2,numpy as np
PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
from patrol_core import Patrol,CrossingPolicy
from image_geometry import planar_position,decode_image
from signal_locator import locate_signal
from traffic_detector_core import detect_signal
from scene_geometry import body_violation,physical_obstacles
from semifinal_core import StreetLedger,EvidenceWriter,front_clearance
from runtime_compat import monotonic,isfinite


def stop():
    return {'id':'one','point':[2.44,3.9],'direction':[-1,0],'wait_point':[2.78,3.9],
            'exit_point':[1.80,3.88],'min_green_seconds':15.}


def send(policy,state,t):policy.signal({'light_id':'one','state':state,'confidence':.95,'stamp':t},t)


class RuntimeContracts(unittest.TestCase):
    def test_green_without_observed_onset_never_releases(self):
        p=CrossingPolicy();p.arm(stop())
        for t in np.arange(1,3,.1):send(p,'green',float(t))
        self.assertFalse(p.ready(2.9))

    def test_witnessed_transition_and_stale_reset(self):
        p=CrossingPolicy();p.arm(stop());send(p,'red',1.)
        for t in [1.1,1.2,1.3]:send(p,'green',t)
        self.assertTrue(p.ready(1.3));self.assertFalse(p.ready(1.7))
        send(p,'green',2.);self.assertFalse(p.ready(2.))

    def test_late_green_and_wrong_light_cannot_authorize(self):
        p=CrossingPolicy();p.arm(stop());send(p,'red',1.)
        for t in np.arange(1.1,9.,.1):send(p,'green',float(t))
        self.assertFalse(p.ready(8.9))
        p.signal({'light_id':'other','state':'green','confidence':1.,'stamp':9.},9.)
        self.assertFalse(p.ready(9.))

    def test_replay_and_clock_reset(self):
        p=CrossingPolicy();p.arm(stop());send(p,'red',1.)
        for _ in range(10):send(p,'green',1.1)
        self.assertFalse(p.ready(1.1));send(p,'green',.1)
        self.assertFalse(p.ready(.1))

    def test_arm_is_idempotent_and_clear_requires_rear_exit(self):
        p=CrossingPolicy();p.arm(stop());send(p,'red',1.)
        for t in [1.1,1.2,1.3]:send(p,'green',t)
        p.arm(stop());self.assertTrue(p.ready(1.3))
        self.assertFalse(p.clear((2.78,3.9,math.pi)))
        self.assertTrue(p.clear((1.4,3.9,math.pi)))

    def test_pnp_metric_position_from_calibrated_pixels(self):
        K=np.array([[1108.,0,640],[0,1108.,480],[0,0,1]])
        w,h=.06,.145;obj=np.array([[-w/2,-h/2,0],[w/2,-h/2,0],[w/2,h/2,0],[-w/2,h/2,0]])
        expected=np.array([.12,.10,.65]);quad,_=cv2.projectPoints(obj,np.zeros(3),expected,K,np.zeros(5))
        p=planar_position({'width_m':w,'height_m':h,'quad':quad.reshape(4,2).tolist()},K)
        self.assertTrue(np.allclose(p['camera_xyz'],expected,atol=1e-5))

    def test_visual_housing_requires_pixels_and_rejects_blank(self):
        frame=np.full((300,700,3),230,np.uint8);cv2.rectangle(frame,(150,90),(470,160),(15,15,15),-1)
        cv2.circle(frame,(420,125),26,(0,255,0),-1)
        candidate=locate_signal(frame,[150,90,320,70]);self.assertIsNotNone(candidate)
        self.assertEqual(detect_signal(frame,candidate['roi'])['state'],'green')
        self.assertIsNone(locate_signal(np.zeros_like(frame),[150,90,320,70]))

    def test_pose_uses_observed_interior_points_despite_corner_extrapolation_error(self):
        rng=np.random.RandomState(21);K=np.array([[1108.,0,640],[0,1108.,480],[0,0,1]])
        objects=np.column_stack((rng.uniform(-.024,.024,48),rng.uniform(-.06,.06,48),np.zeros(48)))
        expected=np.array([.17,.14,.62]);pixels,_=cv2.projectPoints(objects,np.array([.03,.04,0]),expected,K,np.zeros(5))
        pixels=pixels.reshape(-1,2)+rng.normal(0,.6,(48,2))
        result=planar_position({'width_m':.055,'height_m':.145,'quad':[[0,0]]*4,
            'pose_correspondences':{'object':objects.tolist(),'image':pixels.tolist()}},K)
        self.assertIsNotNone(result);self.assertTrue(np.allclose(result['camera_xyz'],expected,atol=.008))

    def test_observation_cannot_finish_on_unconfirmed_detections(self):
        p=Patrol([{'name':'view','xy':[0,0],'yaw':0,'observe':True}]);p.step((0,0,0),0)
        for t in [1.,2.,3.,4.]:p.record_frame('view',t,3,False);p.step((0,0,0),t)
        self.assertEqual(p.phase,'observe')
        for t in [5.,6.,7.]:p.record_frame('view',t,3,True);p.step((0,0,0),t)
        self.assertEqual(p.phase,'done')

    def test_persistent_blocked_motion_fails_explicitly(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0);v,w=p.step((0,0,0),26)
        self.assertEqual(p.phase,'failed');self.assertEqual(p.error,'motion_stalled:goal')
        self.assertEqual((v,w),(0.,0.))

    def test_image_padding_and_rgb_order(self):
        class Msg:pass
        msg=Msg();msg.encoding='rgb8';msg.height=1;msg.width=2;msg.step=8
        msg.data=np.array([255,0,0,0,255,0,77,88],np.uint8).tobytes()
        self.assertEqual(decode_image(msg).tolist(),[[[0,0,255],[0,255,0]]])

    def test_ledger_counts_same_art_at_different_positions(self):
        ledger=StreetLedger()
        for frame in range(3):
            for x in [1.,1.2]:ledger.observe('A','resident_1','resident',(x,1),frame)
        self.assertEqual(ledger.summary('A')['total'],2)
        ledger.observe('A','resident_1','resident',(1.,1),2)
        self.assertEqual(ledger.summary('A')['total'],2)

    def test_evidence_failure_and_unicode(self):
        folder=tempfile.mkdtemp(prefix='semifinal_contract_')
        writer=EvidenceWriter(folder,lambda path,im:False)
        with self.assertRaises(IOError):writer.record(1,1.,[],None)
        self.assertFalse(os.path.exists(writer.path));self.assertEqual(writer.sequence,0)
        self.assertTrue(monotonic()>0);self.assertFalse(isfinite(float('nan')))

    def test_complete_ordered_route_on_kinematic_plant(self):
        # Simulator lamp state exists ONLY in this test fixture.
        with io.open(os.path.join(PKG,'config/layout.json'),encoding='utf-8') as stream:layout=json.load(stream)
        obstacles=physical_obstacles(os.path.join(PKG,'worlds/official_semifinal.world'))
        core=Patrol(layout['route']);policy=CrossingPolicy();policy.clear()
        pose=[3.78,3.9,math.pi];armed=None;visited=set();dt=.05
        for step in range(14000):
            t=step*dt;target=core.target;visited.add(target['name'])
            if target.get('gate') and armed!=target['gate']:
                line=next(s for s in layout['stop_lines'] if s['id']==target['gate'])
                lamp=next(s for s in layout['lights'] if s['id']==target['gate'])
                policy.arm(dict(line,wait_point=target['xy'],exit_point=lamp['xy'],min_green_seconds=15.));armed=target['gate']
            phase=(t+(7 if armed=='light_2' else 0))%28
            state='red' if phase<10 else 'green' if phase<25 else 'yellow'
            if policy.stop:policy.signal({'light_id':armed,'state':state,'confidence':1.,'stamp':t},t)
            if core.phase=='observe':core.record_frame(target['name'],t,1)
            v,w=core.step(pose,t,policy.ready(t));v,w=policy.filter(pose,v,w,t)
            pose[0]+=v*math.cos(pose[2])*dt;pose[1]+=v*math.sin(pose[2])*dt;pose[2]+=w*dt
            self.assertIsNone(body_violation(pose,layout,obstacles),'%s %s'%(target['name'],pose))
            self.assertIsNone(policy.failure)
            if core.phase=='done':break
        self.assertEqual(core.phase,'done')
        self.assertEqual(len(visited),len(layout['route']))


if __name__=='__main__':unittest.main(verbosity=2)
