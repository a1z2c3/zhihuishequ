#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Meaningful runtime contracts, executable on stock Python 2.7 and Python 3."""
from __future__ import division,print_function
import io,json,math,os,shutil,subprocess,sys,tempfile,unittest
import xml.etree.ElementTree as ET
import cv2,numpy as np
PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
import patrol_core
from patrol_core import Patrol,CrossingPolicy
from image_geometry import planar_position,decode_image
from signal_locator import locate_signal
from traffic_detector_core import detect_signal
from scene_geometry import (body_violation,physical_obstacles,camera_from_urdf,
                            visible_card)
from semifinal_core import (StreetLedger,EvidenceWriter,front_clearance,body_over_stop_line,scan_clearance,
                            integrate_twist_pose)
from runtime_compat import monotonic,isfinite


def stop():
    return {'id':'light_1','point':[2.44,3.9],'direction':[-1,0],'wait_point':[2.78,3.9],
            'exit_point':[1.80,3.88],'min_green_seconds':15.}


def send(policy,state,t):policy.signal({'light_id':'light_1','state':state,'confidence':.95,'stamp':t},t)


class RuntimeContracts(unittest.TestCase):
    def test_red_body_overlap_requires_rear_to_clear_stop_line(self):
        line=stop()
        self.assertFalse(body_over_stop_line((2.78,3.9,math.pi),line['point'],line['direction']))
        self.assertTrue(body_over_stop_line((2.44,3.9,math.pi),line['point'],line['direction']))
        self.assertFalse(body_over_stop_line((2.10,3.9,math.pi),line['point'],line['direction']))

    def test_constant_twist_integration_handles_curved_sweep(self):
        pose=integrate_twist_pose((0.,0.,0.),.2,0.,1.,1.)
        self.assertAlmostEqual(pose[0],.2*math.sin(1.),places=6)
        self.assertAlmostEqual(pose[1],.2*(1.-math.cos(1.)),places=6)
        self.assertAlmostEqual(pose[2],1.,places=6)

    def test_first_direct_green_never_fabricates_an_onset(self):
        p=CrossingPolicy();p.arm(stop())
        for t in np.arange(1.,20.,.1):send(p,'green',float(t))
        self.assertIsNone(p.green_start)
        self.assertFalse(p.ready(19.9))
        # Only a witnessed red/yellow baseline followed by green can open
        # the clock used by the remaining-time check.
        send(p,'red',20.0);send(p,'green',20.1);send(p,'green',20.2)
        self.assertFalse(p.ready(20.2))
        send(p,'green',20.3)
        self.assertTrue(p.ready(20.3))

    def test_verified_anchor_survives_unknown_or_long_frame_gap(self):
        p=CrossingPolicy();p.arm(stop());send(p,'red',1.0)
        for t in [1.1,1.2,1.3]:send(p,'green',t)
        self.assertAlmostEqual(p.green_start,1.0)
        # An explicit unknown frame resets quorum, not the verified onset.
        p.signal({'light_id':'light_1','state':'unknown','confidence':0.,'stamp':1.4},1.4)
        self.assertAlmostEqual(p.green_start,1.0)
        for t in [1.5,1.6,1.7]:send(p,'green',t)
        self.assertTrue(p.ready(1.7))
        p.gate.reset();p.previous=None
        send(p,'green',2.2)
        self.assertAlmostEqual(p.green_start,1.0)

    def test_signal_process_waits_for_exact_camera_transform(self):
        source=os.path.join(PKG,'scripts','signal_perception_node.py')
        with io.open(source,encoding='utf-8') as stream:
            text=stream.read()
        self.assertIn("waitForTransform('map','base_footprint'",text)
        self.assertIn("rospy.Duration(.05)",text)

    def test_object_perception_waits_before_exact_timestamp_tf_queries(self):
        source=os.path.join(PKG,'scripts','official_perception_node.py')
        with io.open(source,encoding='utf-8') as stream:
            text=stream.read()
        self.assertEqual(text.count("waitForTransform('map',msg.header.frame_id"),2)
        self.assertEqual(text.count("lookupTransform('map',msg.header.frame_id"),2)
        self.assertIn("rospy.Duration(.05)",text)

    def test_locked_phase_clock_recovers_direct_green_safely(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop());send=lambda state,t:p.signal(
            {'light_id':'light_1','state':state,'confidence':.95,'stamp':t},t)
        for t in np.arange(1.0,6.01,.1):send('red',float(t))
        for t in [6.1,6.2,6.3]:send('green',t)
        self.assertIsNotNone(p.clock_origin)
        p.clear();p.arm(stop())
        # At t=7.0 this is still the same green epoch whose onset was 6.0.
        for t in [7.0,7.1,7.2]:send('green',t)
        self.assertAlmostEqual(p.green_start,6.0,places=5)
        self.assertTrue(p.ready(7.2))
        p.clear();p.arm(stop())
        # Near the end of that green, remaining time is insufficient; the
        # clock must not turn a late green into a full 15-second allowance.
        for t in [20.0,20.1,20.2]:send('green',t)
        self.assertAlmostEqual(p.green_start,6.0,places=5)
        self.assertFalse(p.ready(20.2))

    def test_locked_phase_clock_applies_light_two_offset_once(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(dict(stop(),id='light_2'))
        def emit(state,t):
            p.signal({'light_id':'light_2','state':state,'confidence':.95,'stamp':t},t)
        # For light_2, t=3 is the shared cycle's green onset because
        # (t + 7) reaches the configured red_s boundary at phase 10.
        for t in np.arange(-2.1,2.91,.1):emit('red',float(t))
        emit('green',3.0);emit('green',3.1);emit('green',3.2)
        self.assertAlmostEqual(p.green_start,2.9,places=5)
        self.assertTrue(p.ready(3.2))
        p.clear();p.arm(dict(stop(),id='light_2'))
        emit('green',16.0);emit('green',16.1);emit('green',16.2)
        # The verified red frame at 2.9 is the conservative onset anchor.
        self.assertAlmostEqual(p.green_start,2.9,places=5)
        self.assertFalse(p.ready(16.2))

    def test_short_red_episode_cannot_anchor_green(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop())
        emit=lambda state,t:p.signal({'light_id':'light_1','state':state,
                                      'confidence':.95,'stamp':t},t)
        emit('red',1.0);emit('green',1.1);emit('green',1.2);emit('green',1.3)
        self.assertIsNone(p.green_start)
        self.assertFalse(p.ready(1.3))

    def test_red_anchor_requires_half_configured_red_duration(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop())
        emit=lambda state,t:p.signal({'light_id':'light_1','state':state,
                                      'confidence':.95,'stamp':t},t)
        # A continuous five-second red observation is the configured lower
        # bound; the following green transition is accepted.
        for t in np.arange(1.0,6.01,.1):emit('red',float(t))
        emit('green',6.1);emit('green',6.2);emit('green',6.3)
        self.assertAlmostEqual(p.green_start,6.0,places=5)
        self.assertTrue(p.ready(6.3))

    def test_red_anchor_threshold_is_stable_at_float_boundary(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop())
        emit=lambda state,t:p.signal({'light_id':'light_1','state':state,
                                      'confidence':.95,'stamp':t},t)
        for t in np.arange(1.0,6.01,.1):emit('red',float(t))
        emit('green',6.1);emit('green',6.2);emit('green',6.3)
        self.assertIsNotNone(p.clock_origin)
        self.assertAlmostEqual(p.green_start,6.0,places=5)

    def test_red_anchor_tolerates_one_slow_render_gap(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop())
        emit=lambda state,t:p.signal({'light_id':'light_1','state':state,
                                      'confidence':.95,'stamp':t},t)
        # The red evidence window is intentionally looser than the green
        # quorum: one 0.5 s rendering gap must not erase a real red episode.
        for t in [1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0]:
            emit('red',t)
        emit('green',6.1);emit('green',6.2);emit('green',6.3)
        self.assertAlmostEqual(p.green_start,6.0,places=5)

    def test_unverified_red_gap_cannot_fall_back_to_phase_clock(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop())
        emit=lambda state,t:p.signal({'light_id':'light_1','state':state,
                                      'confidence':.95,'stamp':t},t)
        for t in np.arange(1.0,6.01,.1):emit('red',float(t))
        emit('green',6.1);emit('green',6.2);emit('green',6.3)
        self.assertIsNotNone(p.clock_origin)
        p.clear();p.arm(stop())
        emit('green',7.0);emit('green',7.1);emit('green',7.2)
        emit('red',7.3)
        # A gap beyond the red evidence window cannot turn this unverified
        # red frame into a fresh clock anchor.
        emit('green',8.5);emit('green',8.6);emit('green',8.7)
        self.assertIsNone(p.green_start)
        self.assertFalse(p.ready(8.7))

    def test_false_red_after_locked_clock_cannot_reopen_green(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,'light_2_offset_s':7.}
        p=CrossingPolicy(cycle);p.arm(stop())
        emit=lambda state,t:p.signal({'light_id':'light_1','state':state,
                                      'confidence':.95,'stamp':t},t)
        for t in np.arange(1.0,6.01,.1):emit('red',float(t))
        emit('green',6.1);emit('green',6.2);emit('green',6.3)
        self.assertIsNotNone(p.clock_origin)
        p.clear();p.arm(stop())
        # The clock can infer an existing green epoch, but a one-frame false
        # red must still not create a new witnessed onset.
        emit('green',7.0);emit('green',7.1);emit('green',7.2)
        emit('red',7.3)
        emit('green',7.4);emit('green',7.5);emit('green',7.6)
        self.assertIsNone(p.green_start)
        self.assertFalse(p.ready(7.6))

    def test_unknown_or_frame_gap_requires_verified_green_onset(self):
        p=CrossingPolicy();p.arm(stop())
        send(p,'green',1.0);send(p,'green',1.1);send(p,'green',1.2)
        self.assertFalse(p.ready(1.2))
        # Unknown/stale perception revokes permission; green-only recovery is
        # deliberately unverified and must not fabricate an onset.
        p.signal({'light_id':'light_1','state':'unknown','confidence':0.,'stamp':1.3},1.3)
        self.assertFalse(p.ready(1.3))
        send(p,'green',1.4);send(p,'green',1.5);send(p,'green',1.6)
        self.assertFalse(p.ready(1.6))
        # A green-only stream after a gap remains unverified; a dropped frame
        # must never turn an unknown late-green phase into permission.
        p.signal({'light_id':'light_1','state':'unknown','confidence':0.,'stamp':1.7},1.7)
        for t in (2.1,2.2,2.3):send(p,'green',t)
        self.assertFalse(p.ready(2.3))
        send(p,'red',2.4);send(p,'green',2.5);send(p,'green',2.6);send(p,'green',2.7)
        self.assertTrue(p.ready(2.7))

    def test_red_and_yellow_immediately_revoke_green_permission(self):
        p=CrossingPolicy();p.arm(stop())
        send(p,'red',1.0)
        for t in [1.1,1.2,1.3]:send(p,'green',t)
        self.assertTrue(p.ready(1.3))
        send(p,'yellow',1.4);self.assertFalse(p.ready(1.4))
        send(p,'red',1.5);self.assertFalse(p.ready(1.5))

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

    def test_new_stop_cannot_replace_an_active_line(self):
        p=CrossingPolicy();p.arm(stop())
        with self.assertRaises(ValueError):p.arm(dict(stop(),id='light_2'))
        self.assertEqual(p.stop['id'],'light_1')
        self.assertEqual(p.mode,'approach')
        self.assertTrue(p.clear((1.4,3.9,math.pi)))
        p.arm(dict(stop(),id='light_2'))
        self.assertEqual(p.stop['id'],'light_2')

    def test_signal_rename_requires_explicit_cycle_offset(self):
        cycle={'period_s':28.,'red_s':10.,'green_s':15.,
               'light_2_offset_s':7.}
        renamed=dict(stop(),id='junction_north')
        p=CrossingPolicy(cycle)
        with self.assertRaises(ValueError):p.arm(renamed)
        p=CrossingPolicy(dict(cycle,offsets_s={'junction_north':0.}))
        p.arm(renamed)
        for t in np.arange(1.,6.01,.1):
            p.signal({'light_id':'junction_north','state':'red',
                      'confidence':.95,'stamp':float(t)},float(t))
        for t in (6.1,6.2,6.3):
            p.signal({'light_id':'junction_north','state':'green',
                      'confidence':.95,'stamp':t},t)
        self.assertTrue(p.ready(6.3))

    def test_failure_reason_clears_only_after_valid_stop_transition(self):
        p=CrossingPolicy();p.arm(stop());p.failure='stop_line_crossed_before_permission'
        p.arm(stop())
        self.assertIsNotNone(p.failure)
        self.assertFalse(p.clear((2.78,3.9,math.pi)))
        self.assertIsNotNone(p.failure)
        self.assertTrue(p.clear((1.4,3.9,math.pi)))
        self.assertIsNone(p.failure)
        p.arm(stop());self.assertIsNone(p.failure)

    def test_red_gate_blocks_holonomic_line_crossing(self):
        p=CrossingPolicy();p.arm(stop());send(p,'red',1.)
        speed,lateral,omega=p.filter_command((2.62,3.9,math.pi/2),0.,.12,0.,1.1)
        self.assertEqual((speed,lateral,omega),(0.,0.,0.))
        self.assertIsNone(p.failure)

    def test_crossing_failure_does_not_leak_into_next_gate(self):
        p=CrossingPolicy();p.arm(stop())
        p.filter_command((2.3,3.9,math.pi),0.,0.,0.,1.0)
        self.assertEqual(p.failure,'stop_line_crossed_before_permission')
        self.assertTrue(p.clear((1.4,3.9,math.pi)))
        p.arm(stop())
        self.assertIsNone(p.failure)

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

    def test_low_saturation_yellow_led_is_located_and_classified(self):
        frame=np.full((300,700,3),230,np.uint8)
        cv2.rectangle(frame,(150,90),(470,160),(15,15,15),-1)
        hsv=np.zeros((300,700,3),np.uint8);hsv[:,:,0]=20;hsv[:,:,1]=80;hsv[:,:,2]=230
        circle=np.zeros((300,700),np.uint8);cv2.circle(circle,(420,125),26,255,-1)
        bgr=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR);frame[circle>0]=bgr[circle>0]
        candidate=locate_signal(frame,[150,90,320,70])
        self.assertIsNotNone(candidate)
        self.assertEqual(candidate['colour_proposal'],'yellow')
        self.assertEqual(detect_signal(frame,candidate['roi'])['state'],'yellow')

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

    def test_observation_requires_current_context_and_committed_result(self):
        p=Patrol([{'name':'view','xy':[0,0],'yaw':0,'observe':True}]);p.step((0,0,0),0)
        context=p.observation_id
        # A detector hit without PnP/TF/region commitment cannot advance the
        # frame quorum, even when its raw detection count is non-zero.
        for t in [1.,2.,3.]:
            p.record_frame('view',t,4,True,committed_count=0,context_id=context)
        self.assertEqual(len(p.frames),0);self.assertEqual(p.phase,'observe')
        # Delayed evidence from the preceding context is ignored as well.
        for t in [4.,5.,6.]:
            p.record_frame('view',t,4,True,committed_count=1,context_id='old:view')
        self.assertEqual(len(p.frames),0)
        for t in [7.,8.,9.]:
            p.record_frame('view',t,4,True,committed_count=1,context_id=context)
            p.step((0,0,0),t)
        self.assertEqual(p.phase,'done')

    def test_persistent_blocked_motion_fails_explicitly(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0);v,vy,w=p.step((0,0,0),41)
        self.assertEqual(p.phase,'failed');self.assertEqual(p.error,'motion_stalled:goal')
        self.assertEqual((v,vy,w),(0.,0.,0.))

    def test_gate_wait_has_bounded_failure(self):
        p=Patrol([{'name':'gate','xy':[0,0],'yaw':0,'gate':'light_1'}])
        p.step((0,0,0),0.)
        self.assertEqual(p.phase,'gate')
        p.step((0,0,0),90.1,entry_ready=False)
        self.assertEqual(p.phase,'failed')
        self.assertEqual(p.error,'gate_timeout:gate')

    def test_gate_timeout_pauses_during_safety_hold(self):
        p=Patrol([{'name':'gate','xy':[0,0],'yaw':0,'gate':'light_1'}])
        p.step((0,0,0),0.)
        p.step((0,0,0),1.,guard={'safety_hold':True})
        p.step((0,0,0),30.,guard={'safety_hold':True})
        p.step((0,0,0),30.1,entry_ready=False,guard={})
        self.assertEqual(p.phase,'gate')

    def test_wall_watchdogs_have_wall_driven_ros_loops(self):
        for name in ('patrol_node.py','traffic_guard_node.py'):
            with io.open(os.path.join(PKG,'scripts',name),encoding='utf-8') as stream:
                source=stream.read()
            self.assertNotIn('rospy.Rate(',source)
            self.assertIn('time.sleep(',source)

    def test_lateral_avoidance_locks_until_rear_clear_then_recenters(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        guard={'forward_obstacle':True,'obstacle_ahead':True,
               'left_free':True,'right_free':False}
        for t in [1.,1.1,1.2]:
            v,vy,w=p.step((0,0,0),t,guard=guard)
        self.assertEqual(p.phase,'avoid');self.assertEqual((v,vy,w),(0.,.08,0.))
        self.assertEqual(p.step((0,.09,0),2.0,guard=guard)[0],.08)
        self.assertEqual(p.phase,'avoid')
        v,vy,w=p.step((0,.09,0),3.0,guard={'obstacle_ahead':False,
                                            'recenter_clear':True,
                                            'left_free':True,'right_free':True})
        self.assertEqual(p.phase,'recenter');self.assertLess(vy,0.)
        v,vy,w=p.step((0,.01,0),4.0,guard={'recenter_clear':True})
        self.assertEqual(p.phase,'travel');self.assertGreater(v,0.)

    def test_obstacle_with_both_sides_blocked_stays_stopped(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        guard={'forward_obstacle':True,'left_free':False,'right_free':False}
        for t in [1.,1.1,1.2,1.3]:
            p.step((0,0,0),t,guard=guard)
        self.assertEqual(p.phase,'travel')

    def test_permanent_center_block_has_bounded_failure(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        guard={'forward_obstacle':True,'obstacle_ahead':True,
               'left_free':False,'right_free':False}
        p.step((0,0,0),0,guard=guard)
        p.step((0,0,0),90.1,guard=guard)
        self.assertEqual(p.phase,'failed')
        self.assertEqual(p.error,'obstacle_blocked_timeout:goal')

    def test_avoid_timeout_does_not_retrigger_same_obstacle(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        guard={'forward_obstacle':True,'obstacle_ahead':True,
               'left_free':True,'right_free':False,
               'left_obstacle_ahead':True,'right_obstacle_ahead':True,
               'recenter_clear':False,'obstacle_rear_x':.5}
        for t in [1.,1.1,1.2]:p.step((0,0,0),t,guard=guard)
        self.assertEqual(p.phase,'avoid')
        p.step((0,0,0),20.,guard=guard)
        self.assertEqual(p.phase,'travel');self.assertTrue(p.avoid_failed)
        p.step((0,0,0),21.,guard=guard)
        self.assertEqual(p.phase,'travel');self.assertTrue(p.avoid_failed)

    def test_centered_obstacle_never_selects_lateral_avoidance(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        guard={'forward_obstacle':True,'left_free':False,'right_free':False}
        outputs=[p.step((0,0,0),t,guard=guard) for t in [1.,1.1,1.2,1.3]]
        self.assertEqual(p.phase,'travel')
        self.assertTrue(all(v==0. for v,vy,w in outputs[-2:]))
        self.assertTrue(all(vy==0. for v,vy,w in outputs[-2:]))

    def test_lateral_command_is_body_frame_and_not_yaw(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        guard={'forward_obstacle':True,'left_free':True,'right_free':False}
        for t in [1.,1.1,1.2]:
            result=p.step((0,0,0),t,guard=guard)
        vx,vy,wz=result
        self.assertEqual(vx,0.)
        self.assertEqual(vy,.08)
        self.assertEqual(wz,0.)

    def test_stale_guard_is_a_hold_not_a_clear_scene(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        result=p.step((0,0,0),100.,guard={'safety_hold':True})
        self.assertEqual(result,(0.,0.,0.));self.assertEqual(p.phase,'travel')
        self.assertIsNone(p.error)

    def test_safety_hold_timeout_latches_task_failure_and_zero_command(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.safety_hold_timeout=2.0
        p.step((0,0,0),0.,guard={'safety_hold':True})
        result=p.step((0,0,0),2.1,guard={'safety_hold':True})
        self.assertEqual(result,(0.,0.,0.))
        self.assertEqual(p.phase,'failed')
        self.assertEqual(p.error,'safety_hold_timeout:goal')
        self.assertEqual(p.step((0,0,0),2.2,guard={}),(0.,0.,0.))

    def test_safety_hold_wall_timeout_when_simulation_clock_stops(self):
        original=patrol_core.monotonic;clock=[100.]
        patrol_core.monotonic=lambda:clock[0]
        try:
            p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
            p.safety_hold_wall_timeout=3.
            p.step((0,0,0),0.,guard={'safety_hold':True})
            clock[0]=102.9
            self.assertEqual(p.step((0,0,0),0.,guard={'safety_hold':True}),
                             (0.,0.,0.))
            self.assertEqual(p.phase,'travel')
            clock[0]=103.1
            self.assertEqual(p.step((0,0,0),0.,guard={'safety_hold':True}),
                             (0.,0.,0.))
            self.assertEqual(p.phase,'failed')
            self.assertEqual(p.error,'safety_hold_timeout:goal')
        finally:
            patrol_core.monotonic=original

    def test_gate_wait_pauses_both_clocks_during_temporary_hold(self):
        original=patrol_core.monotonic;clock=[100.]
        patrol_core.monotonic=lambda:clock[0]
        try:
            p=Patrol([{'name':'gate','xy':[0,0],'yaw':0,'gate':'light_1'}])
            p.gate_timeout=10.
            p.step((0,0,0),0.)
            clock[0]=101.;p.step((0,0,0),1.,guard={'safety_hold':True})
            clock[0]=105.;p.step((0,0,0),5.,guard={'safety_hold':True})
            clock[0]=106.;p.step((0,0,0),6.,guard={})
            self.assertEqual(p.phase,'gate')
            self.assertAlmostEqual(p.entered,5.)
            self.assertAlmostEqual(p.entered_wall,105.)
            clock[0]=107.;p.step((0,0,0),7.,entry_ready=True)
            self.assertEqual(p.phase,'done')
        finally:
            patrol_core.monotonic=original

    def test_gate_wall_timeout_when_simulation_clock_stops(self):
        original=patrol_core.monotonic;clock=[100.]
        patrol_core.monotonic=lambda:clock[0]
        try:
            p=Patrol([{'name':'gate','xy':[0,0],'yaw':0,'gate':'light_1'}])
            p.step((0,0,0),0.)
            clock[0]=701.
            self.assertEqual(p.step((0,0,0),0.,entry_ready=False),(0.,0.,0.))
            self.assertEqual(p.phase,'failed')
            self.assertEqual(p.error,'gate_timeout:gate')
        finally:
            patrol_core.monotonic=original

    def test_observation_wall_timeout_when_simulation_clock_stops(self):
        original=patrol_core.monotonic;clock=[100.]
        patrol_core.monotonic=lambda:clock[0]
        try:
            p=Patrol([{'name':'view','xy':[0,0],'yaw':0,'observe':True}])
            p.step((0,0,0),0.)
            clock[0]=281.
            self.assertEqual(p.step((0,0,0),0.),(0.,0.,0.))
            self.assertEqual(p.phase,'failed')
            self.assertEqual(p.error,'observation_timeout:view')
        finally:
            patrol_core.monotonic=original

    def test_stale_hold_pauses_avoid_timeout(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        guard={'forward_obstacle':True,'obstacle_ahead':True,
               'left_free':True,'right_free':False}
        for t in [1.,1.1,1.2]:p.step((0,0,0),t,guard=guard)
        self.assertEqual(p.phase,'avoid')
        p.step((0,.03,0),2.,guard={'safety_hold':True})
        p.step((0,.03,0),30.,guard={'safety_hold':True})
        p.step((0,.09,0),30.1,guard={'obstacle_ahead':False,
                                      'recenter_clear':True,
                                      'left_free':True,'right_free':True})
        self.assertNotEqual(p.phase,'failed')
        self.assertNotEqual(p.error,'avoid_timeout:goal')

    def test_lane_recovery_is_bounded_and_moves_inward(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        guard={'lane_recovery':True,'lane_recovery_lateral':1.0}
        vx,vy,w=p.step((0,0,0),1.,guard=guard)
        self.assertEqual((vx,vy,w),(0.,.04,0.))
        p.step((0,0,0),31.1,guard=guard)
        self.assertEqual(p.phase,'failed')
        self.assertEqual(p.error,'lane_recovery_timeout:goal')

    def test_lane_recovery_direction_is_not_invented(self):
        p=Patrol([{'name':'goal','xy':[1,0],'yaw':0}])
        p.step((0,0,0),0)
        self.assertEqual(p.step((0,0,0),1.,guard={'lane_recovery':True}),
                         (0.,0.,0.))

    def test_reference_detector_category_filter_is_safe(self):
        import os,sys
        import numpy as np
        pkg=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sys.path.insert(0,os.path.join(pkg,'scripts'))
        from reference_detector import ReferenceDetector
        detector=ReferenceDetector(os.path.join(pkg,'assets','manifest.json'))
        frame=np.zeros((64,64,3),np.uint8)
        self.assertEqual(detector.detect(frame,categories=('plate',)),[])

    def test_scan_clearance_exposes_recenter_sweep_state(self):
        class Scan:pass
        scan=Scan();scan.ranges=[.22,.22]
        scan.angle_min=0.;scan.angle_increment=math.pi
        scan.range_min=.02;scan.range_max=8.
        result=scan_clearance(scan,.18)
        self.assertTrue(result['forward_obstacle'])
        self.assertFalse(result['recenter_clear'])

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
        cycle=layout.get('signal_cycle',{'period_s':28.0,'red_s':10.0,'green_s':15.0,
                                         'yellow_s':3.0,'light_2_offset_s':7.0})
        core=Patrol(layout['route']);policy=CrossingPolicy(cycle);policy.clear()
        pose=[3.78,3.9,math.pi];armed=None;visited=set();dt=.05
        for step in range(14000):
            t=step*dt;target=core.target;visited.add(target['name'])
            if target.get('gate') and armed!=target['gate']:
                line=next(s for s in layout['stop_lines'] if s['id']==target['gate'])
                lamp=next(s for s in layout['lights'] if s['id']==target['gate'])
                policy.arm(dict(line,wait_point=target['xy'],exit_point=lamp['xy'],min_green_seconds=15.));armed=target['gate']
            offset=0. if armed=='light_1' else float(cycle.get('light_2_offset_s',7.))
            period=float(cycle.get('period_s',28.));red=float(cycle.get('red_s',10.))
            green=float(cycle.get('green_s',15.))
            phase=(t+offset)%period
            state='red' if phase<red else 'green' if phase<red+green else 'yellow'
            if policy.stop:policy.signal({'light_id':armed,'state':state,'confidence':1.,'stamp':t},t)
            if core.phase=='observe':core.record_frame(target['name'],t,1)
            v,vy,w=core.step(pose,t,policy.ready(t));v,w=policy.filter(pose,v,w,t)
            pose[0]+=v*math.cos(pose[2])*dt;pose[1]+=v*math.sin(pose[2])*dt;pose[2]+=w*dt
            self.assertIsNone(body_violation(pose,layout,obstacles),'%s %s'%(target['name'],pose))
            self.assertIsNone(policy.failure)
            if core.phase=='done':break
        self.assertEqual(core.phase,'done')
        self.assertEqual(len(visited),len(layout['route']))

    def test_repeated_gate_index_gets_a_fresh_arm_key(self):
        # Controller keys are route-index + gate, not gate ID alone.  This
        # prevents a later visit to the same installation from inheriting a
        # cleared or stale policy.
        keys=[(3,'light_1'),(11,'light_1')]
        self.assertNotEqual(keys[0],keys[1])

    def test_world_population_matches_both_street_quotas(self):
        with io.open(os.path.join(PKG,'config','layout.json'),encoding='utf-8') as stream:
            layout=json.load(stream)
        with io.open(os.path.join(PKG,'config','scene_instances_for_evaluation_only.json'),
                     encoding='utf-8') as stream:
            instances=json.load(stream)
        world=ET.parse(os.path.join(PKG,'worlds','official_semifinal.world')).getroot()
        includes={item.findtext('name'):item.findtext('uri') for item in
                  world.findall('./world/include') if item.findtext('name')}

        def inside(x,y,polygon):
            found=False
            for index in range(len(polygon)):
                x1,y1=polygon[index];x2,y2=polygon[(index+1)%len(polygon)]
                if ((y1>y)!=(y2>y)) and x<(x2-x1)*(y-y1)/float(y2-y1)+x1:
                    found=not found
            return found

        counts={'A':{'resident':0,'visitor':0},
                'B':{'resident':0,'visitor':0}}
        people=[item for item in instances if item['name'].startswith('person_')]
        self.assertEqual(len(people),16)
        self.assertEqual(set(item['name'] for item in people),
                         set('person_%02d'%index for index in range(16)))
        for item in people:
            self.assertEqual(includes.get(item['name']),'model://'+item['model'])
            streets=[street for street,key in (('A','a_polygon'),('B','b_polygon'))
                     if inside(item['x'],item['y'],layout[key])]
            self.assertEqual(len(streets),1,item['name'])
            category='resident' if item['model'].startswith('resident_') else 'visitor'
            self.assertTrue(item['model'].startswith(('resident_','visitor_')))
            counts[streets[0]][category]+=1
        for street in ('A','B'):
            self.assertEqual(counts[street],{'resident':7,'visitor':1})
            self.assertEqual(layout['population']['by_street'][street]['total'],8)
        self.assertEqual(layout['population']['total'],16)

    def test_person_orientation_sets_match_official_arrows(self):
        """The scene must preserve the directional arrows in the supplied plan."""
        with io.open(os.path.join(PKG,'config','layout.json'),encoding='utf-8') as stream:
            layout=json.load(stream)
        with io.open(os.path.join(PKG,'config','scene_instances_for_evaluation_only.json'),
                     encoding='utf-8') as stream:
            instances=json.load(stream)

        def inside(x,y,polygon):
            found=False
            for index in range(len(polygon)):
                x1,y1=polygon[index];x2,y2=polygon[(index+1)%len(polygon)]
                if ((y1>y)!=(y2>y)) and x<(x2-x1)*(y-y1)/float(y2-y1)+x1:
                    found=not found
            return found

        def direction(yaw):
            # Card artwork faces local -Y; rotate that normal into map frame.
            x=math.sin(float(yaw));y=-math.cos(float(yaw))
            candidates=[('east',1.,0.),('north',0.,1.),
                        ('west',-1.,0.),('south',0.,-1.)]
            return max(candidates,key=lambda item:x*item[1]+y*item[2])[0]

        observed={'A':set(),'B':set()}
        for item in instances:
            if not item['name'].startswith('person_'):continue
            street='A' if inside(item['x'],item['y'],layout['a_polygon']) else \
                   'B' if inside(item['x'],item['y'],layout['b_polygon']) else None
            self.assertIsNotNone(street,item['name'])
            observed[street].add(direction(item['yaw']))
        self.assertEqual(observed['A'],set(layout['person_orientation_policy']['A']))
        self.assertEqual(observed['B'],set(layout['person_orientation_policy']['B']))

    def test_vm_run_lap_summary_executes_and_checks_population(self):
        root=os.path.abspath(os.path.join(PKG,'..','..','..'))
        with io.open(os.path.join(root,'vm_run_lap.sh'),encoding='utf-8') as stream:
            shell=stream.read()
        script=shell.split("python - \"$RUN_DIR\" \"$PKG\" <<'PY'\n",1)[1].split('\nPY',1)[0]
        with io.open(os.path.join(PKG,'config','layout.json'),encoding='utf-8') as stream:
            layout=json.load(stream)
        labels=[item['expected_label'] for item in layout['route']
                if item.get('expected_category')=='plate']
        detections=[{'commit_state':'committed_reference_match','label':label}
                    for label in labels]
        result={'task':{'phase':'done'},
                'stop_crossings':[{'light_id':light,'pass':True}
                                  for light in ('light_1','light_2')],
                'body_violations':[], 'red_body_violations':[],
                'street_summary':{'A':{'resident':7,'visitor':1,'total':8},
                                  'B':{'resident':7,'visitor':1,'total':8}},
                'events':[{'stamp':stamp,'detections':detections}
                          for stamp in (1.,2.,3.)]}
        folder=tempfile.mkdtemp(prefix='semifinal_runner_')
        try:
            with open(os.path.join(folder,'run_result.json'),'wb') as stream:
                stream.write(json.dumps(result).encode('utf-8'))
            process=subprocess.Popen([sys.executable,'-c',script,folder,PKG],
                                     stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            output,error=process.communicate()
            self.assertEqual(process.returncode,0,(output,error))
            with io.open(os.path.join(folder,'INDEX.md'),encoding='utf-8') as stream:
                index=stream.read()
            self.assertIn('Result: PASS',index)
            self.assertIn('Total detected population matches supplied scene (16): True',index)
        finally:
            shutil.rmtree(folder)

    def test_generated_signal_plugins_match_layout_cycle(self):
        with io.open(os.path.join(PKG,'config','layout.json'),encoding='utf-8') as stream:
            cycle=json.load(stream)['signal_cycle']
        root=ET.parse(os.path.join(PKG,'worlds','official_semifinal.world')).getroot()
        models={m.get('name'):m for m in root.findall('./world/model')}
        for name,offset in [('light_1',0.0),('light_2',cycle['light_2_offset_s'])]:
            plugin=models[name].find("plugin[@name='signal_cycle']")
            self.assertIsNotNone(plugin)
            self.assertAlmostEqual(float(plugin.findtext('offset')),float(offset))
            self.assertAlmostEqual(float(plugin.findtext('period')),float(cycle['period_s']))
            self.assertAlmostEqual(float(plugin.findtext('red')),float(cycle['red_s']))
            self.assertAlmostEqual(float(plugin.findtext('green')),float(cycle['green_s']))
            self.assertLessEqual(float(plugin.findtext('red'))+float(plugin.findtext('green')),
                                 float(plugin.findtext('period')))

    def test_previously_missed_people_have_clear_legal_observation_views(self):
        package=PKG
        with io.open(os.path.join(package,'config','layout.json'),encoding='utf-8') as stream:
            layout=json.load(stream)
        with io.open(os.path.join(package,'assets','manifest.json'),encoding='utf-8') as stream:
            manifest=json.load(stream)
        assets={os.path.splitext(row['file'])[0]:row for row in manifest['recognition_assets']}
        with io.open(os.path.join(package,'config','scene_instances_for_evaluation_only.json'),encoding='utf-8') as stream:
            instances=json.load(stream)
        camera=camera_from_urdf(os.path.join(package,'urdf','semifinal_bot.urdf'))
        obstacles=physical_obstacles(os.path.join(package,'worlds','official_semifinal.world'))
        expected={'person_03':'street_a_west','person_15':'street_b_east_side'}
        for instance_name,view_name in expected.items():
            obj=next(row for row in instances if row['name']==instance_name)
            asset=assets[obj['model']]
            view=next(row for row in layout['route'] if row['name']==view_name)
            pose=tuple(view['xy'])+(math.radians(view['yaw']),)
            self.assertIsNone(body_violation(pose,layout,obstacles),instance_name)
            result=visible_card(obj,asset['width_m'],asset['height_m'],pose,camera)
            self.assertIsNotNone(result,instance_name)
            self.assertGreaterEqual(result['front_cosine'],.55,instance_name)
            self.assertGreaterEqual(result['long_pixels'],180.,instance_name)

    def test_signal_plugin_has_no_unbound_sdf_runtime_reference(self):
        path=os.path.join(PKG,'src','signal_plugin.cpp')
        with io.open(path,encoding='utf-8') as stream:source=stream.read()
        self.assertNotIn('sdf_->',source)
        for name in ('period_','red_','green_'):
            self.assertIn(name,source)


if __name__=='__main__':unittest.main(verbosity=2)
