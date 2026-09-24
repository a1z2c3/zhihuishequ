#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pure-Python closed-loop regression tests for lateral obstacle avoidance.

The production guard consumes LaserScan points, not a hand-written sequence of
booleans.  These tests render a 2-D rectangular obstacle into a 720-beam scan,
feed it through ``scan_clearance`` and ``Patrol``, integrate the mecanum body
command, and check the inflated footprint at every step.
"""
from __future__ import division
import math
import os
import sys
import unittest

PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
from patrol_core import Patrol
from semifinal_core import scan_clearance,footprint_corners,integrate_twist_pose


class Scan(object):
    pass


def render_scan(pose,box,beams=720):
    """Render sampled rectangle boundary points into a nearest-hit scan."""
    scan=Scan();scan.range_min=.02;scan.range_max=8.;scan.angle_min=-math.pi
    scan.angle_increment=2*math.pi/beams;scan.ranges=[8.]*beams
    x0,x1,y0,y1=box;points=[]
    for k in range(31):
        u=k/30.
        points.extend([(x0+(x1-x0)*u,y0),(x0+(x1-x0)*u,y1),
                       (x0,y0+(y1-y0)*u),(x1,y0+(y1-y0)*u)])
    x,y,yaw=pose;co,si=math.cos(yaw),math.sin(yaw)
    for px,py in points:
        dx,dy=px-x,py-y
        bx=co*dx+si*dy;by=-si*dx+co*dy
        distance=math.hypot(bx,by)
        if distance<scan.range_min or distance>=scan.range_max:continue
        angle=math.atan2(by,bx)
        index=int(round((angle-scan.angle_min)/scan.angle_increment))%beams
        scan.ranges[index]=min(scan.ranges[index],distance)
    return scan


def integrate(pose,command,dt):
    x,y,yaw=pose;vx,vy,wz=command
    return [x+(vx*math.cos(yaw)-vy*math.sin(yaw))*dt,
            y+(vx*math.sin(yaw)+vy*math.cos(yaw))*dt,
            yaw+wz*dt]


def overlap_axis_aligned(pose,box):
    """Conservative footprint/box overlap for these yaw=0 maneuvers."""
    body=footprint_corners(*pose)
    bx0=min(p[0] for p in body);bx1=max(p[0] for p in body)
    by0=min(p[1] for p in body);by1=max(p[1] for p in body)
    x0,x1,y0,y1=box
    return bx0<x1 and bx1>x0 and by0<y1 and by1>y0


class ClosedLoopAvoidance(unittest.TestCase):
    def test_curved_holonomic_projection_is_not_linearized(self):
        pose=integrate_twist_pose((0.,0.,0.),.2,0.,1.,1.)
        self.assertAlmostEqual(pose[0],.2*math.sin(1.),places=6)
        self.assertAlmostEqual(pose[1],.2*(1.-math.cos(1.)),places=6)

    def run_box(self,box,expected_side):
        patrol=Patrol([{'name':'goal','xy':[1.5,0.],'yaw':0}])
        pose=[0.,0.,0.];dt=.05;max_steps=1200;avoid_seen=False;recenter_seen=False;chosen_side=None
        for step in range(max_steps):
            scan=render_scan(pose,box)
            guard=scan_clearance(scan,.18)
            command=patrol.step(pose,step*dt,guard=guard)
            if patrol.phase=='avoid':
                avoid_seen=True
                if chosen_side is None:chosen_side=patrol.avoid_side
            if patrol.phase=='recenter':recenter_seen=True
            # The control contract is safety-first: no integrated command may
            # sweep the inflated footprint through the dynamic box.
            pose=integrate(pose,command,dt)
            self.assertFalse(overlap_axis_aligned(pose,box),
                             'collision phase=%s pose=%r guard=%r'%(patrol.phase,pose,guard))
            if patrol.phase=='done':break
        self.assertEqual(patrol.phase,'done',patrol.error)
        self.assertTrue(avoid_seen);self.assertTrue(recenter_seen)
        self.assertLess(abs(pose[1]),.02)
        self.assertEqual(chosen_side,expected_side)

    def test_long_right_offset_obstacle_closes_without_retrigger(self):
        self.run_box((.30,.66,-.24,-.12),'left')

    def test_long_left_offset_obstacle_closes_without_retrigger(self):
        self.run_box((.30,.66,.12,.24),'right')

    def test_centered_unpassable_obstacle_waits_without_stall(self):
        patrol=Patrol([{'name':'goal','xy':[1.5,0.],'yaw':0}])
        box=(.30,.60,-.15,.15)
        for step in range(1600):
            guard=scan_clearance(render_scan((0.,0.,0.),box),.18)
            command=patrol.step((0.,0.,0.),step*.05,guard=guard)
            self.assertEqual(command,(0.,0.,0.))
            self.assertEqual(patrol.phase,'travel')
        self.assertIsNone(patrol.error)


if __name__=='__main__':
    unittest.main(verbosity=2)
