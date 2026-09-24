#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-free tests for route-corridor footprint supervision."""
from __future__ import division
import math
import os
import sys
import unittest

PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
from scene_geometry import (body_violation,lane_corridor_error,
                            lane_recovery_allowed,trajectory_violation,
                            lane_recovery_vector,lane_limited_twist)


class LaneGeometryContracts(unittest.TestCase):
    def setUp(self):
        self.layout={'field_size_m':[4.,3.], 'lane_width_m':.6,
                     'lane_safety_margin_m':.02,
                     'lane_centerline':[[.5,1.5],[3.5,1.5]]}

    def test_centered_footprint_is_inside_corridor(self):
        self.assertLessEqual(lane_corridor_error((2.,1.5,0.),self.layout),0.)
        self.assertIsNone(body_violation((2.,1.5,0.),self.layout))

    def test_corridor_checks_full_body_not_only_robot_center(self):
        self.assertLess(lane_corridor_error((2.,1.60,0.),self.layout),0.)
        self.assertEqual(body_violation((2.,1.62,0.),self.layout),'lane_corridor')

    def test_physical_lane_boundary_is_distinct_from_engineering_reserve(self):
        pose=(2.,1.62,0.)
        physical=dict(self.layout,lane_safety_margin_m=0.)
        self.assertEqual(body_violation(pose,self.layout),'lane_corridor')
        self.assertIsNone(body_violation(pose,physical))
        self.assertGreater(-lane_corridor_error(pose,physical),0.)

    def test_polyline_corner_supports_a_turn_without_false_positive(self):
        self.layout['lane_centerline']=[[.5,1.5],[2.,1.5],[2.,2.5]]
        self.assertIsNone(body_violation((1.95,1.55,math.pi/4),self.layout))

    def test_turn_strip_union_allows_designed_lateral_avoidance(self):
        self.layout['lane_centerline']=[[.5,1.5],[2.,1.5],[2.,2.5]]
        yaw=math.pi/4
        pose=(2.-math.sin(yaw)*.09,1.5+math.cos(yaw)*.09,yaw)
        self.assertLessEqual(lane_corridor_error(pose,self.layout),0.)
        self.assertIsNone(body_violation(pose,self.layout))

    def test_lane_limiter_preserves_forward_component(self):
        pose=(2.,1.59,0.)
        result=lane_limited_twist(pose,.10,.12,0.,self.layout)
        speed,lateral,omega,violation,limited=result
        self.assertTrue(limited)
        self.assertIsNone(violation)
        self.assertEqual(speed,.10)
        self.assertLess(abs(lateral),.12)

    def test_recovery_vector_points_back_to_straight_strip(self):
        vector=lane_recovery_vector((2.,1.82,0.),self.layout)
        self.assertAlmostEqual(vector[0],0.)
        self.assertAlmostEqual(vector[1],-.32)

    def test_recovery_only_allows_monotonic_motion_toward_corridor(self):
        current=(2.,1.62,0.)
        inward=[(2.,1.615,0.),(2.,1.61,0.),(2.,1.605,0.)]
        outward=[(2.,1.625,0.),(2.,1.63,0.),(2.,1.635,0.)]
        self.assertTrue(lane_recovery_allowed(current,inward,self.layout))
        self.assertFalse(lane_recovery_allowed(current,outward,self.layout))
        self.assertIsNone(trajectory_violation(current,inward,self.layout))
        self.assertEqual(trajectory_violation(current,outward,self.layout),'lane_corridor')

    def test_prediction_rejects_non_lane_collision_during_recovery(self):
        self.layout['field_size_m']=[2.1,3.]
        current=(2.02,1.62,0.)
        inward=[(2.015,1.615,0.),(2.01,1.61,0.),(2.005,1.605,0.)]
        self.assertEqual(trajectory_violation(current,inward,self.layout),'field_boundary')

    def test_route_corridor_is_required_when_centerline_exists(self):
        del self.layout['lane_centerline']
        self.assertIsNone(body_violation((2.,1.5,0.),self.layout))


if __name__=='__main__':unittest.main(verbosity=2)
