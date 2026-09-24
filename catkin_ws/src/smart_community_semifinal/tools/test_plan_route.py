#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dependency-free contracts for the saved-map A* route generator."""
from __future__ import division
import io
import json
import os
import shutil
import sys
import tempfile
import unittest

PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.dirname(__file__))
import plan_route
import build_navigation_map


class PlanRouteContracts(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.mkdtemp(prefix='astar_route_')

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _pgm(self,width,height,occupied=()):
        pixels=[255]*(width*height)
        for col,row in occupied:pixels[row*width+col]=0
        path=os.path.join(self.tmp,'map.pgm')
        with open(path,'wb') as stream:
            stream.write(('P5\n%d %d\n255\n'%(width,height)).encode('ascii'))
            stream.write(bytearray(pixels))
        return path

    def test_diagonal_corner_cutting_is_rejected(self):
        blocked=[False]*9
        blocked[0*3+1]=True
        blocked[1*3+0]=True
        with self.assertRaises(ValueError):
            plan_route.astar((0,0),(2,2),blocked,3,3)

    def test_plan_preserves_semantic_targets_and_inserts_detour(self):
        pgm=self._pgm(7,7,occupied=[(3,2),(3,3),(3,4)])
        layout={'route':[{'name':'start','xy':[0.,4.],'yaw':0},
                         {'name':'gate','xy':[6.,0.],'yaw':90,'gate':'light_1'},
                         {'name':'observe','xy':[6.,4.],'yaw':0,'observe':True,
                          'street':'A','expected_category':'person'}],
                'field_size_m':[7.,7.]}
        result=plan_route.plan(layout,{'resolution':1.,'origin':[0.,0.,0.],
                                       'occupied_thresh':.65,'negate':0},pgm,.0)
        self.assertEqual(result['route'][0]['name'],'start')
        self.assertEqual(result['route'][-1]['name'],'observe')
        names=[item['name'] for item in result['route']]
        self.assertLess(names.index('start'),names.index('gate'))
        self.assertLess(names.index('gate'),names.index('observe'))
        self.assertEqual(result['route'][names.index('gate')]['gate'],'light_1')
        self.assertTrue(result['planner']['segments'])
        self.assertTrue(any(item['name'].startswith('plan_') for item in result['route']))
        for item in result['route']:
            if item['name'].startswith('plan_'):
                self.assertEqual(item['planner'],'astar')

    def test_saved_map_generation_uses_yaml_occupied_threshold(self):
        path=self._pgm(2,1,occupied=[(0,0)])
        width,height,maxval,pixels=plan_route.read_pgm(path)
        blocked=plan_route.inflate(width,height,pixels,0,255*(1-.65),False)
        self.assertTrue(blocked[0])
        self.assertFalse(blocked[1])

    def test_inflate_accepts_python_two_character_pixels(self):
        # bytes indexing returns str on Python 2 but int on Python 3.
        self.assertEqual(plan_route.inflate(2,1,u'\x00\xff',0,100),
                         [True,False])
        self.assertEqual(plan_route.inflate(2,1,u'\x00\xff',0,100,True),
                         [False,True])

    def test_route_json_writes_unicode_under_python_two(self):
        path=os.path.join(self.tmp,'route.json')
        plan_route.write_route_json(path,{'label':u'苏AB8Q62'})
        with io.open(path,encoding='utf-8') as stream:
            self.assertEqual(json.load(stream)['label'],u'苏AB8Q62')

    def test_field_boundary_is_not_used_as_free_shortcut(self):
        blocked=[False]*25
        constrained=plan_route.constrain_to_field(blocked,5,5,[0.,0.,0.],1.,[4.,4.],1.)
        self.assertTrue(constrained[0])
        self.assertTrue(constrained[4])
        self.assertFalse(constrained[2*5+2])

    def test_rule_zone_is_not_used_as_free_shortcut(self):
        blocked=[False]*25
        layout={'a_polygon':[[1.,3.],[4.,3.],[4.,1.],[3.,1.],[3.,2.],[1.,2.]],
                'b_polygon':[[0.,0.],[.5,0.],[.5,.5],[0.,.5]],
                'parking_boundary_x':4.,'parking_open_above_y':4.,'field_size_m':[5.,5.]}
        constrained=plan_route.constrain_rule_zones(blocked,5,5,[0.,0.,0.],1.,layout)
        self.assertTrue(constrained[2*5+2])
        self.assertFalse(constrained[0*5+0])

    def test_navigation_map_preserves_dimensions_and_overlays_rules(self):
        source=self._pgm(5,5)
        out=os.path.join(self.tmp,'navigation.pgm')
        layout={'field_size_m':[5.,5.],
                'a_polygon':[[1.,3.],[4.,3.],[4.,1.],[3.,1.],[3.,2.],[1.,2.]],
                'b_polygon':[[0.,0.],[.5,0.],[.5,.5],[0.,.5]],
                'parking_boundary_x':4.,'parking_open_above_y':4.}
        width,height,semantic=build_navigation_map.build(
            layout,{'resolution':1.,'origin':[0.,0.,0.]},source,out)
        self.assertEqual((width,height),(5,5));self.assertGreater(semantic,0)
        self.assertEqual(plan_route.read_pgm(out)[:2],(5,5))


if __name__=='__main__':
    unittest.main(verbosity=2)
