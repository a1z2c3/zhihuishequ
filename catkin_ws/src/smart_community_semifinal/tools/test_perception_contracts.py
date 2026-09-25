#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Observation contracts with ROS message imports isolated from the test VM."""
from __future__ import division,print_function,unicode_literals
import os,sys,threading,types,unittest
import numpy as np

PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
for name in ('rospy','tf','sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg'):
    sys.modules[str(name)]=types.ModuleType(str(name))
sys.modules['rospy'].Duration=lambda value:value
sys.modules['tf'].Exception=Exception
sys.modules['sensor_msgs.msg'].Image=object
sys.modules['sensor_msgs.msg'].CameraInfo=object
sys.modules['std_msgs.msg'].String=object
import official_perception_node as perception
from official_perception_node import Node
from semifinal_core import StreetLedger


class PerceptionContracts(unittest.TestCase):
    def test_other_known_plate_is_recorded_without_expected_label_gate(self):
        node=Node.__new__(Node);node.plate_frames={};node.plate_reference_frames={}
        context={'expected_category':'plate','expected_label':'鄂D7B5Q2'}
        detection={'category':'plate','label':'苏AB8Q62',
                   'ocr_result':{'text':'苏AB8Q62','complete':True}}
        self.assertTrue(node._commit_detection(detection,context,None,1.,None))
        self.assertTrue(detection['unexpected_label'])
        self.assertEqual(detection['expected_label'],'鄂D7B5Q2')
        self.assertEqual(detection['commit_state'],'committed_reference_match')
        self.assertEqual(detection['ocr_status'],'recognized_and_verified')
        self.assertEqual(node.plate_frames,{})
        self.assertEqual(node.plate_reference_frames,{})

    def test_plate_route_quorum_is_independent_from_character_ocr(self):
        node=Node.__new__(Node);node.plate_frames={}
        node.plate_reference_frames={}
        context={'expected_category':'plate','expected_label':'苏AB8Q62'}
        detection={'category':'plate','label':'苏AB8Q62',
                   'ocr_result':{'text':'苏AB?Q62','complete':False}}
        self.assertTrue(node._commit_detection(detection,context,None,1.,None))
        self.assertEqual(Node._stable_count(context,[detection],node.plate_frames,{},
                                            node.plate_reference_frames),1)
        self.assertEqual(Node._stable_count(context,[detection],node.plate_frames,{}),0)
        detection['ocr_result']={'text':'苏AB8Q62','complete':True}
        for stamp in (2.,3.,4.):
            self.assertTrue(node._commit_detection(detection,context,None,stamp,None))
        self.assertEqual(Node._stable_count(context,[detection],node.plate_frames,{},
                                            node.plate_reference_frames),4)
        self.assertEqual(Node._stable_count(context,[detection],node.plate_frames,{}),3)

    def test_reference_quorum_can_finish_when_ocr_never_completes(self):
        node=Node.__new__(Node);node.plate_frames={};node.plate_reference_frames={}
        context={'expected_category':'plate','expected_label':'苏AB8Q62'}
        for stamp in (1.,2.,3.):
            detection={'category':'plate','label':'苏AB8Q62',
                       'ocr_result':{'text':'苏AB?Q62','complete':False}}
            self.assertTrue(node._commit_detection(detection,context,None,stamp,None))
        self.assertEqual(Node._stable_count(context,[detection],node.plate_frames,{},
                                            node.plate_reference_frames),3)
        self.assertEqual(Node._stable_count(context,[detection],node.plate_frames,{}),0)

    def test_plate_display_uses_cross_frame_consensus_and_names_pending_slots(self):
        node=Node.__new__(Node);node.plate_frames={};node.plate_reference_frames={}
        node.ledger=StreetLedger()
        context={'expected_category':'plate','expected_label':'苏AB8Q62'}
        for stamp in (1.,2.,3.):
            detection={'category':'plate','label':'苏AB8Q62',
                       'ocr_result':{'text':'苏AB?Q62','complete':False}}
            node._commit_detection(detection,context,None,stamp,None)
        self.assertEqual(detection['ocr_status'],'uncertain:slots=4')
        self.assertEqual(detection['ocr_consensus']['text'],'苏AB?Q62')
        self.assertEqual(detection['ocr_pending_slots'],[4])
        for stamp in (4.,5.,6.):
            detection={'category':'plate','label':'苏AB8Q62',
                       'ocr_result':{'text':'苏AB8Q62','complete':True}}
            node._commit_detection(detection,context,None,stamp,None)
        self.assertEqual(detection['ocr_consensus']['text'],'苏AB8Q62')
        self.assertEqual(detection['ocr_pending_slots'],[])
        self.assertEqual(detection['ocr_display_text'],'苏AB8Q62')
        self.assertEqual(detection['ocr_display_status'],'verified')

    def test_ocr_tie_is_pending_and_wrong_bay_cannot_vote(self):
        node=Node.__new__(Node);node.ledger=StreetLedger()
        node.plate_ocr_votes={'苏AB8Q62':{3:{'8':3,'B':3}}}
        self.assertIn(4,node._ocr_consensus('苏AB8Q62')['pending_slots'])
        node.plate_frames={};node.plate_reference_frames={};node.plate_results={}
        detection={'category':'plate','label':'苏AB8Q62',
                   'ocr_result':{'text':'苏AB8Q62','complete':True}}
        node._commit_detection(detection,{'expected_category':'plate',
                               'expected_label':'鄂D7B5Q2'},None,1.,None)
        self.assertEqual(node.plate_ocr_votes['苏AB8Q62'][3],{'8':3,'B':3})
        self.assertNotIn('苏AB8Q62',node.plate_results)

    def test_plate_summary_survives_next_observation_context(self):
        node=Node.__new__(Node);node.lock=threading.RLock()
        node.pending=None;node.context={};node.context_id=None
        node.plate_results={'苏AB8Q62':{'text':'苏AB8Q62','complete':True,
                                         'pending_slots':[]}}
        message=type(str('Message'),(object,),{})()
        message.data='{"active":true,"context_id":"next","armed_at":2}'
        node.configure(message)
        self.assertEqual(node.plate_results['苏AB8Q62']['text'],'苏AB8Q62')
        self.assertEqual(node.plate_ocr_votes,{})

    def test_plate_reference_match_requires_current_parking_bay_geometry(self):
        class Listener(object):
            def waitForTransform(self,base,frame,stamp,duration):
                return True
            def lookupTransform(self,base,frame,stamp):
                return (0.,0.,0.),(0.,0.,0.,1.)
        node=Node.__new__(Node);node.plate_frames={};node.plate_reference_frames={}
        node.plate_ocr_votes={};node.listener=Listener()
        original_position=perception.planar_position
        original_transforms=getattr(perception.tf,'transformations',None)
        transforms=types.ModuleType(str('transforms'))
        transforms.quaternion_matrix=lambda quaternion:np.eye(4)
        try:
            perception.planar_position=lambda item,K:{'camera_xyz':[1.,1.,1.],
                                                       'method':'synthetic'}
            perception.tf.transformations=transforms
            message=type(str('ImageMessage'),(object,),{})()
            message.header=type(str('Header'),(object,),{})()
            message.header.frame_id='camera';message.header.stamp=2.0
            detection={'category':'plate','label':'苏AB8Q62','width_m':.095,
                       'height_m':.03,'ocr_result':{'text':'苏AB8Q62','complete':True}}
            self.assertFalse(node._commit_detection(detection,
                             {'expected_category':'plate','expected_label':'苏AB8Q62',
                              'target_xy':[2.,2.]},[1.]*9,2.,message))
            self.assertEqual(detection['rejection_reason'],'outside_expected_plate_bay')
        finally:
            perception.planar_position=original_position
            perception.tf.transformations=original_transforms

    def test_only_current_committed_target_can_satisfy_quorum(self):
        context={'expected_category':'plate'}
        earlier={'category':'plate','label':'苏AB8Q62'}
        current={'category':'plate','label':'鄂D7B5Q2'}
        frames={'苏AB8Q62':set([1.,2.,3.]),'鄂D7B5Q2':set([3.])}
        self.assertEqual(Node._stable_count(context,[current],frames,{}),1)
        frames[current['label']].update([4.,5.])
        self.assertEqual(Node._stable_count(context,[current],frames,{}),3)
        self.assertEqual(Node._stable_count(context,[],frames,{}),0)
        person={'category':'resident','instance_id':'A:1'}
        instance_frames={'A:1':set([1.,2.,3.]),'A:2':set([3.])}
        self.assertEqual(Node._stable_count({'expected_category':'person'},
                         [{'category':'resident','instance_id':'A:2'}],{},
                         instance_frames),1)
        self.assertEqual(Node._stable_count({'expected_category':'person'},
                         [person],{},instance_frames),3)

    def test_wrong_category_still_cannot_complete_observation(self):
        node=Node.__new__(Node);node.plate_frames={}
        detection={'category':'plate','label':'苏AB8Q62'}
        self.assertFalse(node._commit_detection(detection,
                         {'expected_category':'person'},None,1.,None))
        self.assertEqual(detection['rejection_reason'],'unexpected_category')

    def test_person_commit_uses_current_image_for_tf_and_ledger(self):
        class Listener(object):
            calls=[]
            def waitForTransform(self,base,frame,stamp,duration):
                return True
            def lookupTransform(self,base,frame,stamp):
                self.calls.append((base,frame,stamp))
                return (0.,0.,0.),(0.,0.,0.,1.)

        node=Node.__new__(Node);node.listener=Listener();node.ledger=StreetLedger()
        node.localized_labels=set();node.committed_ids=set()
        node.context_instance_frames={}
        node._in_expected_region=lambda street,point: street=='A'
        header=type(str('Header'),(object,),{'frame_id':'camera','stamp':2.0})()
        message=type(str('ImageMessage'),(object,),{'header':header})()
        detection={'category':'resident','label':'resident_1'}
        original_position=perception.planar_position
        original_transforms=getattr(perception.tf,'transformations',None)
        transforms=types.ModuleType(str('transforms'))
        transforms.quaternion_matrix=lambda quaternion:np.eye(4)
        try:
            perception.planar_position=lambda item,K:{'camera_xyz':[1.,1.,1.],
                                                       'method':'synthetic'}
            perception.tf.transformations=transforms
            self.assertTrue(node._commit_detection(detection,
                            {'expected_category':'person','street':'A'},
                            [1.]*9,2.0,message))
        finally:
            perception.planar_position=original_position
            perception.tf.transformations=original_transforms
        self.assertEqual(node.listener.calls,[('map','camera',2.0)])
        self.assertEqual(node.ledger.summary('A')['total'],0)
        self.assertEqual(detection['commit_state'],'committed')
        self.assertEqual(node.context_instance_frames[detection['instance_id']],
                         set([2.0]))


if __name__=='__main__':unittest.main(verbosity=2)
