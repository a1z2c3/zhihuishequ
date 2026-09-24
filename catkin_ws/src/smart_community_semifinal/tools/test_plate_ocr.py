#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Character-level recognition checks on the supplied plate artwork set."""
from __future__ import division,print_function,unicode_literals
import json
import io
import os
import sys
import unittest
import cv2
import numpy as np

PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
from plate_ocr import PlateCharacterRecognizer
from reference_detector import ReferenceDetector
from PIL import ImageDraw


class PlateOCRContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with io.open(os.path.join(PKG,'assets','manifest.json'),'r',encoding='utf-8') as stream:
            manifest=json.load(stream)
        assets=[];cls.images={}
        for item in manifest['recognition_assets']:
            if item['category']!='plate':continue
            image=cv2.imdecode(np.fromfile(os.path.join(PKG,'assets',item['file']),
                                           dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            assets.append((image,item['label']));cls.images[item['label']]=image
        cls.assets=assets
        cls.recognizer=PlateCharacterRecognizer(assets)

    def test_supplied_plates_are_read_character_by_character(self):
        for label,image in self.images.items():
            result=self.recognizer.recognize(image)
            self.assertTrue(result['complete'],label+': '+repr(result))
            self.assertEqual(result['text'],label)
            self.assertEqual(len(result['characters']),7)
            self.assertTrue(all(char['accepted'] for char in result['characters']))

    def test_bounded_perspective_and_illumination_perturbation(self):
        image=self.images['苏AB8Q62'];height,width=image.shape
        source=np.float32([[0,0],[width-1,0],[width-1,height-1],[0,height-1]])
        target=np.float32([[5,3],[width-7,0],[width-1,height-5],[0,height-1]])
        forward=cv2.getPerspectiveTransform(source,target)
        captured=cv2.warpPerspective(image,forward,(width,height),borderValue=15)
        inverse=cv2.getPerspectiveTransform(target,source)
        rectified=cv2.warpPerspective(captured,inverse,(width,height))
        rectified=np.clip(rectified.astype(np.float32)*.86+18,0,255).astype(np.uint8)
        rectified=cv2.GaussianBlur(rectified,(3,3),.4)
        result=self.recognizer.recognize(rectified)
        self.assertTrue(result['complete'],result)
        self.assertEqual(result['text'],'苏AB8Q62')

    def test_leave_one_plate_out_rejects_unseen_characters(self):
        """The test must measure rejection, not template self-reconstruction."""
        assets=[(image,label) for label,image in self.images.items()]
        for held_out_index,(image,label) in enumerate(assets):
            train=[item for index,item in enumerate(assets)
                   if index!=held_out_index]
            recognizer=PlateCharacterRecognizer(train)
            result=recognizer.recognize(image)
            self.assertFalse(result['complete'],label+': '+repr(result))
            self.assertNotEqual(result['text'],label)
            # A glyph absent from the training plates must never be promoted
            # to a different known glyph merely because it is similar.
            for index,char in enumerate(label):
                available=recognizer.slot_templates[index]
                if char not in available:
                    self.assertEqual(result['characters'][index]['value'],'?',
                                     '%s slot %d: %r'%(label,index,result))

    def test_rendered_gazebo_plate_crops_are_read_without_reference_labels(self):
        # Small crops from the three 2026-09-21 Gazebo camera evidence frames;
        # these images are test inputs, never recognizer templates at runtime.
        folder=os.path.join(PKG,'tools','fixtures','rendered_plates')
        for index,(unused,label) in enumerate(self.assets):
            path=os.path.join(folder,'parking_%d.png'%(index+1))
            image=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            result=self.recognizer.recognize(image)
            self.assertTrue(result['complete'],label+': '+repr(result))
            self.assertEqual(result['text'],label)

    def test_one_corrupted_character_does_not_discard_other_slots(self):
        original=self.recognizer._binary_glyph
        calls=[0]
        def fail_fourth(crop):
            calls[0]+=1
            if calls[0]==4:raise ValueError('corrupt slot')
            return original(crop)
        try:
            self.recognizer._binary_glyph=fail_fourth
            result=self.recognizer.recognize(self.images['苏AB8Q62'])
        finally:
            self.recognizer._binary_glyph=original
        self.assertEqual(result['text'],'苏AB?Q62')
        self.assertFalse(result['complete'])

    def test_l_slot_rejects_vertical_without_bottom_bar(self):
        plate=self.recognizer._normalize_plate(self.images['苏APL12A']).copy()
        y0=int(round(self.recognizer.Y_RANGE[0]*self.recognizer.HEIGHT))
        y1=int(round(self.recognizer.Y_RANGE[1]*self.recognizer.HEIGHT))
        x0=int(round(self.recognizer.SLOTS[3][0]*self.recognizer.WIDTH))
        x1=int(round(self.recognizer.SLOTS[3][1]*self.recognizer.WIDTH))
        plate[y0:y1,x0:x1]=np.median(plate[y0:y1,x0:x1]).astype(np.uint8)
        cv2.line(plate,(x0+5,y0+7),(x0+5,y1-7),255,2)
        result=self.recognizer.recognize(plate)
        self.assertEqual(result['characters'][3]['value'],'?',result)

    def test_shape_rejected_candidates_remain_in_margin_ranking(self):
        # Shape rules may down-weight a candidate, but must not delete it:
        # otherwise a weak surviving template gets a fabricated one-item
        # margin and can be promoted as a confident character.
        template=next(iter(self.recognizer.templates.values()))[0]
        score=self.recognizer._candidate_score(template,(99,99),template,'A',1)
        self.assertIsInstance(score,float)
        self.assertGreaterEqual(score,0.)

    def test_b8_shape_ratio_survives_small_roll(self):
        image=self.images['鄂D7B5Q2'];height,width=image.shape
        matrix=cv2.getRotationMatrix2D((width/2.,height/2.),.75,1.)
        rolled=cv2.warpAffine(image,matrix,(width,height),borderValue=0)
        result=self.recognizer.recognize(rolled)
        # The B/8 decision must remain stable under the small residual roll
        # left after homography rectification.
        self.assertEqual(result['characters'][3]['value'],'B',result)

    def test_ocr_exception_keeps_other_targets_in_same_frame(self):
        detector=ReferenceDetector(os.path.join(PKG,'assets','manifest.json'))
        plate=next(ref for item,ref,kp,desc in detector.references
                   if item['category']=='plate')
        person=next(ref for item,ref,kp,desc in detector.references
                    if item['category']=='resident')
        frame=np.full((960,1280,3),180,np.uint8)
        for image,x,y in ((plate,60,80),(person,700,80)):
            h,w=image.shape
            frame[y:y+h,x:x+w]=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        def fail_ocr(unused):raise RuntimeError('injected')
        detector.plate_ocr.recognize=fail_ocr
        detections=detector.detect(frame)
        self.assertIn('resident',set(d['category'] for d in detections))
        plates=[d for d in detections if d['category']=='plate']
        self.assertEqual(len(plates),1)
        self.assertEqual(plates[0]['ocr_status'],'error')
        self.assertFalse(plates[0]['character_ocr'])

    def test_degenerate_homography_rejects_candidate_without_raising(self):
        detector=ReferenceDetector(os.path.join(PKG,'assets','manifest.json'))
        plate=next(ref for item,ref,kp,desc in detector.references
                   if item['category']=='plate')
        frame=np.full((960,1280,3),180,np.uint8)
        h,w=plate.shape
        frame[80:80+h,60:60+w]=cv2.cvtColor(plate,cv2.COLOR_GRAY2BGR)
        original=np.linalg.inv
        np.linalg.inv=lambda unused: (_ for unused in ()).throw(
            np.linalg.LinAlgError('degenerate'))
        try:
            detections=detector.detect(frame)
        finally:
            np.linalg.inv=original
        self.assertIsInstance(detections,list)

    def test_detector_category_filter_limits_reference_work(self):
        detector=ReferenceDetector(os.path.join(PKG,'assets','manifest.json'))
        frame=np.zeros((64,64,3),np.uint8)
        self.assertEqual(detector.detect(frame,categories=('plate',)),[])

    def test_annotation_uses_consensus_not_single_frame_ocr(self):
        original=ImageDraw.Draw
        labels=[]
        class CapturingDraw(object):
            def __init__(self,image):self.delegate=original(image)
            def rectangle(self,*args,**kwargs):
                return self.delegate.rectangle(*args,**kwargs)
            def text(self,xy,label,**kwargs):
                labels.append(label)
                return self.delegate.text(xy,label,**kwargs)
        ImageDraw.Draw=CapturingDraw
        try:
            frame=np.zeros((100,480,3),np.uint8)
            item={'category':'plate','label':'苏AB8Q62','confidence':.9,
                  'bbox':[10,30,180,30],
                  'ocr_result':{'text':'苏AB?Q62','complete':False},
                  'ocr_display_status':'pending','ocr_pending_slots':[4]}
            ReferenceDetector.annotate(frame,[item])
            self.assertIn('OCR:pending[4]',labels[-1])
            self.assertNotIn('苏AB?Q62',labels[-1])
            item['ocr_display_status']='verified'
            item['ocr_display_text']='苏AB8Q62'
            ReferenceDetector.annotate(frame,[item])
            self.assertIn('OCR:苏AB8Q62',labels[-1])
        finally:
            ImageDraw.Draw=original


if __name__=='__main__':unittest.main(verbosity=2)
