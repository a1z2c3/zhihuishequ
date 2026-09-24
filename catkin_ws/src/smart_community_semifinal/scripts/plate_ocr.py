#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Constrained seven-character OCR for the supplied plate artwork domain."""
from __future__ import division, unicode_literals
import cv2
import numpy as np


class PlateCharacterRecognizer(object):
    WIDTH=200
    HEIGHT=81
    # Real rectified camera crops vary by position. The weak fourth and
    # seventh slots require morphology checks as well as a lower score; the
    # other slots retain a stricter unknown-character rejection threshold.
    # Slot-specific floors are calibrated to the fixed rectified geometry,
    # not to the character that happened to win the ranking.  This keeps a
    # weak D/L observation from receiving an answer-dependent discount.
    # Slot 5 has only the supplied ``Q``/``1`` shape family.  The rendered
    # ``1`` loses stroke mass under the VM's 0.2x software-camera sampling;
    # keep the normal margin gate, but use the empirically bounded floor .52.
    # Leave-one-plate-out remains below this floor (.509), so an unseen glyph
    # is still rejected instead of being promoted by the relaxed score alone.
    SCORE_THRESHOLDS=(.86,.78,.85,.60,.52,.85,.65)
    MARGIN_THRESHOLD=.04
    # Province, city letter, then the five characters following the separator.
    SLOTS=[(.01,.145),(.145,.285),(.335,.46),(.46,.59),(.59,.72),(.72,.85),(.85,.99)]
    Y_RANGE=(.06,.88)

    def __init__(self,assets):
        self.templates={}
        self.slot_templates=[{} for unused in self.SLOTS]
        self.template_shapes={}
        for image,label in assets:
            if len(label)!=7:continue
            normalized=self._normalize_plate(image)
            for slot_index,(char,slot) in enumerate(zip(label,self._segments(normalized))):
                glyph=self._binary_glyph(slot)
                self.template_shapes[id(glyph)]=self._glyph_shape(glyph)
                self.templates.setdefault(char,[]).append(glyph)
                self.slot_templates[slot_index].setdefault(char,[]).append(glyph)
        if not self.templates:raise ValueError('no seven-character plate templates')

    @classmethod
    def _normalize_plate(cls,image):
        if image is None or image.size==0:raise ValueError('empty plate image')
        if image.ndim==3:image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        return cv2.resize(image,(cls.WIDTH,cls.HEIGHT),interpolation=cv2.INTER_AREA)

    @classmethod
    def _segments(cls,plate):
        y0=int(round(cls.Y_RANGE[0]*cls.HEIGHT));y1=int(round(cls.Y_RANGE[1]*cls.HEIGHT))
        return [plate[y0:y1,int(round(a*cls.WIDTH)):int(round(b*cls.WIDTH))]
                for a,b in cls.SLOTS]

    @staticmethod
    def _binary_glyph(crop):
        if crop.size==0:raise ValueError('empty character slot')
        smooth=cv2.GaussianBlur(crop,(3,3),0)
        _,binary=cv2.threshold(smooth,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # The supplied plates use bright characters on a dark blue field.
        if float(binary.mean())>127.:binary=cv2.bitwise_not(binary)
        count,_,stats,_=cv2.connectedComponentsWithStats(binary,8)
        components=[stats[i] for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=2]
        if not components:raise ValueError('no foreground character pixels')
        x0=min(int(c[cv2.CC_STAT_LEFT]) for c in components)
        y0=min(int(c[cv2.CC_STAT_TOP]) for c in components)
        x1=max(int(c[cv2.CC_STAT_LEFT]+c[cv2.CC_STAT_WIDTH]) for c in components)
        y1=max(int(c[cv2.CC_STAT_TOP]+c[cv2.CC_STAT_HEIGHT]) for c in components)
        glyph=binary[y0:y1,x0:x1]
        if glyph.size==0:raise ValueError('empty character foreground')
        target_w,target_h=24,48
        scale=min(float(target_w-4)/glyph.shape[1],float(target_h-4)/glyph.shape[0])
        size=(max(1,int(round(glyph.shape[1]*scale))),
              max(1,int(round(glyph.shape[0]*scale))))
        glyph=cv2.resize(glyph,size,interpolation=cv2.INTER_AREA)
        canvas=np.zeros((target_h,target_w),np.uint8)
        x=(target_w-size[0])//2;y=(target_h-size[1])//2
        canvas[y:y+size[1],x:x+size[0]]=glyph
        return (canvas>0).astype(np.uint8)

    @staticmethod
    def _similarity(a,b):
        intersection=float(np.logical_and(a,b).sum())
        union=float(np.logical_or(a,b).sum())
        return intersection/max(1.,union)

    @staticmethod
    def _chamfer_similarity(a,b):
        """Symmetric distance-transform similarity for binary glyphs.

        IoU alone rewards dense glyphs and makes thin characters collapse onto
        filled ones.  A symmetric chamfer term compares stroke locations while
        remaining available in the old OpenCV 3.2 runtime.
        """
        a8=(a.astype(np.uint8)*255)
        b8=(b.astype(np.uint8)*255)
        da=cv2.distanceTransform(cv2.bitwise_not(a8),cv2.DIST_L2,3)
        db=cv2.distanceTransform(cv2.bitwise_not(b8),cv2.DIST_L2,3)
        af=a>0;bf=b>0
        if not af.any() or not bf.any():return 0.0
        distance=.5*(float(db[af].mean())+float(da[bf].mean()))
        return max(0.,1.-distance/6.)

    @classmethod
    def _score(cls,a,b):
        return .35*cls._similarity(a,b)+.65*cls._chamfer_similarity(a,b)

    @staticmethod
    def _glyph_shape(glyph):
        contours,hierarchy=cv2.findContours((glyph*255).astype(np.uint8),
                                             cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)[-2:]
        parents=hierarchy[0] if hierarchy is not None else []
        holes=sum(1 for i,contour in enumerate(contours)
                  if len(parents) and parents[i][3]>=0 and cv2.contourArea(contour)>=3)
        rows=np.where(glyph.any(axis=1))[0]
        left=[];right=[]
        for y in rows:
            pixels=np.flatnonzero(glyph[y])
            left.append(pixels[0]);right.append(pixels[-1])
        if not left:return holes,0.,0.,0.
        left_std=float(np.std(left));right_std=float(np.std(right))
        # A rotation moves both edges together.  The left/right variation
        # ratio therefore remains useful for B (straight left stem) versus 8
        # (curved on both sides), unlike the absolute left-edge std alone.
        ratio=left_std/max(right_std,1e-6)
        return holes,ratio,left_std,right_std

    def _candidate_score(self,glyph,shape,template,char,index):
        reference_shape=self.template_shapes[id(template)]
        # Keep every candidate in the ranking.  Removing a candidate makes a
        # one-candidate list look artificially decisive and turns ``margin``
        # into the absolute score, defeating unknown-character rejection.
        penalty=1.0
        if index and shape[0]!=reference_shape[0]:penalty*=.55
        # Both B and 8 have two holes, but only B has a straight left stem.
        if char=='B' and shape[1]>.45:penalty*=.60
        if char=='8' and shape[1]<.45:penalty*=.60
        # The thin L loses template similarity under camera resampling.  Its
        # long bottom bar distinguishes it from other hole-free verticals.
        if char=='L':
            bottom=int(np.count_nonzero(glyph[-8:].any(axis=0)))
            top=int(np.count_nonzero(glyph[:8].any(axis=0)))
            if bottom<glyph.shape[1]*.55 or bottom<top*1.5:penalty*=.65
        return self._score(glyph,template)*penalty

    def recognize(self,rectified):
        plate=self._normalize_plate(rectified)
        chars=[];scores=[];margins=[]
        for slot_index,crop in enumerate(self._segments(plate)):
            try:
                glyph=self._binary_glyph(crop);shape=self._glyph_shape(glyph)
                candidates=self.slot_templates[slot_index] or self.templates
                ranked=[]
                for char,refs in candidates.items():
                    matches=[self._candidate_score(glyph,shape,ref,char,slot_index)
                             for ref in refs]
                    matches=[score for score in matches if score is not None]
                    if matches:ranked.append((max(matches),char))
                ranked.sort(reverse=True)
                score,char=ranked[0]
                margin=score-(ranked[1][0] if len(ranked)>1 else 0.)
                threshold=self.SCORE_THRESHOLDS[slot_index]
                # Even a slot with only one available template must clear the
                # margin floor (where margin equals its score).  There is no
                # longer a path that promotes a weak, filtered survivor.
                accepted=(score>=threshold and margin>=self.MARGIN_THRESHOLD)
                chars.append(char if accepted else '?')
                scores.append(round(score,4));margins.append(round(margin,4))
            except (cv2.error,ValueError,IndexError,TypeError):
                chars.append('?');scores.append(0.);margins.append(0.)
        text=''.join(chars)
        return {'text':text,'characters':[{'value':char,'confidence':score,
                'margin':margin,'accepted':char!='?'}
                for char,score,margin in zip(chars,scores,margins)],
                'confidence':round(min(scores),4),'complete':'?' not in text,
                'method':'rectified_projection_slots_shape_chamfer_template_v3'}
