#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Locate a lit circular lamp and verify its dark horizontal housing in pixels.

The projected static installation only limits search; no simulator lamp state is
read. A failed/ambiguous visual candidate never becomes a permissive signal.
"""
from __future__ import division, unicode_literals
import math
import cv2,numpy as np


def locate_signal(frame,hint,geometry=None):
    if hint is None:return None
    x,y,w,h=[float(v) for v in hint]
    xa=max(0,int(x-.2*w));ya=max(0,int(y-.45*h))
    xb=min(frame.shape[1],int(x+1.2*w));yb=min(frame.shape[0],int(y+1.45*h))
    if xb-xa<12 or yb-ya<12:return None
    hint_cx=x+w/2.;hint_cy=y+h/2.
    crop=frame[ya:yb,xa:xb];hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    hue,sat,val=cv2.split(hsv)
    # Yellow LEDs can be nearly white after Gazebo/VM rendering.  Keep a
    # bounded saturation floor; the dark housing and ambiguity checks below
    # remain mandatory, so this is not a full-frame colour shortcut.
    mask=(((hue<=16)|(hue>=170)|((hue>=14)&(hue<95)))&
          (sat>70)&(val>150)).astype(np.uint8)*255
    candidates=[]
    for contour in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]:
        area=cv2.contourArea(contour);bx,by,bw,bh=cv2.boundingRect(contour)
        if area<18 or not .55<bw/float(bh)<1.6:continue
        if not .45<area/float(bw*bh)<.92:continue
        component=np.zeros(mask.shape,np.uint8);cv2.drawContours(component,[contour],-1,255,-1)
        values=hue[component>0].astype(np.float32);values[values>169]-=180
        median=float(np.median(values));colour='red' if median<14 else 'yellow' if median<40 else 'green'
        diameter=(bw+bh)/2.;cx=xa+bx+bw/2.;cy=ya+by+bh/2.
        geometry=geometry or {}
        offset=float(geometry.get('active_lamp_offset_diameter',2.115))
        rw_ratio=float(geometry.get('housing_width_diameter',6.25))
        rh_ratio=float(geometry.get('housing_height_diameter',1.55))
        centre=cx+offset*diameter if colour=='red' else cx-offset*diameter if colour=='green' else cx
        rw=rw_ratio*diameter;rh=rh_ratio*diameter
        rx=int(centre-rw/2);ry=int(cy-rh/2);rw=int(rw);rh=int(rh)
        if rx<0 or ry<0 or rx+rw>=frame.shape[1] or ry+rh>=frame.shape[0]:continue
        # AMCL/map projection can be off by several pixels at the approach
        # waypoint.  Keep the active-stop projection as the search prior, but
        # tolerate a bounded error rather than dropping every visual frame.
        if abs(centre-hint_cx)>max(45,.65*w) or abs(cy-hint_cy)>max(32,.75*h):continue
        grey=cv2.cvtColor(frame[ry:ry+rh,rx:rx+rw],cv2.COLOR_BGR2GRAY)
        dark=float((grey<85).sum())/grey.size
        if dark<.45:continue
        distance=math.hypot((centre-hint_cx)/max(1.,w),
                            (cy-hint_cy)/max(1.,h))
        candidates.append({'roi':[rx,ry,rw,rh],'colour_proposal':colour,
                           'dark_housing_fraction':dark,'area':area,
                           'hint_distance':float(distance)})
    if not candidates:return None
    candidates.sort(key=lambda item:(item['hint_distance'],
                                     -item['dark_housing_fraction'],
                                     -item['area']))
    # A nearest candidate is safe only when it is meaningfully closer than the
    # runner-up.  Ambiguous visual evidence remains unknown and is handled by
    # the traffic fail-safe instead of guessing between lamps.
    if (len(candidates)>1 and
            candidates[1]['hint_distance']-candidates[0]['hint_distance']<.08):
        return None
    return candidates[0]
