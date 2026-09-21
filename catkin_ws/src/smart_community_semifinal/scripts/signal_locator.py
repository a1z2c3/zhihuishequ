#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Locate a lit circular lamp and verify its dark horizontal housing in pixels.

The projected static installation only limits search; no simulator lamp state is
read. A failed/ambiguous visual candidate never becomes a permissive signal.
"""
from __future__ import division, unicode_literals
import cv2,numpy as np


def locate_signal(frame,hint):
    if hint is None:return None
    x,y,w,h=[float(v) for v in hint]
    xa=max(0,int(x-.2*w));ya=max(0,int(y-.45*h))
    xb=min(frame.shape[1],int(x+1.2*w));yb=min(frame.shape[0],int(y+1.45*h))
    if xb-xa<12 or yb-ya<12:return None
    crop=frame[ya:yb,xa:xb];hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    hue,sat,val=cv2.split(hsv)
    mask=(((hue<14)|(hue>169)|((hue>16)&(hue<95)))&(sat>95)&(val>150)).astype(np.uint8)*255
    candidates=[]
    for contour in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]:
        area=cv2.contourArea(contour);bx,by,bw,bh=cv2.boundingRect(contour)
        if area<18 or not .55<bw/float(bh)<1.6:continue
        if not .45<area/float(bw*bh)<.92:continue
        component=np.zeros(mask.shape,np.uint8);cv2.drawContours(component,[contour],-1,255,-1)
        values=hue[component>0].astype(np.float32);values[values>169]-=180
        median=float(np.median(values));colour='red' if median<14 else 'yellow' if median<40 else 'green'
        diameter=(bw+bh)/2.;cx=xa+bx+bw/2.;cy=ya+by+bh/2.
        centre=cx+2.115*diameter if colour=='red' else cx-2.115*diameter if colour=='green' else cx
        rw=6.25*diameter;rh=1.55*diameter
        rx=int(centre-rw/2);ry=int(cy-rh/2);rw=int(rw);rh=int(rh)
        if rx<0 or ry<0 or rx+rw>=frame.shape[1] or ry+rh>=frame.shape[0]:continue
        if abs(centre-(x+w/2))>max(25,.30*w) or abs(cy-(y+h/2))>max(20,.5*h):continue
        grey=cv2.cvtColor(frame[ry:ry+rh,rx:rx+rw],cv2.COLOR_BGR2GRAY)
        dark=float((grey<85).sum())/grey.size
        if dark<.45:continue
        candidates.append({'roi':[rx,ry,rw,rh],'colour_proposal':colour,'dark_housing_fraction':dark,'area':area})
    if len(candidates)!=1:return None
    return candidates[0]
