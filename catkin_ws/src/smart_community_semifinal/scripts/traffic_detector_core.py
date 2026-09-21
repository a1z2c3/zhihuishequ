#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conservative light baseline: select bright coloured lamp, else unknown.

Use only a configured light-housing ROI acquired from visual detection/calibration.
Full-frame colour search is not safe in a scene containing coloured person cards.
"""
import cv2
import numpy as np


def detect_signal(frame, roi):
    x,y,w,h = [int(v) for v in roi]
    if x < 0 or y < 0 or w < 6 or h < 6 or x+w > frame.shape[1] or y+h > frame.shape[0]:
        return {"state":"unknown", "confidence":0.0, "bbox":[x,y,w,h]}
    crop = frame[y:y+h,x:x+w]
    hsv = cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    hue,sat,val = cv2.split(hsv)
    scores={}
    # Quantile is local to pixels with the correct hue, so a red shirt elsewhere
    # cannot increase the confidence of a green lamp.
    masks={"red":((hue <= 12)|(hue >= 170)),
           "yellow":((hue >= 16)&(hue <= 38)),
           "green":((hue >= 40)&(hue <= 95))}
    for name,mask in masks.items():
        mask &= (sat > 70)&(val > 185)
        mask = mask.astype(np.uint8)*255
        contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        best=0.0
        for contour in contours:
            area=cv2.contourArea(contour)
            if area < max(5,0.002*w*h): continue
            bx,by,bw,bh=cv2.boundingRect(contour)
            if not 0.35 < bw/float(bh) < 2.8: continue
            best=max(best,min(1.0,area/(w*h*0.03)))
        scores[name]=best
    ranked=sorted(scores,key=scores.get,reverse=True)
    winner=ranked[0]
    confidence=max(0.0,scores[winner]-scores[ranked[1]])
    if scores[winner] < 0.6 or confidence < 0.5:
        winner="unknown"
    if winner == "unknown":
        # Official LED lamps have overexposed white diode cores. Yellow contains
        # both orange and pale-yellow pixels, so competing hue masks can tie.
        # Only use this recovery when bright white cores are spatially inside
        # the lamp, not on exterior screws or reflective housing.
        yy,xx=np.ogrid[:h,:w]
        centre=((xx-w/2.0)/(w*.36))**2+((yy-h/2.0)/(h*.36))**2 <= 1
        valid=centre&(sat>100)&(val>185)
        white=centre&(sat<100)&(val>220)
        white_fraction=float(white.sum())/max(1,int(centre.sum()))
        if valid.sum() > max(8,.02*centre.sum()) and white_fraction > .012:
            values=hue[valid].astype(np.float32)
            values[values>170]-=180
            med=float(np.median(values))
            winner="red" if -10 <= med < 9 else "yellow" if 9 <= med <= 40 else "green" if 40 < med < 95 else "unknown"
            confidence=min(1.0,white_fraction/.04) if winner!="unknown" else 0.0
    return {"state":winner,"confidence":round(confidence,4),"bbox":[x,y,w,h]}
