#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS image decoding and metric position from known-size planar image targets."""
from __future__ import division, unicode_literals
import cv2,numpy as np


def decode_image(msg):
    if msg.encoding not in ('bgr8','rgb8'):raise ValueError('unsupported image encoding')
    raw=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
    result=raw[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
    return cv2.cvtColor(result,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else result


def planar_position(detection,K):
    w,h=detection['width_m'],detection['height_m']
    objects=np.float64([[-w/2,-h/2,0],[w/2,-h/2,0],[w/2,h/2,0],[-w/2,h/2,0]])
    image=np.float64(detection['quad']);camera=np.float64(K).reshape(3,3)
    if detection.get('pose_correspondences'):
        pairs=detection['pose_correspondences']
        objects=np.float64(pairs['object']);image=np.float64(pairs['image'])
        if len(objects)<6 or objects.shape!=(len(image),3):return None
    ok,rv,tv=cv2.solvePnP(objects,image,camera,np.zeros(5),flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(tv).all() or not .12<tv[2,0]<4:return None
    reproj,_=cv2.projectPoints(objects,rv,tv,camera,np.zeros(5))
    error=float(np.sqrt(np.mean(np.sum((reproj.reshape(-1,2)-image)**2,axis=1))))
    if error>3:return None
    return {'camera_xyz':tv.ravel().tolist(),'reprojection_rmse_px':error,
            'method':'known_dimension_planar_pnp','points':len(objects),'scale_requires_material_calibration':True}
