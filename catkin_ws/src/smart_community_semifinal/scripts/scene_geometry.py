#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Camera projection and body/obstacle checks, independent of ROS and truth topics."""
from __future__ import division, unicode_literals
import math
import xml.etree.ElementTree as ET
import numpy as np
from semifinal_core import footprint_corners


def camera_from_urdf(path):
    root=ET.parse(path).getroot()
    camera=root.find(".//gazebo[@reference='camera_link']/sensor/camera")
    if camera is None:raise ValueError('camera_link must contain a calibrated camera')
    width=int(camera.findtext('image/width'));height=int(camera.findtext('image/height'))
    hfov=float(camera.findtext('horizontal_fov'));fx=width/(2*math.tan(hfov/2))
    by_child={j.find('child').get('link'):j for j in root.findall('joint')}
    xyz=np.zeros(3);name='camera_link'
    while name!='base_footprint':
        joint=by_child[name];origin=joint.find('origin')
        if origin is not None:
            assert all(abs(float(v))<1e-7 for v in origin.get('rpy','0 0 0').split()), 'Extend projection for a tilted camera'
            xyz+=np.array([float(v) for v in origin.get('xyz','0 0 0').split()])
        name=joint.find('parent').get('link')
    return {'width':width,'height':height,'fx':fx,'fy':fx,'cx':width/2,'cy':height/2,
            'xyz':xyz.tolist(),'hfov':hfov,'vfov':2*math.atan(height/(2*fx)),
            'near':float(camera.findtext('clip/near'))}


def camera_position(pose,camera):
    x,y,yaw=pose[:3];z=pose[3] if len(pose)>3 else .003
    a,b,c=camera['xyz'];co,si=math.cos(yaw),math.sin(yaw)
    return np.array([x+co*a-si*b,y+si*a+co*b,z+c])


def project(points,pose,camera):
    delta=np.asarray(points)-camera_position(pose,camera)
    co,si=math.cos(pose[2]),math.sin(pose[2])
    forward=delta[:,0]*co+delta[:,1]*si
    left=-delta[:,0]*si+delta[:,1]*co
    if np.any(forward<=camera['near']):return None
    return np.column_stack((camera['cx']-camera['fx']*left/forward,
                            camera['cy']-camera['fy']*delta[:,2]/forward))


def card_corners(x,y,z,yaw,width,height):
    # Source image corners in top-left, top-right, bottom-right, bottom-left order.
    # Card front normal is (sin(yaw), -cos(yaw), 0).
    co,si=math.cos(yaw),math.sin(yaw)
    return np.array([[x+co*u,y+si*u,z+v] for u,v in
                     [(-width/2,height),(width/2,height),(width/2,0),(-width/2,0)]])


def visible_card(obj,width,height,pose,camera,pad=8):
    cp=camera_position(pose,camera)
    direction=cp[:2]-[obj['x'],obj['y']]
    facing=float(np.dot(direction,[math.sin(obj['yaw']),-math.cos(obj['yaw'])]))/max(1e-8,np.linalg.norm(direction))
    quad=project(card_corners(obj['x'],obj['y'],obj.get('z',.003),obj['yaw'],width,height),pose,camera)
    if quad is None or facing<.45:return None
    if np.any(quad<pad) or np.any(quad[:,0]>camera['width']-pad) or np.any(quad[:,1]>camera['height']-pad):return None
    edges=np.linalg.norm(quad-np.roll(quad,1,axis=0),axis=1)
    return {'quad':quad.tolist(),'long_pixels':float(max((edges[0]+edges[2])/2,(edges[1]+edges[3])/2)),
            'front_cosine':facing,'minimum_border_px':float(min(quad.min(),camera['width']-quad[:,0].max(),camera['height']-quad[:,1].max()))}


def polygons_overlap(a,b):
    a=np.asarray(a);b=np.asarray(b)
    for polygon in (a,b):
        for edge in np.roll(polygon,-1,axis=0)-polygon:
            axis=np.array([-edge[1],edge[0]])
            pa=np.dot(a,axis);pb=np.dot(b,axis)
            if pa.max()<pb.min() or pb.max()<pa.min():return False
    return True


def rule_boxes(layout):
    # Decompose the concave L-shaped A island for a valid convex SAT check.
    a=layout['a_polygon'];b=np.asarray(layout['b_polygon'])
    return [(a[0][0],a[-1][1],a[1][0],a[0][1]),(a[4][0],a[3][1],a[2][0],a[4][1]),
            (float(b[:,0].min()),float(b[:,1].min()),float(b[:,0].max()),float(b[:,1].max())),
            (layout['parking_boundary_x'],0,4.2,layout['parking_open_above_y'])]


def rectangle(x1,y1,x2,y2):return [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]


def physical_obstacles(world_path,robot_height=.24):
    root=ET.parse(world_path).getroot();obstacles=[]
    for model in root.findall('.//world/model'):
        pose=[float(v) for v in model.findtext('pose','0 0 0 0 0 0').split()]
        for collision in model.findall('.//collision'):
            size=collision.findtext('geometry/box/size')
            if size is None:continue
            dims=[float(v) for v in size.split()]
            local=[float(v) for v in collision.findtext('pose','0 0 0 0 0 0').split()]
            if pose[2]+local[2]-dims[2]/2>robot_height:continue
            co,si=math.cos(pose[5]),math.sin(pose[5])
            x=pose[0]+co*local[0]-si*local[1];y=pose[1]+si*local[0]+co*local[1]
            poly=footprint_corners(x,y,pose[5]+local[5],dims[0],dims[1],0)
            obstacles.append({'name':model.get('name')+'/'+collision.get('name'),'polygon':poly})
    return obstacles


def body_violation(pose,layout,obstacles=()):
    body=footprint_corners(*pose[:3])
    if any(x<0 or y<0 or x>4.2 or y>4.2 for x,y in body):return 'field_boundary'
    for i,box in enumerate(rule_boxes(layout)):
        if polygons_overlap(body,rectangle(*box)):return 'rule_zone_%d'%i
    for item in obstacles:
        if polygons_overlap(body,item['polygon']):return item['name']
    return None
