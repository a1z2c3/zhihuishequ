#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Camera projection and body/obstacle checks, independent of ROS and truth topics."""
from __future__ import division, unicode_literals
import math
import os
import xml.etree.ElementTree as ET
import numpy as np
from semifinal_core import (footprint_corners, BODY_LENGTH, BODY_WIDTH,
                            FOOTPRINT_MARGIN, integrate_twist_pose)
from runtime_compat import isfinite


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
    boxes=[]
    a=layout.get('a_polygon')
    b=layout.get('b_polygon')
    field=layout.get('field_size_m',[4.2,4.2])
    width,height=float(field[0]),float(field[1])
    if a:
        boxes.extend([(a[0][0],a[-1][1],a[1][0],a[0][1]),
                      (a[4][0],a[3][1],a[2][0],a[4][1])])
    if b:
        b=np.asarray(b)
        boxes.append((float(b[:,0].min()),float(b[:,1].min()),
                      float(b[:,0].max()),float(b[:,1].max())))
    if 'parking_boundary_x' in layout and 'parking_open_above_y' in layout:
        boxes.append((layout['parking_boundary_x'],0,width,
                      layout['parking_open_above_y']))
    return boxes


def rectangle(x1,y1,x2,y2):return [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]


def _pose(text):
    values=[float(v) for v in (text or '0 0 0 0 0 0').split()]
    if len(values)!=6:raise ValueError('SDF pose must have six values')
    return values


def _compose(parent,local):
    """Compose planar SDF poses, including link/collision offsets."""
    co,si=math.cos(parent[5]),math.sin(parent[5])
    return [parent[0]+co*local[0]-si*local[1],
            parent[1]+si*local[0]+co*local[1],
            parent[2]+local[2],0.,0.,parent[5]+local[5]]


def _collision_obstacles(model,model_pose,name,robot_height):
    for link in model.findall('./link'):
        link_pose=_pose(link.findtext('pose'))
        for collision in link.findall('.//collision'):
            size=collision.findtext('geometry/box/size')
            if size is None:continue
            dims=[float(v) for v in size.split()]
            if len(dims)!=3 or min(dims)<=0:continue
            collision_pose=_compose(model_pose,
                                    _compose(link_pose,_pose(collision.findtext('pose'))))
            # Ignore only geometry entirely above the robot body.
            if collision_pose[2]-dims[2]/2>robot_height:continue
            poly=footprint_corners(collision_pose[0],collision_pose[1],
                                   collision_pose[5],dims[0],dims[1],0)
            obstacles_name=name+'/'+(collision.get('name') or 'collision')
            yield {'name':obstacles_name,'polygon':poly}


def physical_obstacles(world_path,robot_height=.24):
    """Return low box collisions from inline and ``model://`` world models."""
    root=ET.parse(world_path).getroot();obstacles=[]
    world=root.find('./world')
    if world is None:return obstacles
    for model in world.findall('./model'):
        obstacles.extend(_collision_obstacles(model,_pose(model.findtext('pose')),
                                               model.get('name') or 'model',robot_height))
    models_dir=os.path.normpath(os.path.join(os.path.dirname(world_path),'..','models'))
    for include in world.findall('./include'):
        uri=(include.findtext('uri') or '').strip()
        if not uri.startswith('model://'):continue
        model_id=uri[len('model://'):].strip('/')
        if not model_id or '/' in model_id:continue
        sdf_path=os.path.join(models_dir,model_id,'model.sdf')
        if not os.path.isfile(sdf_path):continue
        try:
            model_root=ET.parse(sdf_path).getroot()
            model=model_root.find('./model')
            if model is None:continue
            model_pose=_compose(_pose(include.findtext('pose')),
                                _pose(model.findtext('pose')))
            instance=include.findtext('name') or model.get('name') or model_id
            obstacles.extend(_collision_obstacles(model,model_pose,instance,robot_height))
        except (IOError,OSError,ET.ParseError,ValueError):
            continue
    return obstacles


def _lane_segments(layout):
    """Return non-degenerate centre-line segments as ``(a, b, length)``."""
    centerline=layout.get('lane_centerline')
    if centerline is None:
        centerline=[item['xy'] for item in layout.get('route',())]
    points=[(float(point[0]),float(point[1])) for point in centerline]
    segments=[]
    for a,b in zip(points[:-1],points[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if length>1e-9:segments.append((a,b,length))
    if not segments and points:
        segments=[(points[0],points[0],0.)]
    return segments


def _lane_strip_excess(point,segment,allowed,extension):
    """Distance excess for a finite strip around one centre-line segment.

    The strip is extended at both ends by the footprint's turn radius.  This
    makes adjacent strips overlap at a corner, while points well beyond a
    route endpoint still use the endpoint distance check below.
    """
    a,b,length=segment
    if length<=1e-9:return math.hypot(point[0]-a[0],point[1]-a[1])-allowed
    dx,dy=b[0]-a[0],b[1]-a[1]
    projection=((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length
    lateral=abs((point[0]-a[0])*dy-(point[1]-a[1])*dx)/length
    if -extension<=projection<=length+extension:
        return lateral-allowed
    endpoint=a if projection<0. else b
    return math.hypot(point[0]-endpoint[0],point[1]-endpoint[1])-allowed


def lane_corridor_error(pose,layout):
    """Maximum footprint-corner excess beyond the route's drivable corridor.

    A route is a union of oriented strips, not a circular tube around the
    polyline.  Testing every footprint corner against that union preserves
    clearance on straight segments and avoids falsely rejecting a rotated
    footprint at a ninety-degree turn.
    """
    segments=_lane_segments(layout)
    if not segments:return 0.0
    half_width=float(layout.get('lane_width_m',0.0))/2.0
    reserve=float(layout.get('lane_safety_margin_m',.02))
    allowed=half_width-reserve
    if allowed<=0:raise ValueError('lane corridor has no positive footprint clearance')
    half_length=BODY_LENGTH/2.0+FOOTPRINT_MARGIN
    half_body_width=BODY_WIDTH/2.0+FOOTPRINT_MARGIN
    turn_extension=float(layout.get('lane_turn_extension_m',
                                     math.hypot(half_length,half_body_width)))
    if not isfinite(turn_extension) or turn_extension<0:
        raise ValueError('invalid lane turn extension')
    return max(min(_lane_strip_excess(corner,segment,allowed,turn_extension)
                   for segment in segments)
               for corner in footprint_corners(*pose[:3]))


def lane_recovery_vector(pose,layout):
    """Map-frame vector from the robot centre to the closest lane point."""
    segments=_lane_segments(layout)
    if not segments:return (0.,0.)
    x,y=pose[:2];best=None
    for a,b,length in segments:
        if length<=1e-9:
            q=a
        else:
            dx,dy=b[0]-a[0],b[1]-a[1]
            t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(length*length)))
            q=(a[0]+t*dx,a[1]+t*dy)
        distance=(q[0]-x)*(q[0]-x)+(q[1]-y)*(q[1]-y)
        if best is None or distance<best[0]:best=(distance,q)
    return (best[1][0]-x,best[1][1]-y)


def body_violation(pose,layout,obstacles=(),include_lane=True):
    body=footprint_corners(*pose[:3])
    field=layout.get('field_size_m',[4.2,4.2])
    width,height=float(field[0]),float(field[1])
    if any(x<0 or y<0 or x>width or y>height for x,y in body):return 'field_boundary'
    for i,box in enumerate(rule_boxes(layout)):
        if polygons_overlap(body,rectangle(*box)):return 'rule_zone_%d'%i
    for item in obstacles:
        if polygons_overlap(body,item['polygon']):return item['name']
    if include_lane and 'lane_width_m' in layout:
        if lane_corridor_error(pose,layout)>1e-7:return 'lane_corridor'
    return None


def lane_recovery_allowed(current_pose,projected_poses,layout,obstacles=()):
    """Permit only monotonic inward recovery from a lane-only violation."""
    if body_violation(current_pose,layout,obstacles)!='lane_corridor':return False
    initial=lane_corridor_error(current_pose,layout)
    previous=initial
    for pose in projected_poses:
        if body_violation(pose,layout,obstacles,include_lane=False):return False
        error=lane_corridor_error(pose,layout)
        if error>previous+1e-6:return False
        previous=error
    return previous<initial-1e-5


def trajectory_violation(current_pose,projected_poses,layout,obstacles=()):
    """Validate sampled swept-footprint poses, allowing only inward recovery."""
    violations=[body_violation(pose,layout,obstacles) for pose in projected_poses]
    bad=[value for value in violations if value]
    if not bad:return None
    if (set(bad)=={'lane_corridor'} and
            lane_recovery_allowed(current_pose,projected_poses,layout,obstacles)):
        return None
    return bad[0]


def sampled_twist_poses(pose,speed,lateral,omega,horizon=.25):
    return [integrate_twist_pose(pose,speed,lateral,omega,
                                 horizon*fraction)
            for fraction in (.2,.4,.6,.8,1.0)]


def lane_limited_twist(pose,speed,lateral,omega,layout,obstacles=(),horizon=.25):
    """Attenuate a predicted lane exit without hiding other violations.

    Returns ``(speed, lateral, omega, violation, limited)``.  The current pose
    must already be valid; an existing lane violation is handled by the
    explicit recovery state instead of this forward-command limiter.
    """
    projected=sampled_twist_poses(pose,speed,lateral,omega,horizon)
    violation=trajectory_violation(pose,projected,layout,obstacles)
    if violation!='lane_corridor' or body_violation(pose,layout,obstacles):
        return speed,lateral,omega,violation,False
    candidates=[(speed,lateral*.5,omega),
                (speed,0.,omega),
                (speed*.5,0.,omega*.5),
                (0.,0.,0.)]
    for candidate in candidates:
        projected=sampled_twist_poses(pose,candidate[0],candidate[1],
                                      candidate[2],horizon)
        if trajectory_violation(pose,projected,layout,obstacles) is None:
            return candidate[0],candidate[1],candidate[2],None,True
    return speed,lateral,omega,violation,False
