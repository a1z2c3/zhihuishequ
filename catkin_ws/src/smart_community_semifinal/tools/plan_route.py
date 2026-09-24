#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Generate semantic patrol waypoints from the saved occupancy map.

This is a small, dependency-free A* planner for the teaching VM.  It uses the
same map origin/resolution as ``map_server`` and emits the existing route
schema: semantic observation/gate targets are retained, while intermediate
``plan_*`` poses are inserted between them.  Unknown map cells follow the
project's ``track_unknown_space: false`` contract and are treated as free;
black occupied cells are inflated by the configured clearance.
"""
from __future__ import division, print_function, unicode_literals
import argparse
import heapq
import io
import json
import math
import os


NEIGHBOURS=((1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),
            (1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),
            (-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2)))


def read_pgm(path):
    with open(path,'rb') as stream:data=stream.read()
    pos=0;tokens=[]
    while len(tokens)<4:
        end=data.find(b'\n',pos)
        if end<0:raise ValueError('truncated PGM header')
        line=data[pos:end].strip();pos=end+1
        if not line or line.startswith(b'#'):continue
        tokens.extend(line.split())
    if tokens[0]!=b'P5':raise ValueError('only binary P5 PGM is supported')
    width,height,maxval=[int(x) for x in tokens[1:4]]
    if maxval>255:raise ValueError('16-bit PGM is unsupported')
    pixels=data[pos:pos+width*height]
    if len(pixels)!=width*height:raise ValueError('truncated PGM data')
    return width,height,maxval,pixels


def world_to_grid(x,y,origin,resolution,width,height):
    col=int(round((x-origin[0])/resolution))
    row=height-1-int(round((y-origin[1])/resolution))
    return col,row


def grid_to_world(col,row,origin,resolution,width,height):
    return [origin[0]+col*resolution,
            origin[1]+(height-1-row)*resolution]


def constrain_to_field(blocked,width,height,origin,resolution,field,bound):
    """Keep planned robot centres inside the physical field footprint."""
    result=list(blocked)
    field_width,field_height=float(field[0]),float(field[1])
    for row in range(height):
        for col in range(width):
            x,y=grid_to_world(col,row,origin,resolution,width,height)
            if (x < bound or x > field_width-bound or
                    y < bound or y > field_height-bound):
                result[row*width+col]=True
    return result


def constrain_rule_zones(blocked,width,height,origin,resolution,layout):
    """Keep the planner out of rectangular decompositions of rule zones."""
    a=layout.get('a_polygon');b=layout.get('b_polygon')
    boxes=[]
    if a and len(a)>=5:
        boxes.extend(((a[0][0],a[-1][1],a[1][0],a[0][1]),
                      (a[4][0],a[3][1],a[2][0],a[4][1])))
    if b:
        xs=[point[0] for point in b];ys=[point[1] for point in b]
        boxes.append((min(xs),min(ys),max(xs),max(ys)))
    if 'parking_boundary_x' in layout:
        field=layout.get('field_size_m',[4.2,4.2])
        boxes.append((float(layout['parking_boundary_x']),0.,float(field[0]),
                      float(layout.get('parking_open_above_y',field[1]))))
    result=list(blocked)
    for row in range(height):
        for col in range(width):
            x,y=grid_to_world(col,row,origin,resolution,width,height)
            if any(x1<=x<=x2 and y1<=y<=y2 for x1,y1,x2,y2 in boxes):
                result[row*width+col]=True
    return result


def inflate(width,height,pixels,radius_cells,occupied_threshold=100,negate=False):
    occupied=[False]*(width*height)
    for row in range(height):
        for col in range(width):
            value=pixels[row*width+col]
            if not isinstance(value,int):value=ord(value)
            if negate:
                value=255-value
            occupied[row*width+col]=(value<occupied_threshold)
    if radius_cells<=0:return occupied
    result=list(occupied);r=radius_cells
    for row in range(height):
        for col in range(width):
            if not occupied[row*width+col]:continue
            for dr in range(-r,r+1):
                for dc in range(-r,r+1):
                    if dc*dc+dr*dr>r*r:continue
                    rr,cc=row+dr,col+dc
                    if 0<=rr<height and 0<=cc<width:result[rr*width+cc]=True
    return result


def nearest_free(cell,blocked,width,height,max_radius=30):
    col,row=cell
    if 0<=col<width and 0<=row<height and not blocked[row*width+col]:return cell
    for radius in range(1,max_radius+1):
        candidates=[]
        for dr in range(-radius,radius+1):
            for dc in range(-radius,radius+1):
                if max(abs(dc),abs(dr))!=radius:continue
                cc,rr=col+dc,row+dr
                if 0<=cc<width and 0<=rr<height and not blocked[rr*width+cc]:
                    candidates.append((cc,rr))
        if candidates:return min(candidates,key=lambda p:(p[0]-col)**2+(p[1]-row)**2)
    raise ValueError('no free cell near %r'%((col,row),))


def astar(start,goal,blocked,width,height):
    start=nearest_free(start,blocked,width,height);goal=nearest_free(goal,blocked,width,height)
    frontier=[(0.0,start)];cost={start:0.0};came={}
    while frontier:
        _,current=heapq.heappop(frontier)
        if current==goal:
            path=[current]
            while current in came:current=came[current];path.append(current)
            return list(reversed(path)),start,goal
        for dc,dr,step in NEIGHBOURS:
            nxt=(current[0]+dc,current[1]+dr)
            if not (0<=nxt[0]<width and 0<=nxt[1]<height):continue
            if blocked[nxt[1]*width+nxt[0]]:continue
            # Prevent diagonal corner cutting through two occupied cells.
            if dc and (blocked[current[1]*width+nxt[0]] or
                       blocked[nxt[1]*width+current[0]]):continue
            value=cost[current]+step
            if value<cost.get(nxt,float('inf')):
                cost[nxt]=value
                h=math.hypot(goal[0]-nxt[0],goal[1]-nxt[1])
                heapq.heappush(frontier,(value+h,nxt));came[nxt]=current
    raise ValueError('no map path from %r to %r'%(start,goal))


def simplify(path):
    if len(path)<3:return path
    result=[path[0]];last=(path[1][0]-path[0][0],path[1][1]-path[0][1])
    for i in range(1,len(path)-1):
        direction=(path[i+1][0]-path[i][0],path[i+1][1]-path[i][1])
        if direction!=last:result.append(path[i])
        last=direction
    result.append(path[-1]);return result


def plan(layout,map_yaml,pgm_path,clearance,max_snap=0.12,robot_radius=0.0):
    width,height,maxval,pixels=read_pgm(pgm_path)
    resolution=float(map_yaml['resolution']);origin=map_yaml['origin']
    if len(origin)<3 or abs(float(origin[2]))>1e-9:
        raise ValueError('A* planner requires an axis-aligned map origin')
    occupied_thresh=float(map_yaml.get('occupied_thresh',.65))
    negate=str(map_yaml.get('negate',0)).strip().lower() in ('1','true','yes')
    occupied_threshold=max(0.,min(float(maxval),float(maxval)*(1.-occupied_thresh)))
    inflation=clearance+max(0.,robot_radius)
    blocked=inflate(width,height,pixels,int(math.ceil(inflation/resolution)),
                    occupied_threshold,negate)
    field=layout.get('field_size_m',[4.2,4.2])
    blocked=constrain_to_field(blocked,width,height,origin,resolution,field,inflation)
    blocked=constrain_rule_zones(blocked,width,height,origin,resolution,layout)
    original=layout['route'];route=[dict(original[0])];stats=[]
    for index,(source,target) in enumerate(zip(original,original[1:])):
        start=world_to_grid(source['xy'][0],source['xy'][1],origin,resolution,width,height)
        goal=world_to_grid(target['xy'][0],target['xy'][1],origin,resolution,width,height)
        raw_path,actual_start,actual_goal=astar(start,goal,blocked,width,height)
        start_snap=math.hypot(actual_start[0]-start[0],actual_start[1]-start[1])*resolution
        goal_snap=math.hypot(actual_goal[0]-goal[0],actual_goal[1]-goal[1])*resolution
        if start_snap>max_snap or goal_snap>max_snap:
            raise ValueError('semantic waypoint is occupied or too close to an obstacle: %s -> %s'
                             %(source['name'],target['name']))
        raw_cells=len(raw_path)
        path=simplify(raw_path)
        inserted=[]
        for n,cell in enumerate(path[1:-1],1):
            xy=grid_to_world(cell[0],cell[1],origin,resolution,width,height)
            next_cell=path[n+1]
            next_xy=grid_to_world(next_cell[0],next_cell[1],origin,resolution,width,height)
            angle=math.degrees(math.atan2(next_xy[1]-xy[1],next_xy[0]-xy[0]))
            inserted.append({'name':'plan_%02d_%03d'%(index,n),'xy':xy,'yaw':angle,
                             'planner':'astar','segment':index})
        route.extend(inserted);route.append(dict(target))
        stats.append({'segment':index,'from':source['name'],'to':target['name'],
                      'grid_cells':raw_cells,'simplified_cells':len(path),
                      'inserted':len(inserted),
                      'start_cell':actual_start,'goal_cell':actual_goal})
    output=dict(layout);output['route']=route
    output['lane_centerline']=[entry['xy'] for entry in route]
    output['planner']={'algorithm':'A*','map_resolution_m':resolution,
                       'inflation_clearance_m':clearance,
                       'robot_radius_m':robot_radius,
                       'inflation_radius_m':clearance+max(0.,robot_radius),
                       'field_boundary_constrained':True,
                       'rule_zones_constrained':True,
                       'segments':stats}
    return output


def write_route_json(path,route):
    rendered=json.dumps(route,ensure_ascii=False,indent=2)
    if not isinstance(rendered,type(u'')):
        rendered=rendered.decode('utf-8')
    with open(path,'wb') as stream:
        stream.write((rendered+u'\n').encode('utf-8'))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--layout',required=True)
    parser.add_argument('--map-yaml',required=True);parser.add_argument('--map-pgm',required=True)
    parser.add_argument('--out',required=True)
    parser.add_argument('--clearance',type=float,default=.04)
    parser.add_argument('--max-snap',type=float,default=.12)
    parser.add_argument('--robot-radius',type=float,default=.20)
    args=parser.parse_args()
    with io.open(args.layout,encoding='utf-8') as stream:layout=json.load(stream)
    with io.open(args.map_yaml,encoding='utf-8') as stream:
        map_yaml={}
        for line in stream:
            if ':' not in line or line.lstrip().startswith('#'):continue
            key,value=line.split(':',1);value=value.strip()
            if key.strip()=='origin':map_yaml[key.strip()]=json.loads(value.replace('[','[').replace(']',']'))
            elif key.strip() in ('resolution','occupied_thresh','free_thresh'):
                map_yaml[key.strip()]=float(value)
            elif key.strip() in ('negate',):
                map_yaml[key.strip()]=int(value)
    result=plan(layout,map_yaml,args.map_pgm,args.clearance,args.max_snap,args.robot_radius)
    write_route_json(args.out,result)
    print('A*_ROUTE_OK targets=%d planned_waypoints=%d segments=%d'%(len(layout['route']),len(result['route']),len(result['planner']['segments'])))


if __name__=='__main__':main()
