#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Overlay semantic rule zones on the measured SLAM map for move_base.

AMCL continues to use the measured ``slam_map``.  The derived map is only for
the global costmap, so lane/rule geometry is represented to Navfn without
pretending that painted boundaries are lidar observations.
"""
from __future__ import division,print_function,unicode_literals
import argparse
import io
import json
import os
import sys

HERE=os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0,HERE)
from plan_route import read_pgm,constrain_to_field,constrain_rule_zones


def build(layout,map_yaml,source,out_pgm):
    width,height,maxval,pixels=read_pgm(source)
    origin=map_yaml['origin'];resolution=float(map_yaml['resolution'])
    blocked=constrain_to_field([False]*(width*height),width,height,origin,
                               resolution,layout.get('field_size_m',[4.2,4.2]),0.)
    blocked=constrain_rule_zones(blocked,width,height,origin,resolution,layout)
    output=bytearray(pixels)
    for index,cell in enumerate(blocked):
        if cell:output[index]=0
    with open(out_pgm,'wb') as stream:
        stream.write(('P5\n%d %d\n%d\n'%(width,height,maxval)).encode('ascii'))
        stream.write(output)
    return width,height,sum(blocked)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--layout',required=True)
    parser.add_argument('--map-yaml',required=True);parser.add_argument('--map-pgm',required=True)
    parser.add_argument('--out-pgm',required=True);parser.add_argument('--out-yaml',required=True)
    args=parser.parse_args()
    with io.open(args.layout,encoding='utf-8') as stream:layout=json.load(stream)
    map_yaml={}
    with io.open(args.map_yaml,encoding='utf-8') as stream:
        for line in stream:
            if ':' not in line or line.lstrip().startswith('#'):continue
            key,value=line.split(':',1);key=key.strip();value=value.strip()
            if key=='origin':map_yaml[key]=json.loads(value)
            elif key in ('resolution','occupied_thresh','free_thresh'):map_yaml[key]=float(value)
            elif key=='negate':map_yaml[key]=int(value)
    width,height,blocked=build(layout,map_yaml,args.map_pgm,args.out_pgm)
    with open(args.out_yaml,'wb') as stream:
        lines=('image: %s\nresolution: %.6f\norigin: [%s]\n'
               'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n') % (
                   os.path.basename(args.out_pgm),float(map_yaml['resolution']),
                   ', '.join('%.6f'%float(v) for v in map_yaml['origin']))
        stream.write(lines.encode('ascii'))
    print('NAVIGATION_MAP_OK size=%dx%d semantic_occupied=%d'%(width,height,blocked))


if __name__=='__main__':main()
