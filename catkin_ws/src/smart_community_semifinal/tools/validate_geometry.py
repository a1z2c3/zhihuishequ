#!/usr/bin/env python3
"""Fail if actual observation poses cannot resolve front-facing complete targets.

This is a pinhole/SAT design test. Occlusion, texture appearance, ROS timing and
tracking error must still be measured in Gazebo. Reference inventory != instances.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PKG/'scripts'))
from scene_geometry import camera_from_urdf,visible_card,physical_obstacles,body_violation


def check(package):
    layout=json.loads((package/'config/layout.json').read_text(encoding='utf-8'))
    manifest=json.loads((package/'assets/manifest.json').read_text(encoding='utf-8'))
    assets={Path(r['file']).stem:r for r in manifest['recognition_assets']}
    instances=json.loads((package/'config/scene_instances_for_evaluation_only.json').read_text(encoding='utf-8'))
    camera=camera_from_urdf(str(package/'urdf/semifinal_bot.urdf'))
    world=ET.parse(str(package/'worlds/official_semifinal.world')).getroot()
    observations=[e for e in layout['route'] if e.get('observe')]
    targets=[];failures=[]
    for obj in instances:
        if obj['model'] not in assets:continue
        asset=assets[obj['model']];best=None
        for view in observations:
            pose=tuple(view['xy'])+(math.radians(view['yaw']),)
            result=visible_card(obj,asset['width_m'],asset['height_m'],pose,camera)
            if result is not None and (best is None or result['long_pixels']>best['long_pixels']):
                best=dict(result,view=view['name'])
        threshold=160 if asset['category']!='plate' else 180
        ok=best is not None and best['long_pixels']>=threshold
        targets.append({'instance':obj['name'],'label':asset['label'],'required_px':threshold,'best':best,'pass':ok})
        if not ok:failures.append(obj['name']+' has no sufficiently resolved front view')
    lights=[]
    for gate in [e for e in layout['route'] if e.get('gate')]:
        lamp=world.find(".//world/model[@name='%s']"%gate['gate'])
        p=[float(v) for v in lamp.findtext('pose').split()]
        obj={'x':p[0],'y':p[1],'z':p[2]+.34,'yaw':p[5]}
        # Require the entire housing, not just its centre, inside both FOVs.
        visible=visible_card(obj,.64,.14,tuple(gate['xy'])+(math.radians(gate['yaw']),),camera)
        lights.append({'id':gate['gate'],'at':gate['name'],'housing':visible,'pass':visible is not None})
        if visible is None:failures.append(gate['gate']+' housing clipped at waiting pose')
    obstacles=physical_obstacles(str(package/'worlds/official_semifinal.world'))
    poses=[]
    for i,e in enumerate(layout['route']):
        if i:
            previous=layout['route'][i-1];a=np.array(previous['xy']);b=np.array(e['xy'])
            direction=math.atan2(b[1]-a[1],b[0]-a[0]);n=max(2,int(np.linalg.norm(b-a)/.01)+1)
            poses.extend((float(x),float(y),direction) for x,y in np.linspace(a,b,n))
        poses.extend(tuple(e['xy'])+(angle,) for angle in np.linspace(-math.pi,math.pi,73))
    violations=[]
    for pose in poses:
        bad=body_violation(pose,layout,obstacles)
        if bad:violations.append({'pose':pose,'obstacle':bad})
    if violations:failures.append('%d sampled body intersections'%len(violations))
    return {'scope':'pinhole projection at observation poses + nominal inflated-body SAT; not Gazebo validation',
            'camera':camera,'targets':targets,'signals':lights,'physical_obstacles':obstacles,
            'sampled_poses':len(poses),'violations':violations,'failures':failures,'pass':not failures}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--package',type=Path,default=PKG)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    result=check(args.package);args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    for row in result['targets']:
        print(row['instance'], 'unobservable' if row['best'] is None else round(row['best']['long_pixels'],1),row['pass'])
    print('camera',result['camera']['width'],result['camera']['height'],'poses',result['sampled_poses'],
          'physical obstacles',len(result['physical_obstacles']),'failures',result['failures'])
    sys.exit(0 if result['pass'] else 1)
