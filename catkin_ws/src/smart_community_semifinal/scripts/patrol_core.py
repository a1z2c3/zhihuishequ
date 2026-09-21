#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Ordered route and entry policy using fresh, visually witnessed green onset."""
from __future__ import division, unicode_literals
import math
from semifinal_core import GreenGate,wrap_angle,front_clearance,braking_distance,footprint_corners
from runtime_compat import isfinite


class Patrol(object):
    def __init__(self,route,speed=.18):
        self.route=route;self.speed=speed;self.index=0;self.phase='travel'
        self.entered=None;self.frames=set();self.error=None
        self.progress_time=None;self.best_progress=None;self.progress_phase=None

    @property
    def target(self):return self.route[min(self.index,len(self.route)-1)]

    def record_frame(self,view,stamp,count,complete=True):
        if self.phase=='observe' and view==self.target['name'] and count>0 and complete and stamp>=self.entered:
            self.frames.add(stamp)

    def advance(self):
        self.index+=1;self.entered=None;self.frames=set()
        self.progress_time=None;self.best_progress=None;self.progress_phase=None
        self.phase='done' if self.index>=len(self.route) else 'travel'

    def step(self,pose,now,entry_ready=False):
        if self.phase in ('done','failed'):return (0.,0.)
        target=self.target;dx=target['xy'][0]-pose[0];dy=target['xy'][1]-pose[1]
        distance=math.hypot(dx,dy)
        if self.phase in ('travel','orient'):
            desired=math.atan2(dy,dx) if self.phase=='travel' else math.radians(target['yaw'])
            progress=distance+.15*abs(wrap_angle(desired-pose[2]))
            if self.progress_phase!=self.phase or self.best_progress is None or progress<self.best_progress-.006:
                self.progress_phase=self.phase;self.best_progress=progress;self.progress_time=now
            elif now-self.progress_time>25:
                self.phase='failed';self.error='motion_stalled:'+target['name'];return (0.,0.)
        if self.phase=='travel':
            if distance>.018:
                error=wrap_angle(math.atan2(dy,dx)-pose[2])
                if abs(error)>.045:return (0.,max(-.55,min(.55,1.5*error)))
                return (min(self.speed,max(.025,.8*distance)),max(-.12,min(.12,.8*error)))
            self.phase='orient'
        if self.phase=='orient':
            error=wrap_angle(math.radians(target['yaw'])-pose[2])
            if abs(error)>.025:return (0.,max(-.5,min(.5,1.5*error)))
            self.entered=now
            self.phase='gate' if target.get('gate') else 'observe' if target.get('observe') else 'settle'
        if self.phase=='gate':
            if entry_ready:self.advance()
        elif self.phase=='observe':
            if now-self.entered>=2.0 and len(self.frames)>=3:self.advance()
            elif now-self.entered>25:
                self.phase='failed';self.error='observation_timeout:'+target['name']
        elif self.phase=='settle' and now-self.entered>.2:self.advance()
        return (0.,0.)


class CrossingPolicy(object):
    """The configured green-duration lower bound is a simulation assumption.

    Observations authorize entry only after witnessing red/yellow -> green.
    Stale observations and insufficient clearance time cannot authorize entry.
    """
    def __init__(self):
        self.gate=GreenGate();self.stop=None;self.mode='unarmed';self.previous=None
        self.green_start=None;self.failure=None

    def arm(self,stop):
        if self.stop and self.stop['id']==stop['id']:return
        for field in ('point','direction','exit_point','wait_point'):
            if len(stop[field])!=2 or not all(isfinite(v) for v in stop[field]):raise ValueError(field)
        if math.hypot(*stop['direction'])<.99:raise ValueError('invalid direction')
        if not isfinite(stop['min_green_seconds']) or stop['min_green_seconds']<=0:raise ValueError('green bound')
        self.stop=stop;self.mode='approach';self.gate.reset();self.previous=None;self.green_start=None

    def signal(self,s,now):
        if self.stop is None or s.get('light_id')!=self.stop['id']:return
        stamp=s.get('stamp');state=s.get('state');score=s.get('confidence',0)
        valid=isfinite(stamp) and isfinite(score) and 0<=now-stamp<=.35 and score>=.7
        if not valid or state not in ('red','yellow','green'):
            self.green_start=None;self.previous=None;self.gate.reset();return
        if self.previous and stamp<=self.previous[1]:
            if stamp<self.previous[1]:self.green_start=None;self.previous=None;self.gate.reset()
            return
        if self.previous is None or stamp-self.previous[1]>.3:self.green_start=None
        elif state=='green' and self.previous[0] in ('red','yellow'):
            self.green_start=self.previous[1]
        elif state!='green':self.green_start=None
        self.previous=(state,stamp)
        self.gate.update(s['light_id'],state,score,stamp,now)

    def ready(self,now):
        if self.stop is None or self.green_start is None or not self.gate.allow(now):return False
        span=math.hypot(self.stop['exit_point'][0]-self.stop['wait_point'][0],self.stop['exit_point'][1]-self.stop['wait_point'][1])
        required=(span+.187+.06)/.16+1.0
        return self.stop['min_green_seconds']-(now-self.green_start)>=required

    def exited(self,pose):
        if self.stop is None:return True
        dx,dy=self.stop['direction'];x,y=self.stop['exit_point']
        return min((px-x)*dx+(py-y)*dy for px,py in footprint_corners(*pose))>.035

    def clear(self,pose=None):
        if self.stop is not None and (pose is None or not self.exited(pose)):return False
        self.stop=None;self.mode='clear';self.green_start=None;self.previous=None;self.gate.reset();return True

    def filter(self,pose,speed,omega,now):
        if self.mode=='unarmed':return 0.,0.
        if self.stop is not None:
            if self.mode=='crossing' and self.exited(pose):self.clear(pose)
            elif self.mode=='approach':
                clearance=front_clearance(pose,self.stop['point'],self.stop['direction'])
                if clearance<0:
                    self.failure='stop_line_crossed_before_permission';return 0.,0.
                if clearance<=braking_distance(speed)+.012:
                    if self.ready(now):self.mode='crossing'
                    else:return 0.,0.
        return speed,omega
