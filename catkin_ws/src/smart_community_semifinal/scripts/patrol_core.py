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
        # Avoidance is deliberately bounded by the lane geometry.  With a
        # 0.303 m body, 0.020 m footprint margin and a 0.60 m lane, a 0.09 m
        # shift leaves about 0.038 m to the lane edge.
        self.avoid_shift=.09;self.avoid_speed=.08;self.avoid_side=None
        self.avoid_start_pose=None;self.avoid_min_until=None
        self.obstacle_frames=0;self.was_blocked=False

    @property
    def target(self):return self.route[min(self.index,len(self.route)-1)]

    def record_frame(self,view,stamp,count,complete=True):
        if self.phase=='observe' and view==self.target['name'] and count>0 and complete and stamp>=self.entered:
            self.frames.add(stamp)

    def advance(self):
        self.index+=1;self.entered=None;self.frames=set()
        self.progress_time=None;self.best_progress=None;self.progress_phase=None
        self.avoid_side=None;self.avoid_start_pose=None;self.avoid_min_until=None
        self.obstacle_frames=0;self.was_blocked=False
        self.phase='done' if self.index>=len(self.route) else 'travel'

    def _lateral_progress(self,pose):
        if self.avoid_start_pose is None:return 0.
        sx,sy,syaw=self.avoid_start_pose
        dx,dy=pose[0]-sx,pose[1]-sy
        return -dx*math.sin(syaw)+dy*math.cos(syaw)

    def _reset_motion_progress(self,now,phase):
        self.progress_phase=phase;self.best_progress=None;self.progress_time=now

    def step(self,pose,now,entry_ready=False,guard=None):
        """Return body-frame ``(linear.x, linear.y, angular.z)``.

        The guard supplies only sensor-derived facts.  A centered obstacle
        with no legal side corridor pauses the stall watchdog; it is a normal
        waiting condition and must not turn into ``motion_stalled``.  A side
        maneuver is complete only after the scan reports the obstacle gone and
        the robot has returned to its pre-maneuver lateral track.
        """
        if self.phase in ('done','failed'):return (0.,0.,0.)
        guard=guard or {}
        forward=bool(guard.get('forward_obstacle',False))
        obstacle_ahead=bool(guard.get('obstacle_ahead',forward))
        left_free=bool(guard.get('left_free',False))
        right_free=bool(guard.get('right_free',False))
        blocked=forward and not left_free and not right_free
        target=self.target;dx=target['xy'][0]-pose[0];dy=target['xy'][1]-pose[1]
        distance=math.hypot(dx,dy)

        # Three consistent observations avoid triggering on a single noisy
        # scan.  A blocked center is handled as a wait, not as a side choice.
        if self.phase in ('travel','orient'):
            if forward and left_free != right_free:
                self.obstacle_frames+=1
            elif not forward:
                self.obstacle_frames=0
            if self.obstacle_frames>=3:
                self.phase='avoid';self.avoid_side='left' if left_free else 'right'
                self.avoid_start_pose=(pose[0],pose[1],pose[2])
                self.avoid_min_until=now+.30;self.obstacle_frames=0
                self._reset_motion_progress(now,'avoid')

        # A center-blocked obstacle is a safety stop.  Keep resetting the
        # progress clock while it remains present so removal lets patrol
        # resume instead of failing after the old 25/40-second watchdog.
        if blocked and self.phase in ('travel','orient'):
            self.was_blocked=True;self.progress_time=now;self.best_progress=None
            self.progress_phase=self.phase
            return (0.,0.,0.)
        if self.was_blocked and self.phase in ('travel','orient'):
            self.was_blocked=False;self._reset_motion_progress(now,self.phase)

        if self.phase=='avoid':
            sign=1. if self.avoid_side=='left' else -1.
            selected_free=left_free if sign>0 else right_free
            # Never drive into a side that became occupied.  Before the body
            # has shifted, switch once to the other verified side if possible.
            if not selected_free:
                progress=self._lateral_progress(pose)
                other_free=right_free if sign>0 else left_free
                if abs(progress)<.025 and other_free:
                    self.avoid_side='right' if sign>0 else 'left';sign=-sign
                    selected_free=True
                else:
                    return (0.,0.,0.)
            progress=self._lateral_progress(pose)
            if abs(progress)<self.avoid_shift:
                # Hold position while stepping sideways; this is conservative
                # for a static obstacle and deterministic on a slow VM.
                return (0.,sign*self.avoid_speed,0.)
            if obstacle_ahead:
                # We are beside the obstacle.  Creep forward until the scan
                # confirms the full obstacle envelope has cleared, not merely
                # until the narrow center ray is clear.
                return (min(self.speed,.08),0.,0.)
            if self.avoid_min_until is None or now<self.avoid_min_until:
                return (0.,sign*self.avoid_speed,0.)
            self.phase='recenter';self._reset_motion_progress(now,'recenter')

        if self.phase=='recenter':
            if obstacle_ahead:
                self.was_blocked=True;return (0.,0.,0.)
            progress=self._lateral_progress(pose)
            if abs(progress)>.018:
                sign=1. if self.avoid_side=='left' else -1.
                return (0.,-sign*self.avoid_speed,0.)
            self.phase='travel';self.avoid_side=None;self.avoid_start_pose=None
            self.obstacle_frames=0;self._reset_motion_progress(now,'travel')

        if self.phase in ('travel','orient'):
            desired=math.atan2(dy,dx) if self.phase=='travel' else math.radians(target['yaw'])
            progress=distance+.15*abs(wrap_angle(desired-pose[2]))
            if self.progress_phase!=self.phase or self.best_progress is None or progress<self.best_progress-.006:
                self.progress_phase=self.phase;self.best_progress=progress;self.progress_time=now
            elif now-self.progress_time>40:
                self.phase='failed';self.error='motion_stalled:'+target['name'];return (0.,0.,0.)
        if self.phase=='travel':
            if distance>.018:
                error=wrap_angle(math.atan2(dy,dx)-pose[2])
                if abs(error)>.045:return (0.,0.,max(-.55,min(.55,1.5*error)))
                return (min(self.speed,max(.025,.8*distance)),0.,max(-.12,min(.12,.8*error)))
            self.phase='orient'
        if self.phase=='orient':
            error=wrap_angle(math.radians(target['yaw'])-pose[2])
            if abs(error)>.025:return (0.,0.,max(-.5,min(.5,1.5*error)))
            self.entered=now
            self.phase='gate' if target.get('gate') else 'observe' if target.get('observe') else 'settle'
        if self.phase=='gate':
            if entry_ready:self.advance()
        elif self.phase=='observe':
            if now-self.entered>=2.0 and len(self.frames)>=3:self.advance()
            elif now-self.entered>25:
                self.phase='failed';self.error='observation_timeout:'+target['name']
        elif self.phase=='settle' and now-self.entered>.2:self.advance()
        return (0.,0.,0.)


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
