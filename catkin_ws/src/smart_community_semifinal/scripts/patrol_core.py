#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Ordered route and entry policy using fresh visual green epochs."""
from __future__ import division, unicode_literals
import math
from semifinal_core import (GreenGate,wrap_angle,front_clearance,
                             braking_distance,footprint_corners,AVOID_SHIFT,
                             integrate_twist_pose)
from runtime_compat import isfinite,monotonic


class Patrol(object):
    def __init__(self,route,speed=.18):
        self.route=route;self.speed=speed;self.index=0;self.phase='travel'
        self.entered=None;self.entered_wall=None;self.frames=set();self.observation_id=None;self.error=None
        self.progress_time=None;self.best_progress=None;self.progress_phase=None
        # Avoidance is an event, not a timer-based detour.  The selected side
        # remains locked until the obstacle rear has cleared the footprint.
        self.avoid_shift=AVOID_SHIFT;self.avoid_speed=.08
        self.avoid_side=None;self.avoid_start_pose=None
        self.avoid_rear_x=None
        self.avoid_min_until=None;self.avoid_deadline=None
        self.avoid_attempted=False;self.avoid_failed=False
        self.obstacle_frames=0;self.was_blocked=False;self.blocked_since=None
        self.blocked_timeout=90.0
        self.safety_hold=False
        self.hold_started=None
        self.hold_wall_started=None
        self.safety_hold_timeout=90.0
        self.safety_hold_wall_timeout=600.0
        self.lane_recovery_started=None
        self.lane_recovery_timeout=30.0
        self.gate_timeout=90.0

    @property
    def target(self):return self.route[min(self.index,len(self.route)-1)]

    @staticmethod
    def _timeout_expired(now,started,sim_limit,wall_now,wall_started,wall_limit):
        return bool((sim_limit is not None and now-started>=sim_limit) or
                    (wall_limit is not None and
                     wall_now-wall_started>=wall_limit))

    def record_frame(self,view,stamp,count,complete=True,committed_count=None,context_id=None):
        """Accept one frame only after the perception transaction commits it.

        ``count`` is retained for compatibility with older status messages;
        new nodes must provide ``committed_count``.  The context id prevents a
        delayed frame from the previous observation point satisfying this one.
        """
        accepted = count if committed_count is None else committed_count
        same_context = (context_id == self.observation_id if committed_count is not None
                        else context_id in (None,self.observation_id))
        if (self.phase=='observe' and view==self.target['name'] and
                same_context and accepted>0 and
                complete and stamp>=self.entered):
            self.frames.add(stamp)

    def advance(self):
        self.index+=1;self.entered=None;self.entered_wall=None;self.frames=set();self.observation_id=None
        self.progress_time=None;self.best_progress=None;self.progress_phase=None
        self.avoid_side=None;self.avoid_start_pose=None
        self.avoid_rear_x=None
        self.avoid_min_until=None;self.avoid_deadline=None
        self.avoid_attempted=False;self.avoid_failed=False
        self.obstacle_frames=0;self.was_blocked=False;self.blocked_since=None;self.safety_hold=False
        self.hold_started=None;self.hold_wall_started=None
        self.lane_recovery_started=None
        self.phase='done' if self.index>=len(self.route) else 'travel'

    def _lateral_progress(self,pose):
        if self.avoid_start_pose is None:return 0.
        sx,sy,syaw=self.avoid_start_pose
        dx,dy=pose[0]-sx,pose[1]-sy
        return -dx*math.sin(syaw)+dy*math.cos(syaw)

    def _longitudinal_progress(self,pose):
        if self.avoid_start_pose is None:return 0.
        sx,sy,syaw=self.avoid_start_pose
        dx,dy=pose[0]-sx,pose[1]-sy
        return dx*math.cos(syaw)+dy*math.sin(syaw)

    def _reset_motion_progress(self,now,phase):
        self.progress_phase=phase;self.best_progress=None;self.progress_time=now

    def step(self,pose,now,entry_ready=False,guard=None):
        """Return body-frame ``(linear.x, linear.y, angular.z)``.

        A stale guard is a safety hold rather than an empty observation.  A
        centered obstacle is a legitimate waiting state, so neither condition
        consumes the motion-stall watchdog.
        """
        if self.phase in ('done','failed'):return (0.,0.,0.)
        target=self.target;dx=target['xy'][0]-pose[0];dy=target['xy'][1]-pose[1]
        distance=math.hypot(dx,dy)
        guard=guard or {}
        wall_now=monotonic()
        if bool(guard.get('safety_hold',False)):
            if not self.safety_hold:
                self.hold_started=now
                self.hold_wall_started=wall_now
            self.safety_hold=True;self.progress_time=now;self.best_progress=None
            self.progress_phase=self.phase
            sim_elapsed=max(0.,now-self.hold_started)
            wall_elapsed=max(0.,wall_now-self.hold_wall_started)
            if (sim_elapsed>=self.safety_hold_timeout or
                    wall_elapsed>=self.safety_hold_wall_timeout):
                self.phase='failed';self.error='safety_hold_timeout:'+target['name']
            return (0.,0.,0.)
        if self.safety_hold and self.hold_started is not None:
            hold_duration=max(0.,now-self.hold_started)
            if self.avoid_deadline is not None:self.avoid_deadline+=hold_duration
            if self.blocked_since is not None:self.blocked_since+=hold_duration
            if self.lane_recovery_started is not None:self.lane_recovery_started+=hold_duration
            if self.entered is not None:self.entered+=hold_duration
            if self.entered_wall is not None:self.entered_wall+=max(0.,wall_now-self.hold_wall_started)
        self.hold_started=None
        self.hold_wall_started=None
        self.safety_hold=False

        # A geometry supervisor can report that the current footprint is
        # already outside the lane strip.  This is a bounded recovery mode,
        # not a normal route state: only a small body-frame lateral command
        # toward the lane is emitted, and every command still passes through
        # the guard's swept-footprint check.
        if bool(guard.get('lane_recovery',False)):
            if self.lane_recovery_started is None:self.lane_recovery_started=now
            if now-self.lane_recovery_started>self.lane_recovery_timeout:
                self.phase='failed';self.error='lane_recovery_timeout:'+target['name']
                return (0.,0.,0.)
            self.progress_time=now;self.best_progress=None
            direction=float(guard.get('lane_recovery_lateral',0.) or 0.)
            if abs(direction)<.01:return (0.,0.,0.)
            return (0.,.04 if direction>0 else -.04,0.)
        if self.lane_recovery_started is not None:
            self.lane_recovery_started=None
        forward=bool(guard.get('forward_obstacle',False))
        obstacle_ahead=bool(guard.get('obstacle_ahead',forward))
        left_free=bool(guard.get('left_free',False))
        right_free=bool(guard.get('right_free',False))
        recenter_clear=bool(guard.get('recenter_clear',not obstacle_ahead))
        blocked=forward and not left_free and not right_free

        # Once an avoidance attempt has timed out, hold position while the
        # obstacle is still observable.  Do not immediately start the same
        # maneuver again: that creates an avoid/recenter oscillation loop.
        if self.avoid_failed and self.phase in ('travel','orient'):
            if forward or obstacle_ahead or not recenter_clear:
                if self.blocked_since is None:self.blocked_since=now
                self.was_blocked=True;self.progress_time=now;self.best_progress=None
                self.progress_phase=self.phase
                if now-self.blocked_since>self.blocked_timeout:
                    self.phase='failed';self.error='obstacle_blocked_timeout:'+target['name']
                return (0.,0.,0.)
            self.avoid_failed=False;self.error=None;self.blocked_since=None
            self.was_blocked=False;self._reset_motion_progress(now,self.phase)

        # Require consecutive asymmetric observations before creating one
        # maneuver event.  Once in avoid/recenter, never re-trigger from the
        # same obstacle or switch sides because a scan flickers.
        if self.phase in ('travel','orient'):
            if forward and left_free != right_free:
                self.obstacle_frames+=1
            elif not forward:
                self.obstacle_frames=0
            if self.obstacle_frames>=3 and not self.avoid_attempted:
                self.phase='avoid';self.avoid_side='left' if left_free else 'right'
                self.avoid_start_pose=(pose[0],pose[1],pose[2])
                self.avoid_rear_x=guard.get('obstacle_rear_x')
                self.avoid_min_until=now+.30;self.avoid_deadline=now+18.0
                self.avoid_attempted=True;self.obstacle_frames=0;self._reset_motion_progress(now,'avoid')

        # A center-blocked obstacle is a wait condition.  Keep the watchdog
        # clock fresh until the scene changes instead of reporting a failure.
        if blocked and self.phase in ('travel','orient'):
            if self.blocked_since is None:self.blocked_since=now
            self.was_blocked=True;self.progress_time=now;self.best_progress=None
            self.progress_phase=self.phase
            if now-self.blocked_since>self.blocked_timeout:
                self.phase='failed';self.error='obstacle_blocked_timeout:'+target['name']
            return (0.,0.,0.)
        if self.was_blocked and self.phase in ('travel','orient'):
            self.was_blocked=False;self.blocked_since=None;self._reset_motion_progress(now,self.phase)

        if self.phase=='avoid':
            if self.avoid_deadline is not None and now>self.avoid_deadline:
                self.phase='travel';self.error='avoid_timeout:'+target['name']
                self.avoid_failed=True;self.avoid_side=None;self.avoid_start_pose=None;self.avoid_rear_x=None
                self.avoid_min_until=None;self.avoid_deadline=None
                self.blocked_since=now
                return (0.,0.,0.)
            sign=1. if self.avoid_side=='left' else -1.
            selected_obstacle_ahead=bool(guard.get(
                'left_obstacle_ahead' if sign>0 else 'right_obstacle_ahead',
                obstacle_ahead))
            selected_free=left_free if sign>0 else right_free
            progress=self._lateral_progress(pose)
            longitudinal=self._longitudinal_progress(pose)
            passed_snapshot=(self.avoid_rear_x is not None and
                             longitudinal>float(self.avoid_rear_x)+.187)
            if not selected_free:
                other_free=right_free if sign>0 else left_free
                if abs(progress)<.025 and other_free:
                    self.avoid_side='right' if sign>0 else 'left';sign=-sign
                    selected_free=True
                else:
                    self.was_blocked=True;self.progress_time=now
                    return (0.,0.,0.)
            if abs(progress)<self.avoid_shift:
                return (0.,sign*self.avoid_speed,0.)
            # Never keep crawling through the goal while offset.  Recenter
            # first, then let the normal travel/orient completion logic run.
            if not recenter_clear:
                return (min(self.speed,.08),0.,0.)
            if distance<=.035 and (passed_snapshot or
                                   self.avoid_rear_x is None) and not selected_obstacle_ahead:
                self.phase='recenter';self._reset_motion_progress(now,'recenter')
                return (0.,0.,0.)
            if selected_obstacle_ahead or (self.avoid_rear_x is not None and
                                           not passed_snapshot):
                # Remain offset until the scan proves the whole obstacle,
                # including its rear edge, has passed the robot.
                return (min(self.speed,.08),0.,0.)
            if self.avoid_min_until is None or now<self.avoid_min_until:
                return (0.,sign*self.avoid_speed,0.)
            self.phase='recenter';self._reset_motion_progress(now,'recenter')

        if self.phase=='recenter':
            if self.avoid_deadline is not None and now>self.avoid_deadline:
                self.phase='travel';self.error='avoid_timeout:'+target['name']
                self.avoid_failed=True;self.avoid_side=None;self.avoid_start_pose=None;self.avoid_rear_x=None
                self.avoid_min_until=None;self.avoid_deadline=None
                self.blocked_since=now
                return (0.,0.,0.)
            if obstacle_ahead or not recenter_clear:
                self.was_blocked=True;self.progress_time=now
                return (0.,0.,0.)
            progress=self._lateral_progress(pose)
            if abs(progress)>.018:
                sign=1. if self.avoid_side=='left' else -1.
                return (0.,-sign*self.avoid_speed,0.)
            self.phase='travel';self.avoid_side=None;self.avoid_start_pose=None;self.avoid_rear_x=None
            self.avoid_min_until=None;self.avoid_deadline=None
            self.obstacle_frames=0;self.was_blocked=False;self.blocked_since=None
            self._reset_motion_progress(now,'travel')

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
            self.entered=now;self.entered_wall=wall_now
            if target.get('gate'):
                self.phase='gate'
            elif target.get('observe'):
                self.observation_id='%d:%s'%(self.index,target['name'])
                self.frames=set();self.phase='observe'
            else:
                self.phase='settle'
        if self.phase=='observe':
            if now-self.entered>=2.0 and len(self.frames)>=3:self.advance()
            elif self._timeout_expired(now,self.entered,25.0,wall_now,
                                       self.entered_wall,180.0):
                self.phase='failed';self.error='observation_timeout:'+target['name']
        elif self.phase=='gate':
            if entry_ready:self.advance()
            elif self._timeout_expired(now,self.entered,self.gate_timeout,
                                       wall_now,self.entered_wall,600.0):
                self.phase='failed';self.error='gate_timeout:'+target['name']
        elif self.phase=='settle' and now-self.entered>.2:self.advance()
        return (0.,0.,0.)


class CrossingPolicy(object):
    """The configured green-duration lower bound is a simulation assumption.

    A witnessed red/yellow -> green transition establishes both the current
    onset and (when configured) a simulation-phase clock.  Brief perception
    gaps revoke the current frame quorum but preserve that verified onset;
    unknown green without a locked clock remains fail-safe and cannot invent
    a full remaining green interval.
    """
    def __init__(self,signal_cycle=None):
        self.gate=GreenGate();self.stop=None;self.mode='unarmed';self.previous=None
        self.green_start=None;self.red_start=None;self.failure=None
        cycle=signal_cycle or {}
        try:
            period=float(cycle.get('period_s',0.))
            red=float(cycle.get('red_s',0.))
            green=float(cycle.get('green_s',0.))
            light_2_offset=float(cycle.get('light_2_offset_s',0.))
        except (TypeError,ValueError):
            period=red=green=light_2_offset=0.
        self.clock_valid=(isfinite(period) and period>0. and
                          isfinite(red) and red>=0. and
                          isfinite(green) and green>0. and
                          red+green<=period and isfinite(light_2_offset))
        self.clock_period=period if self.clock_valid else None
        self.clock_red=red if self.clock_valid else None
        self.clock_green=green if self.clock_valid else None
        # A single false-red frame must not be promoted to a real cycle edge.
        # Require evidence of at least half the configured red interval before
        # accepting a witnessed red -> green transition as an onset anchor.
        self.min_red_observation=(.5*red if self.clock_valid else 0.)
        # Red-episode continuity is separate from the green quorum gap.  A
        # single slow-render frame may be about 0.4 s apart in the VM; allow a
        # bounded gap for red evidence without accepting arbitrarily stale data.
        self.red_gap_limit=(max(self.gate.max_gap,min(1.0,.1*red))
                            if self.clock_valid else self.gate.max_gap)
        offsets=cycle.get('offsets_s')
        if offsets is None:offsets={'light_1':0.,'light_2':light_2_offset}
        if not isinstance(offsets,dict) or not offsets or not all(
                isfinite(value) for value in offsets.values()):
            raise ValueError('invalid signal clock offsets')
        self.clock_offsets=dict((key,float(value)) for key,value in offsets.items())
        self.clock_origin=None

    def arm(self,stop):
        if self.stop and self.stop['id']==stop['id']:return
        if self.stop is not None:
            raise ValueError('active stop line must clear before arming another')
        if self.clock_valid and stop['id'] not in self.clock_offsets:
            raise ValueError('missing signal clock offset: '+stop['id'])
        for field in ('point','direction','exit_point','wait_point'):
            if len(stop[field])!=2 or not all(isfinite(v) for v in stop[field]):raise ValueError(field)
        if math.hypot(*stop['direction'])<.99:raise ValueError('invalid direction')
        if not isfinite(stop['min_green_seconds']) or stop['min_green_seconds']<=0:raise ValueError('green bound')
        self.stop=stop;self.mode='approach';self.gate.reset();self.previous=None
        self.green_start=None;self.red_start=None;self.failure=None

    def signal(self,s,now):
        if self.stop is None or s.get('light_id')!=self.stop['id']:return
        stamp=s.get('stamp');state=s.get('state');score=s.get('confidence',0)
        valid=isfinite(stamp) and isfinite(score) and 0<=now-stamp<=.35 and score>=.7
        if not valid or state not in ('red','yellow','green'):
            # Unknown/stale observations revoke the current frame quorum.  A
            # verified onset remains useful: throwing it away would turn one
            # dropped frame into a needless full-cycle wait.  Unverified
            # green has no anchor to preserve and remains fail-safe.
            self.previous=None
            self.red_start=None;self.gate.reset();return
        if self.previous and stamp<=self.previous[1]:
            if stamp<self.previous[1]:
                self.green_start=None;self.red_start=None;self.previous=None
                self.gate.reset();self.clock_origin=None
            return
        gap = None if self.previous is None else stamp-self.previous[1]
        if state == 'red':
            # Red immediately revokes the current entry permission.  The
            # episode is tracked separately only to validate the next green.
            self.green_start=None
            # Start a new continuous red episode after any non-red state or a
            # gap too large to establish continuity.  The duration is checked
            # when green arrives; this rejects one-frame false-red detections.
            if (self.previous is None or self.previous[0] != 'red' or
                    gap is None or gap > self.red_gap_limit or
                    self.red_start is None):
                self.red_start=stamp
        elif state == 'green':
            red_duration=(stamp-self.red_start if
                          self.previous is not None and
                          self.previous[0]=='red' and
                          gap is not None and gap <= self.red_gap_limit and
                          self.red_start is not None else None)
            if (red_duration is not None and
                    red_duration+1e-6 >= self.min_red_observation):
                # A witnessed transition is anchored at the last non-green
                # frame, preserving the available-green-time estimate.  The
                # red episode must be long enough to reject false red frames.
                self.green_start=self.previous[1]
                self._lock_phase_clock(s.get('light_id'),self.green_start)
            elif (self.previous is None or
                  (gap > self.gate.max_gap and self.previous[0] != 'red')):
                # A locked clock can recover the real onset after arming or a
                # non-red gap.  A preceding unverified red episode is never
                # bypassed by the clock; it must earn the red-duration check.
                inferred=self._infer_green_start(s.get('light_id'),stamp)
                if inferred is not None and (self.green_start is None or
                                             inferred>self.green_start):
                    self.green_start=inferred
        else:
            # Yellow immediately revokes entry permission.
            self.green_start=None
            self.red_start=None
        self.previous=(state,stamp)
        self.gate.update(s['light_id'],state,score,stamp,now)

    def _offset_for(self,light_id):
        if light_id not in self.clock_offsets:return None
        return float(self.clock_offsets[light_id])

    def _lock_phase_clock(self,light_id,onset):
        if not self.clock_valid or self.clock_origin is not None:return
        offset=self._offset_for(light_id)
        if offset is None:return
        self.clock_origin=float(onset)+offset-self.clock_red

    def _infer_green_start(self,light_id,stamp):
        """Infer the latest green onset only after a visual clock lock."""
        if not self.clock_valid or self.clock_origin is None:return None
        offset=self._offset_for(light_id)
        if offset is None:return None
        # clock_origin is the shared world-time start of a cycle.  The lamp's
        # configured offset is applied exactly once when evaluating its phase.
        phase=(float(stamp)+offset-self.clock_origin)%self.clock_period
        if phase < self.clock_red or phase >= self.clock_red+self.clock_green:return None
        return float(stamp)-phase+self.clock_red

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
        self.stop=None;self.mode='clear';self.green_start=None;self.red_start=None
        self.previous=None
        self.failure=None;self.gate.reset();return True

    def filter_command(self,pose,speed,lateral,omega,now):
        """Filter a complete holonomic command at a visual stop line."""
        if self.mode=='unarmed':return 0.,0.,0.
        if self.stop is not None:
            if self.mode=='crossing' and self.exited(pose):self.clear(pose)
            elif self.mode=='approach':
                clearance=front_clearance(pose,self.stop['point'],self.stop['direction'])
                if clearance<0:
                    self.failure='stop_line_crossed_before_permission';return 0.,0.,0.
                if clearance<=braking_distance(speed)+.012:
                    if self.ready(now):self.mode='crossing'
                    else:return 0.,0.,0.
                # A lateral command can cross the line even when its forward
                # component is small. Sample the full body trajectory before
                # granting permission to any holonomic move_base command.
                if self.mode=='approach' and not self.ready(now):
                    for fraction in (.2,.4,.6,.8,1.0):
                        dt=.25*fraction
                        projected=integrate_twist_pose(pose,speed,lateral,omega,dt)
                        if front_clearance(projected,self.stop['point'],self.stop['direction'])<=0:
                            return 0.,0.,0.
        return speed,lateral,omega

    def filter(self,pose,speed,omega,now):
        speed,unused,omega=self.filter_command(pose,speed,0.,omega,now)
        return speed,omega
