#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Sole velocity publisher: sensor, command, geometry and fresh visual checks."""
from __future__ import division, unicode_literals
import io,json,math,threading,time
import rospy,tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from patrol_core import CrossingPolicy
from runtime_compat import isfinite,monotonic
from scene_geometry import (physical_obstacles,trajectory_violation,
                            body_violation,lane_recovery_vector,
                            lane_limited_twist,sampled_twist_poses)
from semifinal_core import scan_clearance


class Guard(object):
    def __init__(self):
        with io.open(rospy.get_param('~layout'),encoding='utf-8') as f:self.layout=json.load(f)
        self.obstacles=physical_obstacles(rospy.get_param('~world'))
        cycle=self.layout.get('signal_cycle',{})
        self.listener=tf.TransformListener();self.policy=CrossingPolicy(cycle);self.lock=threading.RLock()
        self.command=Twist();self.command_at=None;self.command_sim=None;self.scan=None;self.scan_at=None;self.pose=None
        self.output=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.status=rospy.Publisher('/semifinal/guard_status',String,queue_size=1)
        rospy.Subscriber('/semifinal/cmd_vel_requested',Twist,self.on_command,queue_size=1)
        rospy.Subscriber('/semifinal/active_stop',String,self.on_stop,queue_size=1)
        rospy.Subscriber('/semifinal/visual_signal',String,self.on_signal,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.on_scan,queue_size=1)
        rospy.on_shutdown(lambda:self.output.publish(Twist()))

    def on_command(self,msg):
        with self.lock:
            if not all(isfinite(v) for v in [msg.linear.x,msg.linear.y,msg.angular.z]):
                self.command=Twist();self.command_at=None;return
            self.command=msg;self.command_at=monotonic();self.command_sim=rospy.Time.now().to_sec()

    def on_scan(self,msg):
        with self.lock:self.scan=msg;self.scan_at=monotonic()

    def on_stop(self,msg):
        with self.lock:
            try:
                data=json.loads(msg.data)
                if data.get('mode')=='clear':self.policy.clear(self.pose)
                else:self.policy.arm(data)
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn('Rejected stop-line update: %s'%exc)
                if self.policy.stop is None:self.policy.mode='unarmed'

    def on_signal(self,msg):
        with self.lock:
            try:self.policy.signal(json.loads(msg.data),rospy.Time.now().to_sec())
            except (ValueError,KeyError,TypeError):self.policy.gate.reset();self.policy.green_start=None

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                out=Twist();reason=None;now=rospy.Time.now().to_sec()
                forward_obstacle=False;obstacle_ahead=False
                left_obstacle_ahead=False;right_obstacle_ahead=False
                left_free=False;right_free=False;recenter_clear=False
                clearance={'left_clearance':0.0,'right_clearance':0.0}
                lane_recovery=False;lane_recovery_lateral=0.0
                lane_limited=False;current_violation=None
                safety_hold=True;guard_valid=False
                try:
                    stamp=self.listener.getLatestCommonTime('map','base_footprint')
                    if not -.1<=now-stamp.to_sec()<.4:raise ValueError('stale_tf')
                    xyz,q=self.listener.lookupTransform('map','base_footprint',stamp)
                    self.pose=(xyz[0],xyz[1],tf.transformations.euler_from_quaternion(q)[2])
                    # Software-rendered VM runs at <=0.2 real-time. Enforce both
                    # simulation-time freshness and a bounded wall watchdog.
                    if self.command_at is None or monotonic()-self.command_at>1.0 or not 0<=now-self.command_sim<.15:raise ValueError('command_timeout')
                    if self.scan_at is None or monotonic()-self.scan_at>1.5 or not 0<=now-self.scan.header.stamp.to_sec()<.4:raise ValueError('stale_scan')
                    speed=max(0.,min(.18,self.command.linear.x));lateral=max(-.12,min(.12,self.command.linear.y));omega=max(-.55,min(.55,self.command.angular.z))
                    current_violation=body_violation(self.pose,self.layout,self.obstacles)
                    if current_violation=='lane_corridor':
                        lane_recovery=True
                        vx,vy=lane_recovery_vector(self.pose,self.layout)
                        # Convert the map-frame vector toward the closest
                        # lane point into the robot's body-left convention.
                        yaw=self.pose[2]
                        lane_recovery_lateral=max(-1.,min(1.,
                            -math.sin(yaw)*vx+math.cos(yaw)*vy))
                    # Check the complete commanded horizon, not just its end
                    # point. Endpoint-only checks can tunnel through a thin
                    # pole or card when the VM publishes a large time step.
                    # Keep the nominal horizon, but extend it when wall-time
                    # scheduling has delayed the command callback. This
                    # prevents a delayed guard tick from creating an unseen
                    # validation-to-actuation gap.
                    command_age=0. if self.command_at is None else max(0.,monotonic()-self.command_at)
                    horizon=max(.25,min(.75,.25+command_age*5.0))
                    projected_poses=sampled_twist_poses(
                        self.pose,speed,lateral,omega,horizon)
                    bad=trajectory_violation(self.pose,projected_poses,
                                             self.layout,self.obstacles)
                    if bad=='lane_corridor' and current_violation is None:
                        # A legal pose may still leave the corridor during
                        # the prediction horizon.  Preserve forward progress
                        # where possible and attenuate only the offending
                        # lateral/turning component before resorting to zero.
                        speed,lateral,omega,bad,lane_limited=lane_limited_twist(
                            self.pose,speed,lateral,omega,self.layout,
                            self.obstacles,horizon)
                    if bad=='lane_corridor' and lane_recovery:
                        # Let patrol receive a valid status and generate an
                        # inward recovery command on the next cycle.  A
                        # regular outward command is never passed through.
                        speed=lateral=omega=0.;bad=None
                    if bad:raise ValueError('body_constraint:'+bad)
                    clearance=scan_clearance(self.scan,speed)
                    forward_obstacle=clearance['forward_obstacle']
                    obstacle_ahead=clearance['obstacle_ahead']
                    left_obstacle_ahead=clearance.get('left_obstacle_ahead',obstacle_ahead)
                    right_obstacle_ahead=clearance.get('right_obstacle_ahead',obstacle_ahead)
                    left_free=clearance['left_free'];right_free=clearance['right_free']
                    recenter_clear=clearance['recenter_clear']
                    if forward_obstacle:
                        if not left_free and not right_free:
                            speed=0.;lateral=0.;reason='forward_obstacle_wait'
                        elif abs(lateral)<.01:
                            speed=0.;lateral=0.;reason='forward_obstacle_select_side'
                        elif lateral>0 and not left_free:
                            speed=0.;lateral=0.;reason='requested_left_blocked'
                        elif lateral<0 and not right_free:
                            speed=0.;lateral=0.;reason='requested_right_blocked'
                        else:
                            speed=min(speed,.06)
                    speed,lateral,omega=self.policy.filter_command(self.pose,speed,lateral,omega,now)
                    out.linear.x=speed;out.linear.y=lateral;out.angular.z=omega
                    if self.policy.failure:reason=self.policy.failure
                    elif (self.policy.stop is not None and
                          self.policy.green_start is None):
                        reason='waiting_for_verified_green_onset'
                    elif lane_limited:reason='body_constraint:lane_corridor_limited'
                    elif lane_recovery:reason='body_constraint:lane_corridor_recovery'
                    safety_hold=False;guard_valid=True
                except (tf.Exception,ValueError) as exc:reason=str(exc)
                self.output.publish(out)
                self.status.publish(String(data=json.dumps({'mode':self.policy.mode,'entry_ready':self.policy.ready(now),
                    'stop_id':self.policy.stop['id'] if self.policy.stop else None,'reason':reason,
                    'safety_hold':safety_hold,'guard_valid':guard_valid,
                    'forward_obstacle':forward_obstacle,'obstacle_ahead':obstacle_ahead,
                    'left_obstacle_ahead':left_obstacle_ahead,
                    'right_obstacle_ahead':right_obstacle_ahead,
                    'left_free':left_free,'right_free':right_free,
                    'recenter_clear':recenter_clear,
                    'lane_recovery':lane_recovery,
                    'lane_recovery_lateral':lane_recovery_lateral,
                    'lane_limited':lane_limited,
                    'left_clearance':clearance.get('left_clearance',0.0),
                    'right_clearance':clearance.get('right_clearance',0.0),
                    'stamp':now,'map_pose':self.pose})))
            # Publish a fresh zero command even if simulated /clock stalls.
            time.sleep(.04)


if __name__=='__main__':
    rospy.init_node('traffic_guard');Guard().run()
