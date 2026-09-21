#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Sole velocity publisher: sensor, command, geometry and fresh visual checks."""
from __future__ import division, unicode_literals
import io,json,math,threading
import rospy,tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from patrol_core import CrossingPolicy
from runtime_compat import isfinite,monotonic
from scene_geometry import body_violation,physical_obstacles
from semifinal_core import braking_distance


class Guard(object):
    def __init__(self):
        with io.open(rospy.get_param('~layout'),encoding='utf-8') as f:self.layout=json.load(f)
        self.obstacles=physical_obstacles(rospy.get_param('~world'))
        self.listener=tf.TransformListener();self.policy=CrossingPolicy();self.lock=threading.RLock()
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
            except (ValueError,KeyError,TypeError):self.policy.mode='unarmed'

    def on_signal(self,msg):
        with self.lock:
            try:self.policy.signal(json.loads(msg.data),rospy.Time.now().to_sec())
            except (ValueError,KeyError,TypeError):self.policy.gate.reset();self.policy.green_start=None

    def run(self):
        rate=rospy.Rate(25)
        while not rospy.is_shutdown():
            with self.lock:
                out=Twist();reason=None;now=rospy.Time.now().to_sec()
                try:
                    stamp=self.listener.getLatestCommonTime('map','base_footprint')
                    if not -.1<=now-stamp.to_sec()<.4:raise ValueError('stale_tf')
                    xyz,q=self.listener.lookupTransform('map','base_footprint',stamp)
                    self.pose=(xyz[0],xyz[1],tf.transformations.euler_from_quaternion(q)[2])
                    # Software-rendered VM runs at <=0.2 real-time. Enforce both
                    # simulation-time freshness and a bounded wall watchdog.
                    if self.command_at is None or monotonic()-self.command_at>1.0 or not 0<=now-self.command_sim<.15:raise ValueError('command_timeout')
                    if self.scan_at is None or monotonic()-self.scan_at>1.5 or not 0<=now-self.scan.header.stamp.to_sec()<.4:raise ValueError('stale_scan')
                    speed=max(0.,min(.18,self.command.linear.x));omega=max(-.55,min(.55,self.command.angular.z))
                    projected=(self.pose[0]+speed*math.cos(self.pose[2])*.25,self.pose[1]+speed*math.sin(self.pose[2])*.25,self.pose[2]+omega*.25)
                    bad=body_violation(projected,self.layout,self.obstacles)
                    if bad:raise ValueError('body_constraint:'+bad)
                    for i,d in enumerate(self.scan.ranges):
                        angle=self.scan.angle_min+i*self.scan.angle_increment
                        x=d*math.cos(angle);y=d*math.sin(angle)
                        if isfinite(d) and self.scan.range_min<d<self.scan.range_max and speed>0 and abs(y)<.175 and 0<x<.187+braking_distance(speed):
                            raise ValueError('forward_obstacle')
                    speed,omega=self.policy.filter(self.pose,speed,omega,now)
                    out.linear.x=speed;out.angular.z=omega
                    if self.policy.failure:reason=self.policy.failure
                except (tf.Exception,ValueError) as exc:reason=str(exc)
                self.output.publish(out)
                self.status.publish(String(data=json.dumps({'mode':self.policy.mode,'entry_ready':self.policy.ready(now),
                    'stop_id':self.policy.stop['id'] if self.policy.stop else None,'reason':reason,'stamp':now,'map_pose':self.pose})))
            rate.sleep()


if __name__=='__main__':
    rospy.init_node('traffic_guard');Guard().run()
