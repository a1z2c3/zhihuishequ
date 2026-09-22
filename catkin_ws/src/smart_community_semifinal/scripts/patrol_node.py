#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Minimal ordered-route ROS controller with observations and visual gates."""
from __future__ import division, unicode_literals
import io,json,threading
import rospy,tf
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from patrol_core import Patrol


class Task(object):
    def __init__(self):
        with io.open(rospy.get_param('~layout'),encoding='utf-8') as f:self.layout=json.load(f)
        self.core=Patrol(self.layout['route']);self.lock=threading.RLock();self.guard={};self.armed=None
        self.listener=tf.TransformListener();self.last_context=None
        self.command=rospy.Publisher('/semifinal/cmd_vel_requested',Twist,queue_size=1)
        self.stop=rospy.Publisher('/semifinal/active_stop',String,queue_size=1,latch=True)
        self.context=rospy.Publisher('/semifinal/observation',String,queue_size=1,latch=True)
        self.status=rospy.Publisher('/semifinal/task_status',String,queue_size=1,latch=True)
        rospy.Subscriber('/semifinal/guard_status',String,self.guard_status,queue_size=1)
        rospy.Subscriber('/semifinal/frame_status',String,self.frame_status,queue_size=10)
        rospy.on_shutdown(lambda:self.command.publish(Twist()))

    def guard_status(self,msg):
        try:
            with self.lock:self.guard=json.loads(msg.data)
        except ValueError:pass

    def frame_status(self,msg):
        try:
            data=json.loads(msg.data)
            with self.lock:self.core.record_frame(data['view'],data['stamp'],data['count'],data.get('complete',False))
        except (ValueError,KeyError,TypeError):pass

    def run(self):
        rate=rospy.Rate(20);self.stop.publish(String(data='{"mode":"clear"}'))
        while not rospy.is_shutdown():
            with self.lock:
                command=Twist();now=rospy.Time.now().to_sec();reason=None;target=self.core.target
                if target.get('gate') and self.armed!=target['gate']:
                    line=next(s for s in self.layout['stop_lines'] if s['id']==target['gate'])
                    light=next(s for s in self.layout['lights'] if s['id']==target['gate'])
                    data=dict(line,exit_point=light['xy'],wait_point=target['xy'],min_green_seconds=15.0)
                    self.stop.publish(String(data=json.dumps(data)));self.armed=target['gate']
                try:
                    stamp=self.listener.getLatestCommonTime('map','base_footprint')
                    if not -.1<=now-stamp.to_sec()<.4:raise ValueError('waiting_for_localization')
                    xyz,q=self.listener.lookupTransform('map','base_footprint',stamp)
                    pose=(xyz[0],xyz[1],tf.transformations.euler_from_quaternion(q)[2])
                    guard=dict(self.guard)
                    if not 0<=now-guard.get('stamp',-1)<.3:
                        guard={}
                    ready=guard.get('entry_ready',False) and guard.get('stop_id')==target.get('gate')
                    speed,lateral,omega=self.core.step(pose,now,ready,guard)
                    command.linear.x=speed;command.linear.y=lateral;command.angular.z=omega
                except (tf.Exception,ValueError) as exc:reason=str(exc)
                target=self.core.target
                context={'active':self.core.phase=='observe','view':target['name'],'street':target.get('street'),'stamp':now,'index':self.core.index}
                key=(context['active'],context['view'])
                if key!=self.last_context:
                    self.context.publish(String(data=json.dumps(context)));self.last_context=key
                self.command.publish(command)
                self.status.publish(String(data=json.dumps({'index':self.core.index,'target':target['name'],'phase':self.core.phase,'error':self.core.error or reason,'stamp':now})))
                if self.core.phase in ('done','failed'):
                    rospy.loginfo('Patrol result: %s'%self.core.phase)
                    # Keep the final status available; do not hold the callback lock.
                    break
            rate.sleep()
        if not rospy.is_shutdown():rospy.spin()


if __name__=='__main__':
    rospy.init_node('semifinal_patrol');Task().run()
