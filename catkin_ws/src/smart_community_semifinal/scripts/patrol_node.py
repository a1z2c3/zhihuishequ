#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Minimal ordered-route ROS controller with observations and visual gates."""
from __future__ import division, unicode_literals
import io,json,threading,time
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
            with self.lock:
                self.core.record_frame(data['view'],data['stamp'],data.get('count',0),
                                       data.get('complete',False),
                                       data.get('committed_count'),
                                       data.get('context_id'))
        except (ValueError,KeyError,TypeError):pass

    def run(self):
        self.stop.publish(String(data='{"mode":"clear"}'))
        while not rospy.is_shutdown():
            with self.lock:
                command=Twist();now=rospy.Time.now().to_sec();reason=None;target=self.core.target
                arm_key=(self.core.index,target.get('gate'))
                if target.get('gate') and self.armed!=arm_key:
                    line=next(s for s in self.layout['stop_lines'] if s['id']==target['gate'])
                    light=next(s for s in self.layout['lights'] if s['id']==target['gate'])
                    cycle=self.layout.get('signal_cycle',{})
                    data=dict(line,exit_point=light['xy'],wait_point=target['xy'],min_green_seconds=float(cycle.get('green_s',15.0)))
                    self.stop.publish(String(data=json.dumps(data)));self.armed=arm_key
                try:
                    stamp=self.listener.getLatestCommonTime('map','base_footprint')
                    if not -.1<=now-stamp.to_sec()<.4:raise ValueError('waiting_for_localization')
                    xyz,q=self.listener.lookupTransform('map','base_footprint',stamp)
                    pose=(xyz[0],xyz[1],tf.transformations.euler_from_quaternion(q)[2])
                    guard=dict(self.guard)
                    # An absent or stale guard is an explicit safety hold.
                    # Passing {} would make the core believe the scene is
                    # clear and would also consume its stall watchdog.
                    if (not guard.get('guard_valid',False) or
                            not 0<=now-guard.get('stamp',-1)<.3):
                        guard={'safety_hold':True,'guard_valid':False,
                               'reason':'guard_stale','stamp':now}
                    ready=guard.get('entry_ready',False) and guard.get('stop_id')==target.get('gate')
                    speed,lateral,omega=self.core.step(pose,now,ready,guard)
                    command.linear.x=speed;command.linear.y=lateral;command.angular.z=omega
                except (tf.Exception,ValueError) as exc:
                    reason=str(exc)
                    # Localization loss is a safety hold, not route progress.
                    # Advance neither the stall watchdog nor the avoidance
                    # deadline while waiting for TF to recover.
                    self.core.step((0.,0.,0.),now,guard={'safety_hold':True})
                target=self.core.target
                context_id='%d:%s'%(self.core.index,target['name'])
                context={'active':self.core.phase=='observe','view':target['name'],
                         'street':target.get('street'),'stamp':now,'armed_at':now,
                         'context_id':context_id,'index':self.core.index,
                         'target_xy':target.get('xy'),
                         'target_object_xy':target.get('object_xy'),
                         'expected_category':target.get('expected_category'),
                         'expected_label':target.get('expected_label')}
                key=(context['active'],context['view'],context['index'])
                if key!=self.last_context:
                    self.context.publish(String(data=json.dumps(context)));self.last_context=key
                self.command.publish(command)
                self.status.publish(String(data=json.dumps({'index':self.core.index,'target':target['name'],'phase':self.core.phase,'error':self.core.error or reason,'stamp':now})))
                if self.core.phase in ('done','failed'):
                    rospy.loginfo('Patrol result: %s'%self.core.phase)
                    # Keep the final status available; do not hold the callback lock.
                    break
            # A ROS Rate waits on /clock and cannot service wall watchdogs
            # when the simulator pauses. Keep control/status on wall time.
            time.sleep(.05)
        if not rospy.is_shutdown():
            # Do not leave a stale stop-line policy latched after either a
            # successful route or an explicit failure.
            self.command.publish(Twist())
            self.stop.publish(String(data='{"mode":"clear"}'))
            rospy.spin()


if __name__=='__main__':
    rospy.init_node('semifinal_patrol');Task().run()
