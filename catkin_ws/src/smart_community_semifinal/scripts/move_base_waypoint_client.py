#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Move-base waypoint client for the optional standard navigation chain.

This node owns goals and, while waiting at a visual gate, publishes only a
neutral request to keep the guard's command-freshness watchdog alive.  It
never publishes ``/cmd_vel``; traffic, scan freshness and body geometry
remain enforced by one physical velocity arbiter.
"""
from __future__ import division, unicode_literals
import io,json,math,threading,time
import actionlib
import rospy,tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped,Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String
from navigation_core import gate_entry_ready
from runtime_compat import monotonic


class WaypointClient(object):
    def __init__(self):
        with io.open(rospy.get_param('~route_layout',rospy.get_param('~layout')),
                     encoding='utf-8') as stream:
            self.layout=json.load(stream)
        self.goals=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.hold_request=rospy.Publisher('/semifinal/cmd_vel_requested',Twist,queue_size=1)
        self.stop=rospy.Publisher('/semifinal/active_stop',String,queue_size=1,latch=True)
        self.context=rospy.Publisher('/semifinal/observation',String,queue_size=1,latch=True)
        self.status=rospy.Publisher('/semifinal/task_status',String,queue_size=1,latch=True)
        self.lock=threading.RLock();self.last_frame=None;self.last_guard=None;self.last_view=None
        rospy.Subscriber('/semifinal/frame_status',String,self.on_frame,queue_size=10)
        rospy.Subscriber('/semifinal/guard_status',String,self.on_guard,queue_size=1)
        rospy.on_shutdown(self.shutdown)

    def on_frame(self,msg):
        try:
            data=json.loads(msg.data)
            with self.lock:self.last_frame=data
        except (ValueError,TypeError):
            return

    def on_guard(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                with self.lock:self.last_guard=data
        except (ValueError,TypeError):
            return

    def shutdown(self):
        try:self.goals.cancel_all_goals()
        except Exception:pass
        try:self.hold_request.publish(Twist())
        except Exception:pass
        self.stop.publish(String(data='{"mode":"clear"}'))

    def publish_context(self,item,active,stamp,index,context_id):
        # Arm perception only after move_base reports that the goal is reached.
        # Frames acquired while driving to the waypoint are never evidence for
        # that observation point.
        data={'active':bool(active),'view':item['name'],
              'street':item.get('street'),'stamp':stamp,'armed_at':stamp,
              'warmup_s':.35 if active else 0.,'context_id':context_id,
              'index':index,'target_xy':item.get('xy'),
              'target_object_xy':item.get('object_xy'),
              'expected_category':item.get('expected_category'),
              'expected_label':item.get('expected_label')}
        self.context.publish(String(data=json.dumps(data)))

    def arm_gate(self,item):
        gate=item.get('gate')
        if not gate:return
        line=next(s for s in self.layout['stop_lines'] if s['id']==gate)
        light=next(s for s in self.layout['lights'] if s['id']==gate)
        cycle=self.layout.get('signal_cycle',{})
        data=dict(line,exit_point=light['xy'],wait_point=item['xy'],
                  min_green_seconds=float(cycle.get('green_s',15.0)))
        self.stop.publish(String(data=json.dumps(data)))

    def make_goal(self,item):
        yaw=math.radians(float(item['yaw']))
        q=tf.transformations.quaternion_from_euler(0.,0.,yaw)
        goal=MoveBaseGoal();goal.target_pose=PoseStamped()
        goal.target_pose.header.frame_id='map'
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x=float(item['xy'][0])
        goal.target_pose.pose.position.y=float(item['xy'][1])
        goal.target_pose.pose.orientation.x=q[0];goal.target_pose.pose.orientation.y=q[1]
        goal.target_pose.pose.orientation.z=q[2];goal.target_pose.pose.orientation.w=q[3]
        return goal

    def wait_observation(self,item,start,context_id):
        wall_start=monotonic()
        while (not rospy.is_shutdown() and rospy.Time.now().to_sec()-start<25.0
               and monotonic()-wall_start<180.0):
            with self.lock:frame=self.last_frame
            if (frame and frame.get('view')==item['name'] and
                    frame.get('context_id')==context_id and
                    frame.get('complete',False) and frame.get('count',0)>0 and
                    frame.get('committed_count',0)>0 and
                    frame.get('stamp',-1)>=start):
                return True
            time.sleep(.1)
        return False

    def wait_goal(self,sim_start):
        wall_start=monotonic()
        wall_timeout=float(rospy.get_param('~goal_wall_timeout',600.0))
        terminal=(GoalStatus.SUCCEEDED,GoalStatus.ABORTED,GoalStatus.REJECTED,
                  GoalStatus.PREEMPTED,GoalStatus.RECALLED,GoalStatus.LOST)
        while not rospy.is_shutdown():
            state=self.goals.get_state()
            if state in terminal:return state
            if (rospy.Time.now().to_sec()-sim_start>=120.0 or
                    monotonic()-wall_start>=wall_timeout):
                self.goals.cancel_goal()
                return None
            time.sleep(.1)
        self.goals.cancel_goal()
        return None

    def fail(self,error,index=None,item=None):
        status={'phase':'failed','error':error,'stamp':rospy.Time.now().to_sec()}
        if index is not None:status.update(index=index,target=item['name'])
        self.status.publish(String(data=json.dumps(status)))
        rospy.logerr(error)

    def wait_gate_entry(self,gate,started_at):
        """Wait at an approach waypoint before dispatching the crossing goal.

        The guard remains the sole velocity authority.  This client only
        prevents move_base from holding a red-light crossing goal active,
        which would otherwise trip controller_patience while the robot is
        correctly stopped at the line.
        """
        sim_timeout=float(rospy.get_param('~gate_wait_timeout',90.0))
        wall_timeout=float(rospy.get_param('~gate_wait_wall_timeout',600.0))
        sim_start=float(started_at);wall_start=monotonic()
        scan_period=float(rospy.get_param('~gate_scan_period',3.0))
        scan_speed=float(rospy.get_param('~gate_scan_speed',.12))
        self.status.publish(String(data=json.dumps({'phase':'gate_wait','gate':gate,
                                                     'stamp':rospy.Time.now().to_sec()})))
        while not rospy.is_shutdown():
            # The approach goal is already succeeded, so move_base may stop
            # publishing its zero command.  Keep the request channel fresh
            # without bypassing the guard's physical /cmd_vel arbiter.
            now=rospy.Time.now().to_sec()
            with self.lock:guard=dict(self.last_guard or {})
            if gate_entry_ready(guard,gate,now):
                self.status.publish(String(data=json.dumps({'phase':'gate_ready',
                                                             'gate':gate,'stamp':now})))
                return True
            request=Twist()
            # If the guard is healthy and the robot is still before the line,
            # sweep the camera with a bounded in-place yaw. This never moves
            # through the gate; stale/held/recovery states remain zero-speed.
            fresh=(guard.get('guard_valid',False) and
                   not guard.get('safety_hold',False) and
                   not guard.get('lane_recovery',False) and
                   isinstance(scan_period,(int,float)) and scan_period>0.)
            if fresh and scan_speed>0.:
                phase=(monotonic()-wall_start)/scan_period
                request.angular.z=scan_speed if int(phase)%2==0 else -scan_speed
            self.hold_request.publish(request)
            sim_elapsed=now-sim_start
            if sim_elapsed<0:sim_elapsed=0.0
            if sim_elapsed>=sim_timeout or monotonic()-wall_start>=wall_timeout:
                return False
            time.sleep(.1)
        return False

    def run(self):
        if not self.goals.wait_for_server(rospy.Duration(30.0)):
            self.fail('move_base_unavailable');return
        self.stop.publish(String(data='{"mode":"clear"}'))
        for index,item in enumerate(self.layout['route']):
            if rospy.is_shutdown():break
            stamp=rospy.Time.now().to_sec()
            context_id='%d:%s'%(index,item['name'])
            self.publish_context(item,False,stamp,index,context_id)
            self.arm_gate(item)
            self.status.publish(String(data=json.dumps({'index':index,'target':item['name'],
                                                         'phase':'goal','stamp':stamp})))
            self.goals.send_goal(self.make_goal(item))
            state=self.wait_goal(stamp)
            if state is None:
                self.stop.publish(String(data='{"mode":"clear"}'))
                self.fail('goal_timeout:'+item['name'],index,item);return
            if state!=GoalStatus.SUCCEEDED:
                self.stop.publish(String(data='{"mode":"clear"}'))
                self.fail('goal_failed:%s:%s'%(item['name'],state),index,item);return
            # A gate waypoint is the stop-line approach.  Do not dispatch the
            # following past-light goal until the fresh guard permit matches
            # this gate; otherwise move_base can abort during a normal red
            # phase after controller_patience expires.
            if item.get('gate'):
                if not self.wait_gate_entry(item['gate'],rospy.Time.now().to_sec()):
                    self.goals.cancel_all_goals()
                    self.stop.publish(String(data='{"mode":"clear"}'))
                    self.fail('gate_wait_timeout:'+item['gate'],index,item);return
            if item.get('observe'):
                armed=rospy.Time.now().to_sec()
                self.publish_context(item,True,armed,index,context_id)
                if not self.wait_observation(item,armed,context_id):
                    self.publish_context(item,False,rospy.Time.now().to_sec(),index,context_id)
                    self.stop.publish(String(data='{"mode":"clear"}'))
                    self.fail('observation_timeout:'+item['name'],index,item);return
                self.publish_context(item,False,rospy.Time.now().to_sec(),index,context_id)
            self.status.publish(String(data=json.dumps({'index':index,'target':item['name'],
                                                         'phase':'reached','stamp':rospy.Time.now().to_sec()})))
        self.stop.publish(String(data='{"mode":"clear"}'))
        if not rospy.is_shutdown():
            self.status.publish(String(data=json.dumps({'index':len(self.layout['route'])-1,
                'target':self.layout['route'][-1]['name'],'phase':'done',
                'stamp':rospy.Time.now().to_sec()})))


if __name__=='__main__':
    rospy.init_node('move_base_waypoint_client');WaypointClient().run()
