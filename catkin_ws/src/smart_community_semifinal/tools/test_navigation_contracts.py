#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-free contracts for gate-gated move_base navigation."""
from __future__ import division, unicode_literals
import ast,io,json,os,sys,types,unittest
import xml.etree.ElementTree as ET

PKG=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(PKG,'scripts'))
from navigation_core import gate_entry_ready


class NavigationContracts(unittest.TestCase):
    def test_gate_permit_requires_fresh_matching_guard(self):
        base={'guard_valid':True,'stop_id':'light_1','entry_ready':True,'stamp':10.0}
        self.assertTrue(gate_entry_ready(base,'light_1',10.2))
        self.assertFalse(gate_entry_ready(base,'light_2',10.2))
        self.assertFalse(gate_entry_ready(dict(base,entry_ready=False),'light_1',10.2))
        self.assertFalse(gate_entry_ready(dict(base,guard_valid=False),'light_1',10.2))
        self.assertFalse(gate_entry_ready(base,'light_1',10.36))
        self.assertFalse(gate_entry_ready(dict(base,stamp='bad'),'light_1',10.2))

    def test_client_source_contains_gate_wait_and_neutral_hold(self):
        path=os.path.join(PKG,'scripts','move_base_waypoint_client.py')
        with io.open(path,encoding='utf-8') as stream:source=stream.read()
        ast.parse(source.encode('utf-8'),filename=path)
        for required in ('/semifinal/guard_status','wait_gate_entry',
                         'gate_entry_ready','/semifinal/cmd_vel_requested',
                         'gate_wait_timeout','gate_wait_wall_timeout',
                         'gate_scan_period','gate_scan_speed'):
            self.assertIn(required,source)

    def test_navigation_launch_and_patience_contract(self):
        launch=ET.parse(os.path.join(PKG,'launch','navigation.launch')).getroot()
        remaps=[(r.get('from'),r.get('to')) for r in launch.findall(".//remap")]
        self.assertIn(('cmd_vel','/semifinal/cmd_vel_requested'),remaps)
        names=[n.get('name') for n in launch.findall(".//node")]
        self.assertIn('move_base_waypoint_client',names)
        with io.open(os.path.join(PKG,'config','move_base.yaml'),encoding='utf-8') as stream:
            config=stream.read()
        self.assertIn('controller_patience: 20.0',config)
        self.assertIn('planner_patience: 10.0',config)
        with io.open(os.path.join(PKG,'config','dwa_local_planner.yaml'),encoding='utf-8') as stream:
            dwa=stream.read()
        self.assertIn('yaw_goal_tolerance: 0.35',dwa)
        with io.open(os.path.join(PKG,'config','move_base.yaml'),encoding='utf-8') as stream:
            move_base=stream.read()
        self.assertIn('controller_frequency: 10.0',move_base)
        self.assertIn('oscillation_timeout: 20.0',move_base)
        with io.open(os.path.join(os.path.dirname(PKG),'..','..','setup_and_check.sh'),
                     encoding='utf-8') as stream:
            setup=stream.read()
        self.assertIn('roslaunch --nodes smart_community_semifinal navigation.launch',setup)
        with io.open(os.path.join(os.path.dirname(PKG),'..','..','vm_run_lap.sh'),
                     encoding='utf-8') as stream:
            runner=stream.read()
        self.assertIn('RUN_MODE',runner)
        self.assertIn('navigation.launch',runner)
        self.assertIn('move_base_waypoint_client.py',runner)

    def test_client_reports_only_final_done_and_explicit_failures(self):
        names=('rospy','tf','actionlib','actionlib_msgs','actionlib_msgs.msg',
               'geometry_msgs','geometry_msgs.msg','move_base_msgs',
               'move_base_msgs.msg','std_msgs','std_msgs.msg')
        saved={name:sys.modules.get(name) for name in names}
        class Message(object):
            def __init__(self,data=None):self.data=data
        class Publisher(object):
            def __init__(self):self.messages=[]
            def publish(self,message):self.messages.append(message)
        class Clock(object):
            @staticmethod
            def now():return Clock()
            def to_sec(self):return 42.
        class GoalStatus(object):
            SUCCEEDED=3;ABORTED=4;REJECTED=5;PREEMPTED=2;RECALLED=8;LOST=9
        class Goals(object):
            def __init__(self,states):self.states=list(states);self.cancelled=False
            def wait_for_server(self,duration):return True
            def send_goal(self,goal):pass
            def get_state(self):return self.states.pop(0)
            def cancel_goal(self):self.cancelled=True
        try:
            for name in names:sys.modules[name]=types.ModuleType(str(name))
            ros=sys.modules['rospy'];ros.Time=Clock;ros.Duration=lambda value:value
            ros.get_param=lambda name,default=None:default
            ros.is_shutdown=lambda:False;ros.logerr=lambda message:None
            sys.modules['actionlib_msgs.msg'].GoalStatus=GoalStatus
            sys.modules['geometry_msgs.msg'].PoseStamped=object
            sys.modules['geometry_msgs.msg'].Twist=Message
            sys.modules['move_base_msgs.msg'].MoveBaseAction=object
            sys.modules['move_base_msgs.msg'].MoveBaseGoal=object
            sys.modules['std_msgs.msg'].String=Message
            path=os.path.join(PKG,'scripts','move_base_waypoint_client.py')
            with io.open(path,encoding='utf-8') as stream:source=stream.read()
            namespace={'__name__':'waypoint_client_contract','__file__':path}
            eval(compile(source.encode('utf-8'),path,'exec'),namespace)
            waypoint_client=namespace['WaypointClient']
            route=[{'name':'start'},{'name':'finish'}]
            def run_with(states):
                node=waypoint_client.__new__(waypoint_client)
                node.layout={'route':route};node.goals=Goals(states)
                node.stop=Publisher();node.status=Publisher()
                node.make_goal=lambda item:None
                node.publish_context=lambda *args:None
                node.arm_gate=lambda item:None
                node.time_for_test=42.
                node.run()
                return [json.loads(message.data) for message in node.status.messages]
            success=run_with([GoalStatus.SUCCEEDED,GoalStatus.SUCCEEDED])
            self.assertEqual([item['phase'] for item in success],
                             ['goal','reached','goal','reached','done'])
            self.assertEqual(success[-1]['target'],'finish')
            failed=run_with([GoalStatus.SUCCEEDED,GoalStatus.ABORTED])
            self.assertEqual([item['phase'] for item in failed],
                             ['goal','reached','goal','failed'])
            self.assertIn('goal_failed:finish',failed[-1]['error'])
        finally:
            for name,original in saved.items():
                if original is None:sys.modules.pop(name,None)
                else:sys.modules[name]=original

    def test_wall_clock_waits_do_not_depend_on_ros_rate(self):
        path=os.path.join(PKG,'scripts','move_base_waypoint_client.py')
        with io.open(path,encoding='utf-8') as stream:source=stream.read()
        tree=ast.parse(source.encode('utf-8'),filename=path)
        waits=[node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef)
               and node.name in ('wait_goal','wait_gate_entry','wait_observation')]
        self.assertEqual(len(waits),3)
        for function in waits:
            calls=[node for node in ast.walk(function) if isinstance(node,ast.Call)]
            self.assertTrue(any(isinstance(call.func,ast.Attribute) and
                                call.func.attr=='sleep' and
                                isinstance(call.func.value,ast.Name) and
                                call.func.value.id=='time' for call in calls))
            self.assertFalse(any(isinstance(call.func,ast.Attribute) and
                                 call.func.attr=='sleep' and
                                 isinstance(call.func.value,ast.Name) and
                                 call.func.value.id=='rate' for call in calls))


if __name__=='__main__':
    unittest.main(verbosity=2)
