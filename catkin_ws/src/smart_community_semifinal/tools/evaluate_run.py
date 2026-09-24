#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Independent run evaluator. Gazebo truth is subscribed HERE ONLY, never control."""
from __future__ import division,print_function
import io,json,math,os,sys,time,threading
import rospy,rospkg,tf
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
pkg=rospkg.RosPack().get_path('smart_community_semifinal');sys.path.insert(0,os.path.join(pkg,'scripts'))
from scene_geometry import body_violation,lane_corridor_error,physical_obstacles
from semifinal_core import front_clearance,body_over_stop_line
from runtime_compat import monotonic,makedirs

rospy.init_node('independent_run_evaluator')
out=rospy.get_param('~output_dir',os.path.expanduser('~/semifinal_evaluation'));makedirs(out)
with io.open(os.path.join(pkg,'config/layout.json'),encoding='utf-8') as f:layout=json.load(f)
cycle=layout.get('signal_cycle',{'period_s':28.0,'red_s':10.0,'green_s':15.0,'yellow_s':3.0,
                                 'light_2_offset_s':7.0,
                                 'offsets_s':{'light_1':0.0,'light_2':7.0}})
clock_offsets=cycle.get('offsets_s') or {
    'light_1':0.0,
    'light_2':float(cycle.get('light_2_offset_s',7.0)),
}
obstacles=physical_obstacles(os.path.join(pkg,'worlds/official_semifinal.world'))
physical_layout=dict(layout,lane_safety_margin_m=0.)
lock=threading.RLock();start=monotonic();poses=[];violations=[];crossings=[];red_body_violations=[];images=[];states=[];events=[];frames=[];signals=[];guard=[];localization=[]
safety_margin_warnings=[];minimum_lane_clearance=None
task={};summary={};last_pose_time=-1;last_clear={};red_occupied={};first_time=None


def on_pose(msg):
    global last_pose_time,first_time,minimum_lane_clearance
    now=rospy.Time.now().to_sec()
    if now-last_pose_time<.09 or 'semifinal_bot' not in msg.name:return
    last_pose_time=now
    if first_time is None:first_time=now
    p=msg.pose[msg.name.index('semifinal_bot')];q=p.orientation
    pose=(p.position.x,p.position.y,tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2])
    bad=body_violation(pose,physical_layout,obstacles)
    clearance=-lane_corridor_error(pose,physical_layout)
    reserved=lane_corridor_error(pose,layout)
    with lock:
        poses.append([now]+list(pose))
        if minimum_lane_clearance is None or clearance<minimum_lane_clearance:
            minimum_lane_clearance=clearance
        if bad and len(violations)<100:violations.append({'stamp':now,'obstacle':bad,'pose':pose})
        if reserved>1e-7 and bad is None and len(safety_margin_warnings)<100:
            safety_margin_warnings.append({'stamp':now,'pose':pose,
                                           'reserve_excess_m':reserved,
                                           'physical_clearance_m':clearance})
        for i,line in enumerate(layout['stop_lines']):
            clear=front_clearance(pose,line['point'],line['direction'],margin=0)
            # Spatially restrict the evaluation to the associated corridor.
            near=abs(pose[1]-3.9)<.22 if i==0 else abs(pose[0]-1.95)<.22 and pose[1]<2.1
            offset=float(clock_offsets.get(line['id'],
                                           0.0 if i==0 else
                                           cycle.get('light_2_offset_s',7.0)))
            period=float(cycle.get('period_s',28.0));red=float(cycle.get('red_s',10.0))
            green=float(cycle.get('green_s',15.0))
            phase=(now+offset)%period
            state='red' if phase<red else 'green' if phase<red+green else 'yellow'
            occupied=near and state=='red' and body_over_stop_line(pose,line['point'],line['direction'])
            if occupied and not red_occupied.get(line['id'],False):
                red_body_violations.append({'light_id':line['id'],'stamp':now,'pose':pose,
                                            'true_phase':state})
            red_occupied[line['id']]=occupied
            if near and line['id'] in last_clear and last_clear[line['id']]>=0 and clear<0:
                crossings.append({'light_id':line['id'],'stamp':now,'true_phase':state,'pass':state=='green'})
            last_clear[line['id']]=clear


def store_json(msg,key):
    global task,summary
    try:d=json.loads(msg.data)
    except ValueError:return
    with lock:
        if key=='task':
            if (d.get('target'),d.get('phase'))!=(task.get('target'),task.get('phase')):states.append(d);print('STATE',d)
            task=d
        elif key=='summary':summary=d
        elif key=='events':events.append(d)
        elif key=='frames':frames.append(d)
        elif key=='signals':
            if not signals or (signals[-1].get('light_id'),signals[-1].get('state'))!=(d.get('light_id'),d.get('state')):signals.append(d)
        elif key=='guard':
            if d.get('map_pose') is not None and (not localization or d['stamp']-localization[-1][0]>=.09):
                localization.append([d['stamp']]+d['map_pose'])
            if not guard or (guard[-1].get('mode'),guard[-1].get('reason'))!=(d.get('mode'),d.get('reason')):guard.append(d)


def on_image(msg):
    with lock:images.append([msg.header.stamp.to_sec(),monotonic()-start,msg.header.seq,rospy.Time.now().to_sec()-msg.header.stamp.to_sec()])


rospy.Subscriber('/gazebo/model_states',ModelStates,on_pose,queue_size=1)
rospy.Subscriber('/camera/color/image_raw',Image,on_image,queue_size=1,buff_size=2**24)
for topic,key in [('task_status','task'),('street_summary','summary'),('events','events'),('frame_status','frames'),('visual_signal','signals'),('guard_status','guard')]:
    rospy.Subscriber('/semifinal/'+topic,String,lambda msg,k=key:store_json(msg,k),queue_size=50)
while not rospy.is_shutdown():
    time.sleep(1)
    with lock:
        result={'scope':'Independent Gazebo evaluation, truth isolated from controller',
                'wall_seconds':monotonic()-start,'task':task,'street_summary':summary,'state_transitions':states,
                'poses':poses,'body_violations':violations,
                'safety_margin_warnings':safety_margin_warnings,
                'minimum_lane_clearance_m':minimum_lane_clearance,
                'stop_crossings':crossings,
                'red_body_violations':red_body_violations,'events':events,
                'processed_frames':frames,'signal_transitions':signals,'guard_transitions':guard,
                'camera_samples':images,'localization_samples':localization,'simulated_elapsed':None if first_time is None else rospy.Time.now().to_sec()-first_time}
        with io.open(os.path.join(out,'run_result.tmp'),'w',encoding='utf-8') as f:f.write(json.dumps(result,ensure_ascii=False,indent=2))
        os.rename(os.path.join(out,'run_result.tmp'),os.path.join(out,'run_result.json'))
        if task.get('phase') in ('done','failed') or (first_time is not None and rospy.Time.now().to_sec()-first_time>700) or monotonic()-start>1500:break
print('RUN_EVALUATION_COMPLETE',task)
