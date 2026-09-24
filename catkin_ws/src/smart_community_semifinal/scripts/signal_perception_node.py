#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dedicated image-driven ROI publisher and signal classifier with source stamps."""
from __future__ import division, unicode_literals
import io,json,math,threading,time,os
import numpy as np
import rospy,tf
from sensor_msgs.msg import Image
from std_msgs.msg import String
from image_geometry import decode_image
from signal_locator import locate_signal
from traffic_detector_core import detect_signal
from scene_geometry import camera_from_urdf,card_corners,project
from reference_detector import imwrite
from semifinal_core import EvidenceWriter


class SignalNode(object):
    def __init__(self):
        with io.open(rospy.get_param('~layout'),encoding='utf-8') as f:self.layout=json.load(f)
        self.camera=camera_from_urdf(rospy.get_param('~urdf'));self.listener=tf.TransformListener()
        self.signal_geometry=rospy.get_param('~signal_geometry',self.layout.get('signal_geometry',{}))
        self.active=None;self.pending=None;self.lock=threading.RLock();self.last_saved=None
        self.stable_light=None;self.stable_state='unknown';self.green_candidate=0
        self.publisher=rospy.Publisher('/semifinal/visual_signal',String,queue_size=1)
        self.roi=rospy.Publisher('/semifinal/signal_roi',String,queue_size=1)
        self.annotated=rospy.Publisher('/semifinal/signal_annotated',Image,queue_size=1)
        self.writer=EvidenceWriter(rospy.get_param('~evidence_dir'),imwrite,run_id='signal_%s_%d'%(time.strftime('%Y%m%d_%H%M%S'),os.getpid()))
        rospy.Subscriber('/semifinal/active_stop',String,self.configure,queue_size=1)
        self.image_topic=rospy.get_param('~image_topic','/camera/color/image_raw')
        rospy.Subscriber(self.image_topic,Image,self.receive,queue_size=1,buff_size=2**24)

    def configure(self,msg):
        try:
            data=json.loads(msg.data)
            with self.lock:
                new_active=data.get('id')
                if new_active!=self.active:
                    self.pending=None
                    self.stable_light=new_active;self.stable_state='unknown';self.green_candidate=0
                self.active=new_active
        except ValueError:
            with self.lock:
                self.active=None;self.stable_light=None;self.stable_state='unknown';self.green_candidate=0

    def stabilize(self,result):
        """Apply fail-safe temporal hysteresis before policy arbitration.

        Dangerous evidence (red, yellow or unknown) is immediate.  Green is
        released only after three consecutive fresh classifications; while
        waiting, a previously known red/yellow state is held so the gate can
        still witness the real transition instead of seeing a false unknown.
        """
        light=result.get('light_id');raw=result.get('state','unknown')
        if light!=self.stable_light:
            self.stable_light=light;self.stable_state='unknown';self.green_candidate=0
        result['raw_state']=raw
        if raw=='green':
            self.green_candidate+=1
            if self.green_candidate>=3:self.stable_state='green'
            elif self.stable_state not in ('red','yellow'):
                self.stable_state='unknown'
        else:
            self.green_candidate=0
            self.stable_state=raw if raw in ('red','yellow','unknown') else 'unknown'
        result['state']=self.stable_state
        result['temporal_confirmed']=(self.stable_state=='green' and self.green_candidate>=3)
        return result

    def receive(self,msg):
        with self.lock:self.pending=(msg,self.active)

    def process(self,msg,light_id):
        stamp=msg.header.stamp.to_sec();now=rospy.Time.now().to_sec()
        result={'light_id':light_id,'state':'unknown','confidence':0.0,'stamp':stamp,'frame_id':msg.header.seq}
        if light_id is None or not 0<=now-stamp<=.35:return result
        lamp=next((r for r in self.layout['lights'] if r['id']==light_id),None)
        if lamp is None:return result
        try:
            xyz,q=self.listener.lookupTransform('map','base_footprint',msg.header.stamp)
            yaw=tf.transformations.euler_from_quaternion(q)[2]
            points=card_corners(lamp['xy'][0],lamp['xy'][1],.34,math.radians(lamp['yaw']),.64,.14)
            quad=project(points,(xyz[0],xyz[1],yaw),self.camera)
            if quad is None:return result
            lo=quad.min(axis=0);hi=quad.max(axis=0)
            frame=decode_image(msg);candidate=locate_signal(frame,[lo[0],lo[1],hi[0]-lo[0],hi[1]-lo[1]],self.signal_geometry)
            if candidate is None:return result
            result.update(detect_signal(frame,candidate['roi']))
            self.roi.publish(String(data=json.dumps({'light_id':light_id,'roi':candidate['roi'],'stamp':stamp,'source':'visual_circle_and_housing'})))
            # Keep the signal evidence on its own fast channel.
            import cv2
            x,y,w,h=candidate['roi'];cv2.rectangle(frame,(x,y),(x+w,y+h),(0,240,240),2)
            cv2.putText(frame,'%s %s %.2f'%(light_id,result['state'],result['confidence']),(x,max(20,y-8)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,240,240),2)
            out=Image();out.header=msg.header;out.height,out.width=frame.shape[:2];out.encoding='bgr8';out.step=out.width*3;out.data=frame.tobytes();self.annotated.publish(out)
            key=(light_id,result['state'])
            if key!=self.last_saved:
                line=self.writer.record(msg.header.seq,stamp,[{'category':'traffic_light','label':result['state'],'confidence':result['confidence'],'bbox':candidate['roi']}],frame)
                rospy.loginfo(line);self.last_saved=key
        except (tf.Exception,ValueError,IOError) as exc:rospy.logwarn_throttle(3,'Signal rejected: %s'%exc)
        return result

    def run(self):
        rate=rospy.Rate(15)
        while not rospy.is_shutdown():
            with self.lock:pending,self.pending=self.pending,None
            if pending is not None:
                result=self.stabilize(self.process(*pending))
                self.publisher.publish(String(data=json.dumps(result)))
            rate.sleep()


if __name__=='__main__':
    rospy.init_node('signal_perception');SignalNode().run()
