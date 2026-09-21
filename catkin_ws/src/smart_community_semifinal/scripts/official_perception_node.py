#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Observation-context image matching, metric PnP ledger and paired evidence."""
from __future__ import division, unicode_literals
import json,os,threading,time
import numpy as np
import rospy,tf
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from reference_detector import ReferenceDetector,imwrite
from semifinal_core import EvidenceWriter,StreetLedger
from image_geometry import decode_image,planar_position
from runtime_compat import ros_text


class Node(object):
    def __init__(self):
        self.detector=ReferenceDetector(rospy.get_param('~manifest'))
        self.writer=EvidenceWriter(rospy.get_param('~evidence_dir'),imwrite,run_id='objects_%s_%d'%(time.strftime('%Y%m%d_%H%M%S'),os.getpid()))
        self.ledger=StreetLedger();self.listener=tf.TransformListener()
        self.lock=threading.RLock();self.pending=None;self.context={};self.K=None
        self.view=None;self.seen_labels=set();self.seen_instances=set()
        self.images=rospy.Publisher('/semifinal/annotated',Image,queue_size=1)
        self.events=rospy.Publisher('/semifinal/events',String,queue_size=10)
        self.status=rospy.Publisher('/semifinal/frame_status',String,queue_size=10)
        self.summary=rospy.Publisher('/semifinal/street_summary',String,queue_size=1,latch=True)
        rospy.Subscriber('/semifinal/observation',String,self.configure,queue_size=1)
        rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.calibration,queue_size=1)
        rospy.Subscriber(rospy.get_param('~image_topic','/camera/color/image_raw'),Image,self.receive,queue_size=1,buff_size=2**24)

    def calibration(self,msg):
        with self.lock:self.K=list(msg.K)

    def configure(self,msg):
        try:
            with self.lock:self.context=json.loads(msg.data)
        except ValueError:
            with self.lock:self.context={}

    def receive(self,msg):
        with self.lock:
            if self.context.get('active') and msg.header.stamp.to_sec()>=self.context['stamp']+.35:
                self.pending=(msg,dict(self.context),self.K)

    def run(self):
        rate=rospy.Rate(3)
        while not rospy.is_shutdown():
            with self.lock:pending,self.pending=self.pending,None
            if pending is None:rate.sleep();continue
            msg,context,K=pending;stamp=msg.header.stamp.to_sec()
            try:
                if not 0<=(rospy.Time.now()-msg.header.stamp).to_sec()<.7:raise ValueError('stale observation')
                frame=decode_image(msg);start=time.time();detections=self.detector.detect(frame)
                if context['view']!=self.view:
                    self.view=context['view'];self.seen_labels=set();self.seen_instances=set()
                for detection in detections:
                    if K is None or context.get('street') is None or detection['category']=='plate':continue
                    position=planar_position(detection,K)
                    if position is None:continue
                    try:
                        xyz,q=self.listener.lookupTransform('map',msg.header.frame_id,msg.header.stamp)
                        p=np.dot(tf.transformations.quaternion_matrix(q),position['camera_xyz']+[1.])[:3]+xyz
                        detection['map_position']=p.tolist();detection['position_method']=position['method']
                        detection['instance_id']=self.ledger.observe(context['street'],detection['label'],detection['category'],p[:2],stamp)
                        self.seen_instances.add(detection['instance_id'])
                    except tf.Exception:pass
                people=[d for d in detections if d['category']!='plate']
                self.seen_labels.update(d['label'] for d in people)
                confirmed=[i for i in self.ledger.instances if i['street']==context.get('street') and len(i['frames'])>=self.ledger.min_views]
                confirmed_ids=set(i['instance_id'] for i in confirmed)
                confirmed_labels=set(i['label'] for i in confirmed if i['instance_id'] in self.seen_instances)
                complete=bool(detections) and (context.get('street') is None or
                         (self.seen_labels<=confirmed_labels and self.seen_instances<=confirmed_ids))
                annotated=self.detector.annotate(frame,detections)
                out=Image();out.header=msg.header;out.height,out.width=annotated.shape[:2];out.encoding='bgr8';out.step=out.width*3;out.data=annotated.tobytes();self.images.publish(out)
                if detections:
                    line=self.writer.record(msg.header.seq,stamp,detections,annotated)
                    event=json.loads(line);event['observation']=context
                    self.events.publish(String(data=ros_text(json.dumps(event,ensure_ascii=False))));rospy.loginfo(ros_text(line))
                self.status.publish(String(data=json.dumps({'view':context['view'],'stamp':stamp,'count':len(detections),'complete':complete,'latency_wall_seconds':time.time()-start})))
                summaries={street:self.ledger.summary(street) for street in ('A','B')}
                self.summary.publish(String(data=json.dumps(summaries)))
                rospy.loginfo('Street ledger %s'%json.dumps(summaries))
            except Exception as exc:rospy.logerr_throttle(2,'Perception failed: %s'%exc)
            rate.sleep()


if __name__=='__main__':
    rospy.init_node('official_perception');Node().run()
