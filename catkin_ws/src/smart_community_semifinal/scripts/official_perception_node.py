#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Observation-context image matching, metric PnP ledger and paired evidence."""
from __future__ import division, unicode_literals
import copy,io,json,os,threading,time
import numpy as np
import rospy,tf
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from reference_detector import ReferenceDetector,imwrite
from semifinal_core import EvidenceWriter,StreetLedger
from image_geometry import decode_image,planar_position
from runtime_compat import ros_text,isfinite


class Node(object):
    def __init__(self):
        self.detector=ReferenceDetector(rospy.get_param('~manifest'))
        self.writer=EvidenceWriter(rospy.get_param('~evidence_dir'),imwrite,run_id='objects_%s_%d'%(time.strftime('%Y%m%d_%H%M%S'),os.getpid()))
        with io.open(rospy.get_param('~layout'),encoding='utf-8') as stream:self.layout=json.load(stream)
        self.ledger=StreetLedger();self.listener=tf.TransformListener()
        self.lock=threading.RLock();self.pending=None;self.context={};self.context_id=None;self.K=None
        # Detection, localization and ledger commitment are separate states.
        # A single template match must never satisfy an observation point;
        # stable reference and character evidence are tracked separately.
        self.detected_labels=set();self.localized_labels=set();self.committed_ids=set()
        self.context_instance_frames={};self.plate_frames={}
        # A plate reference match and character OCR are independent evidence
        # channels.  Route completion uses the former; the latter remains an
        # explicit quality signal and never gets silently replaced by the
        # reference label.
        self.plate_reference_frames={};self.plate_ocr_votes={};self.plate_results={};self.last_stamp=None
        self.last_received_stamp=None;self.last_processed_stamp=None
        self.context_started_wall=time.time();self.last_status_wall=0.
        self.images=rospy.Publisher('/semifinal/annotated',Image,queue_size=1)
        self.events=rospy.Publisher('/semifinal/events',String,queue_size=10)
        self.status=rospy.Publisher('/semifinal/frame_status',String,queue_size=10)
        self.summary=rospy.Publisher('/semifinal/street_summary',String,queue_size=1,latch=True)
        self.plate_summary=rospy.Publisher('/semifinal/plate_summary',String,queue_size=1,latch=True)
        rospy.Subscriber('/semifinal/observation',String,self.configure,queue_size=1)
        self.camera_info_topic=rospy.get_param('~camera_info_topic','/camera/color/camera_info')
        rospy.Subscriber(self.camera_info_topic,CameraInfo,self.calibration,queue_size=1)
        self.image_topic=rospy.get_param('~image_topic','/camera/color/image_raw')
        rospy.Subscriber(self.image_topic,Image,self.receive,queue_size=1,buff_size=2**24)

    def calibration(self,msg):
        with self.lock:self.K=list(msg.K)

    def configure(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('observation context is not an object')
            context_id=data.get('context_id')
            with self.lock:
                old_armed=self.context.get('armed_at',self.context.get('stamp'))
                new_active=bool(data.get('active'))
                rearmed=(new_active and data.get('armed_at',data.get('stamp'))!=old_armed)
                if context_id!=self.context_id or not new_active or rearmed:
                    # Drop queued frames from the previous view before arming
                    # the new one.  The message is latched, so this also makes
                    # startup deterministic when perception joins late.
                    self.pending=None;self.detected_labels=set();self.localized_labels=set()
                    self.committed_ids=set();self.context_instance_frames={}
                    self.plate_frames={};self.plate_reference_frames={};self.plate_ocr_votes={}
                    self.last_stamp=None;self.last_received_stamp=None
                    self.last_processed_stamp=None;self.context_started_wall=time.time()
                    self.context_id=context_id
                self.context=data
        except ValueError:
            with self.lock:self.pending=None;self.context={};self.context_id=None

    def receive(self,msg):
        with self.lock:
            context=dict(self.context);stamp=msg.header.stamp.to_sec()
            armed=float(context.get('armed_at',context.get('stamp',0.)))
            warmup=float(context.get('warmup_s',.35))
            if (context.get('active') and context.get('context_id')==self.context_id and
                    stamp>=armed+max(0.,warmup) and
                    (self.last_received_stamp is None or stamp>self.last_received_stamp)):
                self.last_received_stamp=stamp
                self.pending=(msg,context,self.K)

    @staticmethod
    def _accepts(context,detection):
        expected=context.get('expected_category')
        category=detection.get('category')
        if expected=='person':return category in ('resident','visitor')
        if expected in ('resident','visitor','plate'):return category==expected
        return category in ('resident','visitor','plate')

    @staticmethod
    def _reject(detection,reason):
        detection['commit_state']='rejected';detection['rejection_reason']=reason

    def _ocr_consensus(self,label):
        votes=getattr(self,'plate_ocr_votes',{}).get(label,{})
        min_views=getattr(getattr(self,'ledger',None),'min_views',3)
        chars=[];pending=[]
        for slot in range(7):
            counts=votes.get(slot,{})
            ranked=sorted(counts.items(),key=lambda item:(item[1],item[0]),reverse=True)
            if (not ranked or ranked[0][1]<min_views or
                    ranked[0][1]*2<=sum(counts.values())):
                chars.append('?');pending.append(slot+1)
            else:
                chars.append(ranked[0][0])
        return {'text':''.join(chars),'complete':not pending,
                'pending_slots':pending}

    def _record_ocr_vote(self,label,ocr):
        votes=getattr(self,'plate_ocr_votes',{})
        self.plate_ocr_votes=votes
        text=ocr.get('text','') if isinstance(ocr,dict) else ''
        if len(text)!=7:return self._ocr_consensus(label)
        for slot,char in enumerate(text):
            if char=='?':continue
            counts=votes.setdefault(label,{}).setdefault(slot,{})
            counts[char]=counts.get(char,0)+1
        return self._ocr_consensus(label)

    @staticmethod
    def _stable_count(context,committed,plate_frames,instance_frames,
                      reference_frames=None):
        if context.get('expected_category')=='plate':
            frames=reference_frames if reference_frames is not None else plate_frames
            return max([len(frames.get(d['label'],set()))
                        for d in committed if d['category']=='plate' and
                        not d.get('unexpected_label')] or [0])
        return max([len(instance_frames.get(d['instance_id'],set()))
                    for d in committed if d.get('instance_id')] or [0])

    @staticmethod
    def _inside(point,polygon):
        x,y=point;inside=False
        for i in range(len(polygon)):
            x1,y1=polygon[i];x2,y2=polygon[(i+1)%len(polygon)]
            if ((y1>y)!=(y2>y)) and x < (x2-x1)*(y-y1)/float(y2-y1)+x1:
                inside=not inside
        return inside

    def _in_expected_region(self,street,point):
        key='a_polygon' if street=='A' else 'b_polygon' if street=='B' else None
        polygon=self.layout.get(key) if key else None
        return bool(polygon and self._inside(point,polygon))

    def _commit_detection(self,detection,context,K,msg_stamp,msg):
        """Commit one detection only after all checks for this view pass."""
        if not self._accepts(context,detection):
            self._reject(detection,'unexpected_category');return False
        expected_label=context.get('expected_label')
        if expected_label and detection.get('label')!=expected_label:
            detection['unexpected_label']=True
            detection['expected_label']=expected_label
        if detection.get('category')=='plate':
            # Plates are physical objects too.  When the route supplies the
            # current parking bay, require an independent metric position
            # before allowing this reference match into the observation
            # quorum.  The expected text remains a disagreement signal, not
            # the spatial gate.
            target_xy=context.get('target_object_xy',context.get('target_xy'))
            if target_xy is not None:
                if K is None:
                    self._reject(detection,'camera_calibration_unavailable');return False
                try:
                    position=planar_position(detection,K)
                    if position is None:raise ValueError('pnp_rejected')
                    self.listener.waitForTransform('map',msg.header.frame_id,
                                                   msg.header.stamp,rospy.Duration(.05))
                    xyz,q=self.listener.lookupTransform('map',msg.header.frame_id,
                                                        msg.header.stamp)
                    p=np.dot(tf.transformations.quaternion_matrix(q),
                              position['camera_xyz']+[1.])[:3]+xyz
                    if not np.isfinite(p).all():raise ValueError('non_finite_map_position')
                    tolerance=float(context.get('plate_position_tolerance_m',.26))
                    distance=float(np.linalg.norm(np.asarray(p[:2],dtype=float)-
                                                   np.asarray(target_xy,dtype=float)))
                    if not isfinite(distance) or tolerance<=0. or distance>tolerance:
                        self._reject(detection,'outside_expected_plate_bay');return False
                    detection['map_position']=p.tolist()
                    detection['position_method']=position['method']
                    detection['bay_distance_m']=round(distance,4)
                except tf.Exception:
                    self._reject(detection,'tf_unavailable');return False
                except (ValueError,TypeError,IndexError):
                    self._reject(detection,'plate_position_rejected');return False
            ocr=detection.get('ocr_result') or {}
            detection['ocr_result']=ocr
            detection['character_ocr']=bool(ocr.get('complete',False))
            detection['recognition_mode']='reference_match_plus_character_ocr'
            if ocr.get('error'):
                detection['ocr_status']='error:'+ocr['error']
            elif not ocr.get('complete'):
                text=ocr.get('text','')
                pending=[str(i+1) for i,char in enumerate(text)
                         if char=='?'] if len(text)==7 else [str(i+1) for i in range(7)]
                detection['ocr_status']='uncertain:slots='+','.join(pending)
            elif ocr.get('text')!=detection.get('label'):
                # Keep the independently verified reference detection for
                # route completion, but expose disagreement instead of
                # silently presenting a guessed character string.
                detection['character_ocr']=False
                detection['ocr_status']='disagrees:reference_mismatch'
            else:
                detection['ocr_status']='recognized_and_verified'
            consensus=(self._ocr_consensus(detection['label'])
                       if detection.get('unexpected_label') else
                       self._record_ocr_vote(detection['label'],ocr))
            detection['ocr_consensus']=consensus
            detection['ocr_pending_slots']=consensus['pending_slots']
            if detection.get('unexpected_label'):
                detection['ocr_display_status']='unexpected_plate'
            elif not consensus['complete']:
                detection['ocr_display_status']='pending'
            elif consensus['text']!=detection.get('label'):
                detection['ocr_display_status']='disagrees'
            else:
                detection['ocr_display_status']='verified'
                detection['ocr_display_text']=consensus['text']
            if not detection.get('unexpected_label'):
                self.plate_results=getattr(self,'plate_results',{})
                self.plate_results[detection['label']]=dict(consensus)
            if not detection.get('unexpected_label'):
                self.plate_reference_frames=getattr(self,'plate_reference_frames',{})
                self.plate_reference_frames.setdefault(detection['label'],set()).add(msg_stamp)
            if (detection['ocr_status']=='recognized_and_verified' and
                    not detection.get('unexpected_label')):
                self.plate_frames.setdefault(detection['label'],set()).add(msg_stamp)
            detection['commit_state']='committed_reference_match'
            detection['commit_key']='plate:'+detection['label']
            return True
        if K is None:
            self._reject(detection,'camera_calibration_unavailable');return False
        try:
            position=planar_position(detection,K)
        except Exception:
            # One malformed homography must be recorded as a rejected
            # detection, not discard the rest of the frame transaction.
            position=None
        if position is None:
            self._reject(detection,'pnp_rejected');return False
        if not context.get('street'):
            self._reject(detection,'missing_expected_region');return False
        try:
            self.listener.waitForTransform('map',msg.header.frame_id,
                                           msg.header.stamp,rospy.Duration(.05))
            xyz,q=self.listener.lookupTransform('map',msg.header.frame_id,
                                                msg.header.stamp)
            p=np.dot(tf.transformations.quaternion_matrix(q),
                     position['camera_xyz']+[1.])[:3]+xyz
            if not np.isfinite(p).all():raise ValueError('non_finite_map_position')
            if not self._in_expected_region(context['street'],p[:2]):
                self._reject(detection,'outside_expected_region');return False
            detection['map_position']=p.tolist();detection['position_method']=position['method']
            instance=self.ledger.observe(context['street'],detection['label'],
                                         detection['category'],p[:2],msg_stamp)
            detection['instance_id']=instance;detection['commit_state']='committed'
            self.localized_labels.add(detection['label']);self.committed_ids.add(instance)
            self.context_instance_frames.setdefault(instance,set()).add(msg_stamp)
            return True
        except tf.Exception:
            self._reject(detection,'tf_unavailable');return False
        except (ValueError,TypeError):
            self._reject(detection,'invalid_map_position');return False

    def run(self):
        rate=rospy.Rate(3)
        while not rospy.is_shutdown():
            with self.lock:pending,self.pending=self.pending,None
            if pending is None:
                with self.lock:
                    context=dict(self.context);now_wall=time.time()
                    active=bool(context.get('active'))
                    emit=active and now_wall-self.last_status_wall>=1.0
                    if emit:self.last_status_wall=now_wall
                    received=self.last_received_stamp;processed=self.last_processed_stamp
                    started=self.context_started_wall
                if emit:
                    reason='camera_no_frame' if received is None and now_wall-started>5.0 else 'waiting_for_frame'
                    self.status.publish(String(data=json.dumps({
                        'view':context.get('view'),'context_id':context.get('context_id'),
                        'stamp':rospy.Time.now().to_sec(),'count':0,'committed_count':0,
                        'complete':False,'heartbeat':True,'reason':reason,
                        'last_received_stamp':received,'last_processed_stamp':processed,
                        'wall_seconds_since_context':now_wall-started})))
                rate.sleep();continue
            msg,context,K=pending;stamp=msg.header.stamp.to_sec()
            with self.lock:
                ledger_before=copy.deepcopy(self.ledger.instances)
                plates_before=copy.deepcopy(self.plate_frames)
                reference_before=copy.deepcopy(self.plate_reference_frames)
                ocr_votes_before=copy.deepcopy(self.plate_ocr_votes)
                plate_results_before=copy.deepcopy(self.plate_results)
                context_frames_before=copy.deepcopy(self.context_instance_frames)
            try:
                age=(rospy.Time.now()-msg.header.stamp).to_sec()
                if not 0<=age<.7:raise ValueError('stale observation')
                if context.get('context_id')!=self.context_id:raise ValueError('context switched')
                frame=decode_image(msg);start=time.time()
                expected=context.get('expected_category')
                categories=('resident','visitor') if expected=='person' else \
                           ('plate',) if expected=='plate' else None
                detections=self.detector.detect(frame,categories=categories)
                with self.lock:
                    if context.get('context_id')!=self.context_id:
                        raise ValueError('context switched')
                    self.last_processed_stamp=stamp
                    self.detected_labels.update(d['label'] for d in detections)
                    committed=[];reasons=[]
                    for detection in detections:
                        if self._commit_detection(detection,context,K,stamp,msg):
                            committed.append(detection)
                        elif detection.get('rejection_reason'):
                            reasons.append(detection['rejection_reason'])
                # Completion requires temporal stability of the *reference*
                # identity and a current matching commit.  Character OCR is
                # reported separately: a transient OCR rejection must not
                # discard an otherwise stable physical observation, while an
                # uncertain character is never replaced by the reference text.
                stable_count=self._stable_count(
                    context,committed,self.plate_reference_frames,
                    self.context_instance_frames)
                ocr_stable_count=(max([
                    len(self.plate_frames.get(d['label'],set()))
                    for d in committed if d['category']=='plate' and
                    not d.get('unexpected_label')] or [0])
                    if context.get('expected_category')=='plate' else None)
                complete=bool(committed) and stable_count>=self.ledger.min_views
                completion_mode='reference_quorum' if complete else 'pending'
                ocr_complete=(ocr_stable_count is not None and
                              ocr_stable_count>=self.ledger.min_views)
                if (complete and context.get('expected_category')=='plate' and
                        not ocr_complete):
                    completion_mode='reference_quorum_ocr_pending'
                annotated=self.detector.annotate(frame,detections)
                out=Image();out.header=msg.header;out.height,out.width=annotated.shape[:2];out.encoding='bgr8';out.step=out.width*3;out.data=annotated.tobytes();self.images.publish(out)
                if detections:
                    line=self.writer.record(msg.header.seq,stamp,detections,annotated)
                    event=json.loads(line);event['observation']=context
                    event['committed_count']=len(committed);event['rejection_reasons']=reasons
                    event_line=json.dumps(event,ensure_ascii=False)
                    self.events.publish(String(data=ros_text(event_line)));rospy.loginfo(ros_text(event_line))
                self.status.publish(String(data=json.dumps({'view':context['view'],'context_id':context.get('context_id'),
                    'stamp':stamp,'count':len(detections),'committed_count':len(committed),
                     'stable_count':stable_count,
                     'reference_stable_count':stable_count,
                     'ocr_stable_count':ocr_stable_count,
                     'ocr_complete':ocr_complete,
                     'completion_mode':completion_mode,
                    'rejected_count':len(detections)-len(committed),'complete':complete,
                    'rejection_reasons':reasons,'latency_wall_seconds':time.time()-start,
                    'last_received_stamp':self.last_received_stamp,
                    'last_processed_stamp':self.last_processed_stamp})))
                summaries={street:self.ledger.summary(street) for street in ('A','B')}
                self.summary.publish(String(data=json.dumps(summaries)))
                plate_summary=dict(self.plate_results)
                self.plate_summary.publish(String(data=json.dumps(
                    plate_summary,ensure_ascii=False)))
                rospy.loginfo('Street ledger %s'%json.dumps(summaries))
            except Exception as exc:
                # Evidence writing is part of the observation transaction.  If
                # the paired PNG/JSON record fails, do not leave a successful
                # ledger or plate quorum behind for a later frame to inherit.
                with self.lock:
                    # A context switch owns the new transaction.  Never put
                    # the old view's plate/quorum state back into it.
                    if self.context_id==context.get('context_id'):
                        self.ledger.instances=ledger_before
                        self.plate_frames=plates_before
                        self.plate_reference_frames=reference_before
                        self.plate_ocr_votes=ocr_votes_before
                        self.plate_results=plate_results_before
                        self.context_instance_frames=context_frames_before
                rospy.logerr_throttle(2,'Perception failed: %s'%exc)
                if context.get('active'):
                    self.status.publish(String(data=json.dumps({'view':context.get('view'),
                        'context_id':context.get('context_id'),'stamp':stamp,'count':0,
                        'committed_count':0,'complete':False,'error':str(exc)})))
            rate.sleep()


if __name__=='__main__':
    rospy.init_node('official_perception');Node().run()
