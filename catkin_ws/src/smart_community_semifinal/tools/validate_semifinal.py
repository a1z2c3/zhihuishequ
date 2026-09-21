#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Contract tests + measured synthetic-artwork benchmark. Not Gazebo validation."""
import argparse
import ast
import json
import math
from pathlib import Path
import platform
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
import cv2
import numpy as np

PKG=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PKG/"scripts"))
from semifinal_core import GreenGate,StreetLedger,EvidenceWriter,footprint_corners,front_clearance,braking_distance
from reference_detector import ReferenceDetector,imread,imwrite,iou
from traffic_detector_core import detect_signal


class Contracts(unittest.TestCase):
    def test_green_requires_distinct_frames_and_expires(self):
        gate=GreenGate()
        self.assertFalse(gate.update("one","green",.9,1,1))
        self.assertFalse(gate.update("one","green",.9,1,1))
        self.assertFalse(gate.update("one","green",.9,1.1,1.1))
        self.assertTrue(gate.update("one","green",.9,1.2,1.2))
        self.assertFalse(gate.allow(1.56))

    def test_red_yellow_unknown_reset(self):
        for state in ["red","yellow","unknown"]:
            gate=GreenGate(min_frames=1)
            self.assertTrue(gate.update("one","green",.9,1,1))
            self.assertFalse(gate.update("one",state,.9,1.1,1.1))
            self.assertFalse(gate.allow(1.1))

    def test_clock_reset_and_wrong_light(self):
        gate=GreenGate(min_frames=2)
        gate.update("one","green",.9,5,5)
        self.assertTrue(gate.update("one","green",.9,5.1,5.1))
        self.assertFalse(gate.update("two","green",.9,5.2,5.2))
        self.assertFalse(gate.update("two","green",.9,1,1))
        self.assertFalse(gate.allow(1))

    def test_stale_future_low_and_nan_rejected(self):
        for conf,stamp,now in [(.9,1,2),(.9,2,1),(.2,1,1),(float("nan"),1,1),(.9,1,float("nan"))]:
            gate=GreenGate(min_frames=1)
            self.assertFalse(gate.update("one","green",conf,stamp,now))

    def test_footprint_stop_line_not_robot_center(self):
        self.assertLess(front_clearance((.9,1,0),(1,1),(1,0)),0)
        self.assertGreater(front_clearance((.7,1,0),(1,1),(1,0)),0)
        self.assertLess(front_clearance((1,1.1,-math.pi/2),(1,1),(0,-1)),0)
        self.assertGreater(braking_distance(.4),braking_distance(.2))
        with self.assertRaises(ValueError):braking_distance(.2,deceleration=0)

    def test_same_artwork_two_instances_and_cross_street(self):
        ledger=StreetLedger(min_views=3)
        for frame in range(3):
            ledger.observe("A","same","resident",(1,1),frame)
            ledger.observe("A","same","resident",(1.2,1),frame)
            ledger.observe("B","same","resident",(1,1),frame)
        self.assertEqual(ledger.summary("A")["total"],2)
        self.assertEqual(ledger.summary("B")["total"],1)

    def test_replay_cannot_increase_count(self):
        ledger=StreetLedger(min_views=3)
        for _ in range(20):ledger.observe("A","one","resident",(1,1),1)
        self.assertEqual(ledger.summary("A")["total"],0)

    def test_image_failure_does_not_emit_success(self):
        with tempfile.TemporaryDirectory() as d:
            writer=EvidenceWriter(d,lambda p,img:False,run_id="test")
            with self.assertRaises(IOError):writer.record(1,0,[],None)
            self.assertFalse(Path(writer.path).exists())
            self.assertEqual(writer.sequence,0)

    def test_terminal_image_correspondence(self):
        with tempfile.TemporaryDirectory() as d:
            writer=EvidenceWriter(d,imwrite,run_id="test")
            detections=[{"label":"person_1","category":"resident","confidence":.8,"bbox":[1,1,4,8]}]
            row=json.loads(writer.record(12,1.2,detections,np.zeros((20,20,3),np.uint8)))
            self.assertEqual(row["detections"],detections)
            self.assertTrue((Path(d)/row["image"]).exists())
            self.assertEqual(json.loads(Path(writer.path).read_text(encoding="utf-8")),row)

    def test_xml_python_and_assets(self):
        for suffix in ["*.sdf","*.world","*.dae","*.launch","*.xml"]:
            for path in PKG.rglob(suffix):ET.parse(str(path))
        for path in PKG.rglob("*.py"):
            if "__pycache__" not in str(path):ast.parse(path.read_text(encoding="utf-8"),feature_version=(3,6))
        manifest=json.loads((PKG/"assets/manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(len(manifest["recognition_assets"]),21)
        import hashlib
        for row in manifest["recognition_assets"]:
            self.assertEqual(hashlib.sha256((PKG/"assets"/row["file"]).read_bytes()).hexdigest(),row["sha256"])


def route_check():
    from validate_geometry import check
    return check(PKG)


def benchmark(out):
    detector=ReferenceDetector(PKG/"assets/manifest.json")
    rng=np.random.RandomState(20260921)
    cv2.setRNGSeed(20260921);cv2.setNumThreads(1)
    results=[]
    for item in detector.manifest["recognition_assets"]:
        source=imread(PKG/"assets"/item["file"],cv2.IMREAD_UNCHANGED)
        if source.shape[2]==4:
            alpha=source[:,:,3:4]/255.
            source=(source[:,:,:3]*alpha+210*(1-alpha)).astype(np.uint8)
        for condition,target_long,gain,blur in [("clear",260,1,0),("small",120,1,0),("dim",210,.6,0),("blur",210,1,3)]:
            if item["category"]=="plate":target_long=int(target_long*.65)
            factor=target_long/float(max(source.shape[:2]))
            thumb=cv2.resize(source,None,fx=factor,fy=factor)
            h,w=thumb.shape[:2]
            frame=np.full((480,640,3),75,np.uint8)
            x,y=int(rng.randint(65,400-w//2)),int(rng.randint(55,390-h))
            corners=np.float32([[x,y],[x+w-1,y+4],[x+w-6,y+h-1],[x+4,y+h-4]])
            matrix=cv2.getPerspectiveTransform(np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]]),corners)
            warped=cv2.warpPerspective(thumb,matrix,(640,480))
            mask=cv2.warpPerspective(np.full((h,w),255,np.uint8),matrix,(640,480))
            frame[mask>0]=warped[mask>0]
            frame=np.clip(frame*gain+rng.normal(0,1,frame.shape),0,255).astype(np.uint8)
            if blur:frame=cv2.GaussianBlur(frame,(blur,blur),0)
            start=time.perf_counter();dets=detector.detect(frame);ms=(time.perf_counter()-start)*1000
            correct=[d for d in dets if d["label"]==item["label"] and iou(d["bbox"],[x,y,w,h])>=.5]
            wrong=[d for d in dets if d["label"]!=item["label"]]
            results.append({"label":item["label"],"category":item["category"],"condition":condition,
                            "target_long_pixels":target_long,"correct":bool(correct),"wrong_labels":[d["label"] for d in wrong],"latency_ms":round(ms,2)})
            if condition=="clear" or (not correct and condition=="small"):
                stem=Path(item["file"]).stem+"_"+condition
                imwrite(out/(stem+".png"),detector.annotate(frame,dets))
    negatives=[]
    for i in range(12):
        frame=np.full((480,640,3),int(10+i*15),np.uint8)
        cv2.rectangle(frame,(120,120),(390,320),(120,40,90),-1)
        cv2.putText(frame,"UNKNOWN %d"%i,(140,250),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
        negatives.append(len(detector.detect(frame)))
    lights=[]
    for name in ["red","yellow","green"]:
        for state in ["on","off"]:
            frame=imread(PKG/"assets"/(name+"_"+state+".png"))
            lights.append({"asset":name+"_"+state,"result":detect_signal(frame,[0,0,frame.shape[1],frame.shape[0]])})
    grouped={}
    for c in ["clear","small","dim","blur"]:
        rows=[r for r in results if r["condition"]==c]
        grouped[c]={"correct":sum(r["correct"] for r in rows),"total":len(rows),"wrong_label_frames":sum(bool(r["wrong_labels"]) for r in rows)}
    times=[r["latency_ms"] for r in results]
    return {"scope":"known-artwork synthetic development benchmark; no independent Gazebo or real camera test; not general OCR",
            "seed":20260921,"environment":{"python":platform.python_version(),"opencv":cv2.__version__,"platform":platform.platform()},
            "conditions":grouped,"negative_frames":len(negatives),"negative_false_positive_frames":sum(n>0 for n in negatives),
            "latency_ms":{"median":float(np.median(times)),"p95":float(np.percentile(times,95))},
            "light_asset_results":lights,"cases":results}


def main():
    p=argparse.ArgumentParser();p.add_argument("--out",type=Path,required=True);args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    tests=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Contracts))
    route=route_check()
    report=benchmark(args.out)
    report["contracts"]={"tests":tests.testsRun,"failures":len(tests.failures),"errors":len(tests.errors)}
    report["route_check"]=route
    (args.out/"validation_results.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({k:v for k,v in report.items() if k!="cases"},ensure_ascii=False,indent=2))
    return 0 if tests.wasSuccessful() and route["pass"] else 1


if __name__=="__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
