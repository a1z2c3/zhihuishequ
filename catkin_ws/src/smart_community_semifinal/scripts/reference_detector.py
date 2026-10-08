#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""通过特征匹配和稳健单应估计识别参考素材。"""
from __future__ import division, unicode_literals
import io
import json
import os
from runtime_compat import text_type
import cv2
import numpy as np


REJECTIONS = []

MATCH_MAX_DISTANCE = 60
MATCH_MAX_RATIO = 0.72


def drain_rejections():
    drained = list(REJECTIONS)
    del REJECTIONS[:]
    return drained


def imread(path, flags=cv2.IMREAD_COLOR):
    return cv2.imdecode(np.fromfile(text_type(path), dtype=np.uint8), flags)


def imwrite(path, image):
    ok, buffer = cv2.imencode(os.path.splitext(text_type(path))[1], image)
    if ok:
        buffer.tofile(text_type(path))
    return ok


def iou(a, b):
    x, y = max(a[0], b[0]), max(a[1], b[1])
    w, h = max(0, min(a[0]+a[2], b[0]+b[2])-x), max(0, min(a[1]+a[3], b[1]+b[3])-y)
    inter = w*h
    return inter / max(1, a[2]*a[3]+b[2]*b[3]-inter)


class ReferenceDetector(object):
    def __init__(self, manifest_path, min_inliers=10, min_ratio=0.40):
        self.manifest_path = text_type(manifest_path)
        with io.open(self.manifest_path, encoding="utf-8") as stream:
            self.manifest = json.load(stream)
        self.orb = cv2.ORB_create(nfeatures=2200, scaleFactor=1.15, nlevels=12,
                                 edgeThreshold=8, fastThreshold=7)
        self.match_distance = MATCH_MAX_DISTANCE
        self.match_ratio = MATCH_MAX_RATIO
        self.min_inliers, self.min_ratio = min_inliers, min_ratio
        self.references = []
        plate_templates=[]
        for item in self.manifest["recognition_assets"]:
            if item["category"] not in ("resident", "visitor", "plate"):
                continue
            src = imread(os.path.join(os.path.dirname(self.manifest_path), item["file"]), cv2.IMREAD_UNCHANGED)
            if src is None:
                raise IOError(item["file"])
            if src.ndim == 3 and src.shape[2] == 4:
                alpha = src[:, :, 3:4] / 255.0
                src = (src[:, :, :3]*alpha + 210*(1-alpha)).astype(np.uint8)
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            if item['category']=='plate':plate_templates.append((gray,item['label']))
            scale = 480.0 / max(gray.shape)
            gray = cv2.resize(gray, None, fx=scale, fy=scale)
            kp, desc = self.orb.detectAndCompute(gray, None)
            self.references.append((item, gray, kp, desc))
        from plate_ocr import PlateCharacterRecognizer
        self.plate_ocr=PlateCharacterRecognizer(plate_templates)

    @staticmethod
    def _confidence_from_evidence(category, raw_score, ratio,
                                  inlier_factor, spread_factor,
                                  photometric_correlation, photometric_bound):
        """计算展示用匹配分数，不降低接收门槛。"""
        clamp=lambda value:max(0.,min(1.,float(value)))
        ratio=clamp(ratio);inlier_factor=clamp(inlier_factor)
        spread_factor=clamp(spread_factor)
        bound=clamp(photometric_bound)
        photo=clamp((float(photometric_correlation)-bound)/max(1e-6,1.-bound))
        evidence=(.35*ratio+.25*inlier_factor+.20*spread_factor+.20*photo)
        return round(max(clamp(raw_score), clamp(evidence)), 4)

    def detect(self, frame, categories=None):
        allowed=set(categories) if categories else None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, desc = self.orb.detectAndCompute(gray, None)
        if desc is None or len(kp) < self.min_inliers:
            return []
        matcher = cv2.FlannBasedMatcher(dict(algorithm=6, table_number=6,
                                            key_size=12, multi_probe_level=1), dict(checks=32))
        matcher.add([desc]); matcher.train()
        output = []
        for item, reference, ref_kp, ref_desc in self.references:
            if allowed is not None and item['category'] not in allowed:
                continue
            if ref_desc is None:
                continue
            pairs = matcher.knnMatch(ref_desc, k=2)
            good = [p[0] for p in pairs if len(p) == 2
                    and p[0].distance < self.match_distance
                    and p[0].distance < self.match_ratio*p[1].distance]
            if len(good) < self.min_inliers:
                continue
            unique = {}
            for m in good:
                if m.trainIdx not in unique or m.distance < unique[m.trainIdx].distance:
                    unique[m.trainIdx] = m
            good = list(unique.values())
            if len(good) < self.min_inliers:
                continue
            source = np.float32([ref_kp[m.queryIdx].pt for m in good])
            target = np.float32([kp[m.trainIdx].pt for m in good])
            matrix, mask = cv2.findHomography(source, target, cv2.RANSAC, 3.0)
            if matrix is None or mask is None or not np.isfinite(matrix).all():
                continue
            n_inliers = int(mask.sum())
            ratio = n_inliers / len(good)
            if n_inliers < self.min_inliers or ratio < self.min_ratio:
                continue
            h, w = reference.shape
            quad = cv2.perspectiveTransform(np.float32([[[0,0],[w-1,0],[w-1,h-1],[0,h-1]]]), matrix)[0]
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
                continue
            area = abs(cv2.contourArea(quad))
            if area < 200 or area > frame.shape[0]*frame.shape[1]*0.9:
                continue
            if np.any(quad[:,0] < -10) or np.any(quad[:,1] < -10) or np.any(quad[:,0] > frame.shape[1]+10) or np.any(quad[:,1] > frame.shape[0]+10):
                continue
            spread = abs(cv2.contourArea(cv2.convexHull(source[mask.ravel().astype(bool)]))) / (w*h)
            if spread < 0.025:
                continue
            try:
                inverse = np.linalg.inv(matrix)
            except (np.linalg.LinAlgError, ValueError, TypeError):
                continue
            rectified = cv2.warpPerspective(gray, inverse, (w,h))
            pad_y,pad_x = max(1,int(h*.08)),max(1,int(w*.08))
            a=reference[pad_y:-pad_y,pad_x:-pad_x].astype(np.float32).ravel()
            b=rectified[pad_y:-pad_y,pad_x:-pad_x].astype(np.float32).ravel()
            a-=a.mean();b-=b.mean()
            correlation=float(np.dot(a,b)/max(1e-6,np.linalg.norm(a)*np.linalg.norm(b)))
            bound = 0.76 if item["category"] == "plate" else 0.50
            if correlation < bound:
                REJECTIONS.append({"label": item["label"], "category": item["category"],
                                   "correlation": round(correlation, 4),
                                   "inliers": int(n_inliers), "bound": bound})
                continue
            ocr=None
            if item['category']=='plate':
                try:
                    ocr=self.plate_ocr.recognize(rectified)
                except Exception as exc:
                    ocr={'text':'???????','characters':[], 'confidence':0.,
                         'complete':False,'error':type(exc).__name__}
            x,y,bw,bh = cv2.boundingRect(quad)
            inlier_factor = min(1, n_inliers/16.0)
            spread_factor = min(1, spread/0.10)
            raw_score = float(min(1, ratio)*inlier_factor*spread_factor)
            score = self._confidence_from_evidence(
                item["category"], raw_score, ratio, inlier_factor,
                spread_factor, correlation, bound)
            metric_width = float(item.get('artwork_width_m', item['width_m']))
            metric_height = float(item.get('artwork_height_m', item['height_m']))
            indices=np.flatnonzero(mask.ravel()).tolist()
            indices.sort(key=lambda k:(source[k,1],source[k,0]))
            indices=[indices[k] for k in np.linspace(0,len(indices)-1,min(64,len(indices))).astype(int)]
            metric=np.column_stack(((source[indices,0]/(w-1)-.5)*metric_width,
                                    (source[indices,1]/(h-1)-.5)*metric_height,np.zeros(len(indices))))
            output.append({"label": item["label"], "category": item["category"],
                           "confidence": score, "raw_matching_score": round(raw_score, 4),
                           "bbox": [x,y,bw,bh],
                           "score_factors": {"ratio": round(float(ratio), 4),
                                             "inliers": round(inlier_factor, 4),
                                             "spread": round(spread_factor, 4),
                                             "photometric": round(float(
                                                 max(0.,min(1.,(correlation-bound)/
                                                           max(1e-6,1.-bound)))),4)},
                           "quad": quad.round(2).tolist(), "inliers": n_inliers,
                           "width_m":metric_width,"height_m":metric_height,
                           "board_width_m":item["width_m"],"board_height_m":item["height_m"],
                           "pose_correspondences":{"object":metric.tolist(),"image":target[indices].tolist()},
                           "photometric_correlation":round(correlation,4),
                           "method": "official_reference_orb_ransac",
                           "character_ocr":bool(ocr and ocr['complete']),
                           "ocr_status":('error' if ocr and ocr.get('error') else
                                         'recognized' if ocr and ocr['complete'] else 'uncertain'),
                           "ocr_result":ocr,
                           "ocr_matches_reference":bool(ocr and ocr['complete'] and
                                                         ocr['text']==item['label'])})
        output.sort(key=lambda d: d["confidence"], reverse=True)
        kept = []
        for candidate in output:
            if all(iou(candidate["bbox"], prior["bbox"]) < 0.35 for prior in kept):
                kept.append(candidate)
        return sorted(kept, key=lambda d: (d["bbox"][0], d["bbox"][1]))

    @staticmethod
    def annotate(frame, detections):
        from PIL import Image, ImageDraw, ImageFont
        rgb = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(rgb)
        font = None
        for name in ["/usr/share/fonts/truetype/wqy/wqy-microhei.ttc", "C:/Windows/Fonts/msyh.ttc",
                     "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"]:
            if os.path.exists(name):
                font = ImageFont.truetype(name, 17)
                break
        if font is None:
            font = ImageFont.load_default()
        for d in detections:
            x,y,w,h = d["bbox"]
            for offset in (0,1):
                draw.rectangle((x+offset,y+offset,x+w-offset,y+h-offset), outline=(15,220,145))
            name = d["label"]
            if not name.startswith(d["category"]):
                name = "%s %s" % (d["category"], name)
            label = "%s %.2f" % (name, d["confidence"])
            if d.get('category')=='plate':
                state=d.get('ocr_display_status','pending')
                if state=='verified':
                    label += ' OCR:'+d['ocr_display_text']
                elif state=='pending':
                    slots=d.get('ocr_pending_slots',list(range(1,8)))
                    label += ' OCR:pending[%s]'%(','.join(str(i) for i in slots))
                else:
                    label += ' OCR:'+state
            try:
                draw.text((max(0,x), max(0,y-21)), label, font=font, fill=(255,210,30))
            except UnicodeEncodeError:
                draw.text((max(0,x), max(0,y-21)), d["category"] + " %.2f" % d["confidence"], font=font, fill=(255,210,30))
        return cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
