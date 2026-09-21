#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Known official-artwork baseline: ORB correspondence + RANSAC homography.

No map IDs, filenames, target poses or simulator truth enter detection. This is
reference recognition, NOT general character OCR or person re-identification.
Unknown artworks must be rejected. Gazebo camera evaluation is still required.
"""
from __future__ import division, unicode_literals
import io
import json
import os
from runtime_compat import text_type
import cv2
import numpy as np


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
        self.min_inliers, self.min_ratio = min_inliers, min_ratio
        self.references = []
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
            scale = 480.0 / max(gray.shape)
            gray = cv2.resize(gray, None, fx=scale, fy=scale)
            kp, desc = self.orb.detectAndCompute(gray, None)
            self.references.append((item, gray, kp, desc))

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp, desc = self.orb.detectAndCompute(gray, None)
        if desc is None or len(kp) < self.min_inliers:
            return []
        # Build the scene's binary-descriptor index once per frame. Brute-force
        # matching every reference was measured at >1 s/frame on this host.
        matcher = cv2.FlannBasedMatcher(dict(algorithm=6, table_number=6,
                                            key_size=12, multi_probe_level=1), dict(checks=32))
        matcher.add([desc]); matcher.train()
        output = []
        for item, reference, ref_kp, ref_desc in self.references:
            if ref_desc is None:
                continue
            pairs = matcher.knnMatch(ref_desc, k=2)
            good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < 60
                    and p[0].distance < 0.72*p[1].distance]
            if len(good) < self.min_inliers:
                continue
            # One observed feature must not vote multiple times for a template.
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
            # Plates share borders and repeated characters. Geometric inliers
            # alone can confidently pick the wrong plate: verify rectified pixels.
            rectified = cv2.warpPerspective(gray, np.linalg.inv(matrix), (w,h))
            pad_y,pad_x = max(1,int(h*.08)),max(1,int(w*.08))
            a=reference[pad_y:-pad_y,pad_x:-pad_x].astype(np.float32).ravel()
            b=rectified[pad_y:-pad_y,pad_x:-pad_x].astype(np.float32).ravel()
            a-=a.mean();b-=b.mean()
            correlation=float(np.dot(a,b)/max(1e-6,np.linalg.norm(a)*np.linalg.norm(b)))
            if correlation < (0.76 if item["category"]=="plate" else 0.58):
                continue
            x,y,bw,bh = cv2.boundingRect(quad)
            score = float(min(1, ratio)*min(1, n_inliers/24.0)*min(1, spread/0.10))
            # Interior feature correspondences constrain metric pose better than
            # four extrapolated homography corners on a narrow standing card.
            indices=np.flatnonzero(mask.ravel()).tolist()
            indices.sort(key=lambda k:(source[k,1],source[k,0]))
            indices=[indices[k] for k in np.linspace(0,len(indices)-1,min(64,len(indices))).astype(int)]
            metric=np.column_stack(((source[indices,0]/(w-1)-.5)*item['width_m'],
                                    (source[indices,1]/(h-1)-.5)*item['height_m'],np.zeros(len(indices))))
            output.append({"label": item["label"], "category": item["category"],
                           "confidence": round(score, 4), "bbox": [x,y,bw,bh],
                           "quad": quad.round(2).tolist(), "inliers": n_inliers,
                           "width_m":item["width_m"],"height_m":item["height_m"],
                           "pose_correspondences":{"object":metric.tolist(),"image":target[indices].tolist()},
                           "photometric_correlation":round(correlation,4),
                           "method": "official_reference_orb_ransac"})
        output.sort(key=lambda d: d["confidence"], reverse=True)
        kept = []
        for candidate in output:
            if all(iou(candidate["bbox"], prior["bbox"]) < 0.35 for prior in kept):
                kept.append(candidate)
        return sorted(kept, key=lambda d: (d["bbox"][0], d["bbox"][1]))

    @staticmethod
    def annotate(frame, detections):
        # Pillow can render Chinese plate characters; OpenCV putText cannot.
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
            # Pillow 5.1 in Ubuntu 18.04 has no rectangle(width=...).
            for offset in (0,1):
                draw.rectangle((x+offset,y+offset,x+w-offset,y+h-offset), outline=(15,220,145))
            label = "%s %s %.2f" % (d["category"], d["label"], d["confidence"])
            # ASCII label fallback preserves reference ID in a font-poor VM.
            try:
                draw.text((max(0,x), max(0,y-21)), label, font=font, fill=(255,210,30))
            except UnicodeEncodeError:
                draw.text((max(0,x), max(0,y-21)), d["category"] + " %.2f" % d["confidence"], font=font, fill=(255,210,30))
        return cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR)
