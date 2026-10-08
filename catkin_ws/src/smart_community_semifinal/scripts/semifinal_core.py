#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""与中间件无关的规则检查和证据约定。"""
from __future__ import division, unicode_literals
import io
import json
import math
import os
import time
from runtime_compat import isfinite, makedirs


BODY_LENGTH = .334
BODY_WIDTH = .303
FOOTPRINT_MARGIN = .020
LANE_WIDTH = .600
AVOID_SHIFT = .090
SIDE_SAFETY = .012
REAR_EXTENT = BODY_LENGTH / 2.0 + FOOTPRINT_MARGIN


def temporal_confidence(history, value, window=5, frames_since=1, max_gap=2):
    """融合当前有效匹配与有限窗口内的近期证据。"""
    try:
        value=float(value);window=int(window);frames_since=int(frames_since)
    except (TypeError,ValueError):
        raise ValueError('invalid confidence history')
    if window<1 or frames_since<1 or not isfinite(value) or not 0<=value<=1:
        raise ValueError('invalid confidence history')
    retained=list(history) if frames_since<=int(max_gap) else []
    values=[float(item) for item in retained[-(window-1):]]+[value]
    if not all(isfinite(item) and 0<=item<=1 for item in values):
        raise ValueError('invalid confidence history')
    ranked=sorted(values);middle=len(ranked)//2
    median=(ranked[middle] if len(ranked)%2 else
            .5*(ranked[middle-1]+ranked[middle]))
    return median,.35*value+.65*median,values


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def footprint_corners(x, y, yaw, length=BODY_LENGTH, width=BODY_WIDTH,
                      margin=FOOTPRINT_MARGIN):
    c, s = math.cos(yaw), math.sin(yaw)
    a, b = length / 2 + margin, width / 2 + margin
    return [(x + c * u - s * v, y + s * u + c * v)
            for u, v in [(a, b), (a, -b), (-a, -b), (-a, b)]]


def integrate_twist_pose(pose, speed, lateral, omega, duration):
    """根据恒定的车体速度积分位姿。"""
    if not all(isfinite(v) for v in (speed,lateral,omega,duration)) or duration < 0:
        raise ValueError('invalid twist integration input')
    x,y,yaw=pose[:3]
    end=yaw+omega*duration
    if abs(omega)<1e-8:
        return (x+(speed*math.cos(yaw)-lateral*math.sin(yaw))*duration,
                y+(speed*math.sin(yaw)+lateral*math.cos(yaw))*duration,
                end)
    return (x+(speed*(math.sin(end)-math.sin(yaw))+
               lateral*(math.cos(end)-math.cos(yaw)))/omega,
            y+(speed*(math.cos(yaw)-math.cos(end))+
               lateral*(math.sin(end)-math.sin(yaw)))/omega,
            end)


def front_clearance(pose, line_point, direction, margin=0.02):
    """车身膨胀轮廓完全位于停止线后方时为正。"""
    dx, dy = direction
    norm = math.hypot(dx, dy)
    if norm == 0:
        raise ValueError("stop-line direction cannot be zero")
    dx, dy = dx / norm, dy / norm
    return min((line_point[0] - x) * dx + (line_point[1] - y) * dy
               for x, y in footprint_corners(pose[0],pose[1],pose[2],margin=margin))


def body_over_stop_line(pose, line_point, direction):
    """判断实际车身是否跨在停止线上。"""
    dx, dy = direction
    norm = math.hypot(dx, dy)
    if norm == 0:
        raise ValueError("stop-line direction cannot be zero")
    distances = [(line_point[0] - x) * dx / norm +
                 (line_point[1] - y) * dy / norm
                 for x, y in footprint_corners(pose[0],pose[1],pose[2],margin=0)]
    return min(distances) < -1e-6 and max(distances) >= -1e-6


def braking_distance(speed, latency=0.25, deceleration=0.35, buffer=0.03):
    if not all(isfinite(v) for v in (speed, latency, deceleration, buffer)):
        raise ValueError("non-finite braking inputs")
    if deceleration <= 0 or latency < 0 or buffer < 0:
        raise ValueError("invalid braking parameters")
    v = abs(speed)
    return v * latency + v * v / (2 * deceleration) + buffer


def scan_clearance(scan, speed):
    """根据膨胀车身的有限走廊检查激光通行空间。"""
    body_half_width = BODY_WIDTH / 2.0 + FOOTPRINT_MARGIN
    half = body_half_width + SIDE_SAFETY
    stop_horizon = .187 + braking_distance(max(abs(speed), .08))
    rear_limit = -REAR_EXTENT - SIDE_SAFETY
    forward = False
    obstacle_ahead = False
    left_obstacle_ahead = False
    right_obstacle_ahead = False
    obstacle_rear_x = None
    center_blocked = False
    recenter_blocked = False
    left_blocked = False
    right_blocked = False
    left_min = right_min = max(0., LANE_WIDTH / 2.0 - AVOID_SHIFT - half)
    for i, distance in enumerate(scan.ranges):
        if (not isfinite(distance) or
                not scan.range_min < distance < scan.range_max):
            continue
        angle = scan.angle_min + i * scan.angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if x <= rear_limit or x >= .65:
            continue
        if x < stop_horizon and abs(y) <= half:
            forward = True
        if abs(y) <= half + AVOID_SHIFT:
            recenter_blocked = True
        if abs(y) <= half:
            center_blocked = True
            if x > rear_limit:
                obstacle_ahead = True
        if abs(y - AVOID_SHIFT) <= half and x > rear_limit:
            left_obstacle_ahead = True
        if abs(y + AVOID_SHIFT) <= half and x > rear_limit:
            right_obstacle_ahead = True
        if abs(y) <= half + AVOID_SHIFT and x > rear_limit:
            obstacle_rear_x = x if obstacle_rear_x is None else min(obstacle_rear_x,x)
        left_distance = abs(y - AVOID_SHIFT) - half
        right_distance = abs(y + AVOID_SHIFT) - half
        if left_distance <= 0:
            left_blocked = True
            left_min = 0.
        else:
            left_min = min(left_min, left_distance)
        if right_distance <= 0:
            right_blocked = True
            right_min = 0.
        else:
            right_min = min(right_min, right_distance)
    return {
        "forward_obstacle": forward,
        "obstacle_ahead": obstacle_ahead,
        "left_obstacle_ahead": left_obstacle_ahead,
        "right_obstacle_ahead": right_obstacle_ahead,
        "obstacle_rear_x": obstacle_rear_x,
        "left_free": not left_blocked,
        "right_free": not right_blocked,
        "recenter_clear": not recenter_blocked,
        "left_clearance": left_min,
        "right_clearance": right_min,
    }


class GreenGate(object):
    """时钟预测仍须由新鲜的视觉证据确认。"""
    def __init__(self, max_age=0.35, min_frames=3, min_confidence=0.70, max_gap=0.30):
        self.max_age, self.min_frames = max_age, min_frames
        self.min_confidence, self.max_gap = min_confidence, max_gap
        self.reset()

    def reset(self):
        self.light_id = None
        self.stamp = None
        self.count = 0
        self.state = "unknown"

    def update(self, light_id, state, confidence, stamp, now):
        if self.light_id != light_id:
            self.reset()
            self.light_id = light_id
        if not all(isinstance(v, (int, float)) and isfinite(v)
                   for v in (stamp, now, confidence)):
            self.reset()
            return False
        age = now - stamp
        if age < 0 or age > self.max_age or confidence < self.min_confidence:
            self.count, self.state = 0, "unknown"
            return False
        if self.stamp is not None and stamp <= self.stamp:
            if stamp < self.stamp:
                self.count, self.state = 0, "unknown"
            return False
        if self.stamp is not None and stamp - self.stamp > self.max_gap:
            self.count = 0
        self.stamp, self.state = stamp, state
        self.count = self.count + 1 if state == "green" else 0
        return self.allow(now)

    def allow(self, now):
        return bool(self.stamp is not None and isfinite(now)
                    and 0 <= now - self.stamp <= self.max_age
                    and self.state == "green" and self.count >= self.min_frames)


class StreetLedger(object):
    """按地图位置统计实际实例，不以图像类别代替身份。"""
    def __init__(self, radius=0.055, min_views=3):
        self.radius, self.min_views = radius, min_views
        self.instances = []

    def observe(self, street, label, category, point, frame_id):
        if category not in ("resident", "visitor") or street not in ("A", "B"):
            raise ValueError("unknown category/street")
        if len(point) != 2 or not all(isfinite(v) for v in point):
            raise ValueError("a valid metric location is required")
        candidates = [(math.hypot(it["x"] - point[0], it["y"] - point[1]), it)
                      for it in self.instances if it["street"] == street and it["label"] == label]
        candidates.sort(key=lambda pair: pair[0])
        if candidates and candidates[0][0] <= self.radius:
            item = candidates[0][1]
        else:
            item = {"instance_id": len(self.instances) + 1, "street": street,
                    "label": label, "category": category, "x": point[0], "y": point[1],
                    "frames": set(), "n": 0}
            self.instances.append(item)
        if frame_id not in item["frames"]:
            n = item["n"]
            item["x"] = (item["x"] * n + point[0]) / (n + 1)
            item["y"] = (item["y"] * n + point[1]) / (n + 1)
            item["n"] += 1
            item["frames"].add(frame_id)
        return item["instance_id"]

    def summary(self, street):
        confirmed = [i for i in self.instances if i["street"] == street
                     and len(i["frames"]) >= self.min_views]
        return {"street": street, "total": len(confirmed),
                "resident": sum(i["category"] == "resident" for i in confirmed),
                "visitor": sum(i["category"] == "visitor" for i in confirmed),
                "instance_ids": [i["instance_id"] for i in confirmed]}


class EvidenceWriter(object):
    """终端记录和标注图像使用同一份检测结果。"""
    def __init__(self, directory, image_writer, run_id=None):
        self.directory = directory
        makedirs(directory)
        self.image_writer = image_writer
        self.run_id = run_id or time.strftime("%Y%m%d_%H%M%S")
        self.sequence = 0
        self.path = os.path.join(directory, self.run_id + "_events.jsonl")

    def record(self, frame_id, stamp, detections, annotated):
        for d in detections:
            for key in ("category", "label", "confidence", "bbox"):
                if key not in d:
                    raise ValueError("missing detection field " + key)
            if not isfinite(d["confidence"]) or not 0 <= d["confidence"] <= 1:
                raise ValueError("invalid confidence")
        sequence = self.sequence + 1
        name = "%s_%06d.png" % (self.run_id, sequence)
        if not self.image_writer(os.path.join(self.directory, name), annotated):
            raise IOError("annotated image was not saved")
        row = {"schema_version": 1, "run_id": self.run_id, "sequence": sequence,
               "frame_id": frame_id, "stamp": stamp, "image": name,
               "confidence_kind": "evidence_fused_reference_match_score_not_probability",
               "detections": detections}
        line = json.dumps(row, ensure_ascii=False, allow_nan=False)
        with io.open(self.path, "a", encoding="utf-8") as stream:
            stream.write(line + "\n")
        self.sequence = sequence
        return line
