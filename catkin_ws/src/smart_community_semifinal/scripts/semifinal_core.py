#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-independent rule checks and evidence contracts, Python 2.7/3 compatible."""
from __future__ import division, unicode_literals
import io
import json
import math
import os
import time
from runtime_compat import isfinite, makedirs


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def footprint_corners(x, y, yaw, length=0.334, width=0.303, margin=0.02):
    c, s = math.cos(yaw), math.sin(yaw)
    a, b = length / 2 + margin, width / 2 + margin
    return [(x + c * u - s * v, y + s * u + c * v)
            for u, v in [(a, b), (a, -b), (-a, -b), (-a, b)]]


def front_clearance(pose, line_point, direction, margin=0.02):
    """Positive while the entire inflated footprint is behind the stop line."""
    dx, dy = direction
    norm = math.hypot(dx, dy)
    if norm == 0:
        raise ValueError("stop-line direction cannot be zero")
    dx, dy = dx / norm, dy / norm
    return min((line_point[0] - x) * dx + (line_point[1] - y) * dy
               for x, y in footprint_corners(pose[0],pose[1],pose[2],margin=margin))


def braking_distance(speed, latency=0.25, deceleration=0.35, buffer=0.03):
    if not all(isfinite(v) for v in (speed, latency, deceleration, buffer)):
        raise ValueError("non-finite braking inputs")
    if deceleration <= 0 or latency < 0 or buffer < 0:
        raise ValueError("invalid braking parameters")
    v = abs(speed)
    return v * latency + v * v / (2 * deceleration) + buffer


def scan_clearance(scan, speed):
    """Classify a scan against the candidate inflated-body corridors.

    A side is free only if the whole robot footprint after a bounded 0.09 m
    sidestep is clear.  This distinguishes an empty side sector from a
    centered obstacle that overlaps both legal corridors.  The scan object is
    intentionally duck-typed so the function works in offline tests too.
    """
    body_half_width = .303 / 2 + .02
    avoid_shift = .09
    side_safety = .012
    forward = False
    obstacle_ahead = False
    left_blocked = False
    right_blocked = False
    # Remaining clearance to the lane edge after the bounded sidestep and
    # the inflated footprint.  This is also the truthful default when no
    # return is present in a sector.
    lane_clearance = max(0., .6 / 2 - avoid_shift - body_half_width - side_safety)
    left_min = right_min = lane_clearance
    stop_horizon = .187 + braking_distance(max(abs(speed), .08))
    half = body_half_width + side_safety
    for i, distance in enumerate(scan.ranges):
        if not isfinite(distance) or not scan.range_min < distance < scan.range_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if x <= 0 or x >= .65:
            continue
        if x < stop_horizon and abs(y) <= body_half_width:
            forward = True
        # Keep the obstacle in the avoidance lane until its full body has
        # passed the robot.  A narrow forward-only test would go false as
        # soon as the robot sidesteps beside the box, causing an unsafe
        # immediate recenter into the box.
        if abs(y) <= body_half_width + avoid_shift + .05:
            obstacle_ahead = True
        left_distance = abs(y - avoid_shift) - half
        right_distance = abs(y + avoid_shift) - half
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
        "left_free": not left_blocked,
        "right_free": not right_blocked,
        "left_clearance": left_min,
        "right_clearance": right_min,
    }


class GreenGate(object):
    """A clock prediction can never replace fresh visual evidence.

    Timestamp must be the image acquisition time in the same clock domain as now.
    Repeated frames, clock resets, missing observations, red and yellow close it.
    This grants entry only; already crossing vehicles need a separate exit policy.
    """
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
            # Replay must not accumulate confirmations or keep a stale green alive.
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
    """Count physical instances by map position, independently from image class.

    Two copies of one artwork remain two people; seeing one person twice remains
    one. Metric position must come from timestamped depth/TF or multi-view geometry.
    Artwork IDs alone are never a valid whole-street counter.
    """
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
    """One detection object drives both terminal JSON and annotated image.

    The caller supplies an image writer to keep this component independent of cv2.
    Write image first, then append the record; a failed image write cannot produce
    a terminal-only success. Confidence is explicitly a score until calibrated.
    """
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
               "confidence_kind": "uncalibrated_matching_score", "detections": detections}
        line = json.dumps(row, ensure_ascii=False, allow_nan=False)
        with io.open(self.path, "a", encoding="utf-8") as stream:
            stream.write(line + "\n")
        self.sequence = sequence
        return line
