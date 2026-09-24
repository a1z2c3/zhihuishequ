#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dependency-free contracts shared by the optional navigation client."""
from __future__ import division, unicode_literals
from runtime_compat import isfinite


def gate_entry_ready(guard, gate, now, max_age=.35):
    """Return true only for a fresh, valid permit for the requested gate."""
    if not isinstance(guard, dict):
        return False
    try:
        stamp=float(guard.get('stamp', -1.))
        now=float(now)
        max_age=float(max_age)
    except (TypeError,ValueError):
        return False
    return bool(isfinite(stamp) and isfinite(now) and isfinite(max_age) and
                0.<=now-stamp<=max_age and guard.get('guard_valid',False) and
                guard.get('stop_id')==gate and guard.get('entry_ready',False))
