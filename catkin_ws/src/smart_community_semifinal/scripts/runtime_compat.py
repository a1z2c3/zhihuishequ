#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""兼容目标虚拟机和本地运行环境。"""
from __future__ import division, unicode_literals
import ctypes
import errno
import math
import os
import sys
import time

try:
    text_type = unicode
except NameError:
    text_type = str


def ros_text(value):
    """将日志文本转为兼容旧版运行环境的编码。"""
    if sys.version_info[0]==2 and isinstance(value,text_type):
        return value.encode('utf-8')
    return value


def isfinite(value):
    try:
        return not (math.isnan(value) or math.isinf(value))
    except (TypeError, ValueError, OverflowError):
        return False


def makedirs(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise


if hasattr(time, 'monotonic'):
    monotonic = time.monotonic
else:
    class Timespec(ctypes.Structure):
        _fields_ = [('seconds', ctypes.c_long), ('nanoseconds', ctypes.c_long)]
    _clock = ctypes.CDLL('librt.so.1', use_errno=True).clock_gettime
    _clock.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
    _clock.restype = ctypes.c_int

    def monotonic():
        ts = Timespec()
        if _clock(1, ctypes.byref(ts)) != 0:
            raise OSError(ctypes.get_errno(), 'CLOCK_MONOTONIC failed')
        return ts.seconds + ts.nanoseconds / 1e9
