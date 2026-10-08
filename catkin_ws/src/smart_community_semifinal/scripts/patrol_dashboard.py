#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""只读的巡检终端状态面板。"""
from __future__ import print_function

import argparse
import io
import json
import sys
import time

import rospy
from std_msgs.msg import String

WIDTH = 78
RECENT_ROWS = 8

STREET_NAME = {'A': u'A 街区', 'B': u'B 街区'}
CATEGORY_NAME = {'resident': u'社区人员', 'visitor': u'非社区人员',
                 'plate': u'车牌', 'person': u'人偶'}
STATE_NAME = {'red': u'红', 'green': u'绿', 'yellow': u'黄', 'unknown': u'?'}


def _decode(text):
    """解码消息文本。"""
    if isinstance(text, bytes):
        try:
            return text.decode('utf-8')
        except UnicodeDecodeError:
            return text.decode('utf-8', 'replace')
    return text


def _text(value):
    """安全转换为统一文本类型。"""
    if value is None:
        return u''
    if isinstance(value, type(u'')):
        return value
    if isinstance(value, bytes):
        return _decode(value)
    return u'%s' % value


def _dwidth(text):
    """计算文本在终端中占用的列数。"""
    total = 0
    for ch in text:
        code = ord(ch)
        if (0x1100 <= code <= 0x115F or 0x2E80 <= code <= 0xA4CF or
                0xAC00 <= code <= 0xD7A3 or 0xF900 <= code <= 0xFAFF or
                0xFE30 <= code <= 0xFE6F or 0xFF00 <= code <= 0xFF60 or
                0xFFE0 <= code <= 0xFFE6):
            total += 2
        else:
            total += 1
    return total


def _pad(text, width, align='left'):
    """按显示宽度补齐空格。"""
    text = text if isinstance(text, type(u'')) else _decode(text)
    gap = width - _dwidth(text)
    if gap <= 0:
        return text
    return (text + u' ' * gap) if align == 'left' else (u' ' * gap + text)


def _load(msg):
    try:
        return json.loads(_decode(msg.data))
    except (ValueError, AttributeError, TypeError):
        return None


class Dashboard(object):
    def __init__(self, plain=False, log_path=None):
        self.plain = plain
        self.task = {}
        self.streets = {}
        self.plates = {}
        self.signals = {}
        self.guard = {}
        self.recent = []
        self.started = time.time()
        self.first_task = None
        self.last_rx = None
        self.log = io.open(log_path, 'a', encoding='utf-8') if log_path else None

        rospy.Subscriber('/semifinal/task_status', String, self.on_task)
        rospy.Subscriber('/semifinal/street_summary', String, self.on_streets)
        rospy.Subscriber('/semifinal/events', String, self.on_events)
        rospy.Subscriber('/semifinal/plate_summary', String, self.on_plates)
        rospy.Subscriber('/semifinal/visual_signal', String, self.on_signal)
        rospy.Subscriber('/semifinal/guard_status', String, self.on_guard)

    def on_task(self, msg):
        self.last_rx = time.time()
        data = _load(msg)
        if data:
            self.task = data
            if self.first_task is None:
                self.first_task = time.time()

    def on_streets(self, msg):
        self.last_rx = time.time()
        data = _load(msg)
        if isinstance(data, dict):
            self.streets = data

    def on_plates(self, msg):
        self.last_rx = time.time()
        data = _load(msg)
        if isinstance(data, dict):
            self.plates = data

    def on_signal(self, msg):
        self.last_rx = time.time()
        data = _load(msg)
        if data and data.get('light_id'):
            self.signals[data['light_id']] = data

    def on_guard(self, msg):
        self.last_rx = time.time()
        data = _load(msg)
        if isinstance(data, dict):
            self.guard = data

    def on_events(self, msg):
        self.last_rx = time.time()
        data = _load(msg)
        if not isinstance(data, dict):
            return
        stamp = data.get('stamp')
        for det in data.get('detections') or []:
            label = det.get('label')
            if not label:
                continue
            entry = (time.time(), det.get('category'), label,
                     det.get('confidence'))
            self.recent.append(entry)
            if self.log is not None:
                self.log.write(u'%.3f\t%s\t%s\t%.4f\n' % (
                    stamp if isinstance(stamp, (int, float)) else 0.0,
                    _decode(entry[1] or ''), _decode(label),
                    float(entry[3] or 0.0)))
                self.log.flush()
        if len(self.recent) > RECENT_ROWS * 4:
            del self.recent[:-RECENT_ROWS * 4]

    def rule(self, char=u'─'):
        return char * WIDTH

    def render(self):
        out = []
        task = self.task or {}
        phase = task.get('phase', u'?')
        index = task.get('index', u'?')
        target = task.get('target', u'?')
        error = task.get('error')
        elapsed = time.time() - (self.first_task or self.started)

        out.append(self.rule(u'═'))
        out.append(u' 智慧社区巡检 · 实时状态%s   [ %s ]' % (
            u'' if not error else u'   ★ ' + _text(error),
            time.strftime('%H:%M:%S')))
        out.append(self.rule(u'═'))
        silent = (self.last_rx is None and elapsed > 5.0)
        if self.last_rx is not None and time.time() - self.last_rx > 15.0:
            silent = True
        if silent:
            out.append(u' ⚠ 还没收到任何话题消息')
            out.append(u'   可能原因：面板起在 roscore 之前，或 ROS_MASTER_URI 不一致')
            out.append(u'   检查：rostopic list | grep semifinal')
            out.append(u'   解法：等场景起来后再启动面板')
            out.append(self.rule())

        out.append(u' 航点   %s  phase=%s  target=%s' % (
            _pad(_text(index), 5), _pad(_text(phase), 10),
            _text(target)))
        out.append(u' 计时   %s' % self._hms(elapsed))

        if self.signals:
            parts = []
            for lid in sorted(self.signals):
                s = self.signals[lid]
                parts.append(u'%s · %s' % (
                    lid, STATE_NAME.get(s.get('state'), s.get('state'))))
            out.append(u' 红绿灯 %s' % u'      '.join(parts))
        else:
            out.append(u' 红绿灯 (等待信号感知)')

        if self.streets:
            for key in ('A', 'B'):
                value = self.streets.get(key) or {}
                out.append(u' %s 共 %s 人（社区 %s · 非社区 %s）' % (
                    _pad(STREET_NAME.get(key, key), 7),
                    value.get('total', u'?'), value.get('resident', u'?'),
                    value.get('visitor', u'?')))
        else:
            out.append(u' 街区   (等待账本)')

        if self.plates:
            parts = []
            for key, value in sorted(self.plates.items()):
                label = _text(key)
                if isinstance(value, dict):
                    pending = value.get('pending_slots') or []
                    if value.get('complete'):
                        parts.append(u'%s ✓' % label)
                    elif pending:
                        parts.append(u'%s …待%d' % (label, len(pending)))
                    else:
                        parts.append(u'%s …' % label)
                else:
                    parts.append(u'%s ×%s' % (label, _text(value)))
            out.append(u' 车牌   %s' % u'  '.join(parts))

        if self.guard:
            out.append(u' 避障   前向障碍=%s 左侧=%s 右侧=%s' % (
                _pad(_text(self.guard.get('forward_obstacle')), 6),
                _pad(_text(self.guard.get('left_free')), 6),
                _text(self.guard.get('right_free'))))

        out.append(self.rule())
        out.append(u' 最近识别')
        recent = self.recent[-RECENT_ROWS:]
        if not recent:
            out.append(u'   (还没有检测事件)')
        for wall, category, label, conf in recent:
            out.append(u'   %s  %s %s %s' % (
                time.strftime('%H:%M:%S', time.localtime(wall)),
                _pad(CATEGORY_NAME.get(category, _text(category)), 11),
                _pad(_text(label), 14),
                (u'%.2f' % conf) if isinstance(conf, (int, float)) else u'—'))
        out.append(self.rule(u'═'))
        return u'\n'.join(out)

    @staticmethod
    def _hms(seconds):
        seconds = int(seconds)
        return u'%02d:%02d:%02d' % (seconds // 3600,
                                    (seconds % 3600) // 60, seconds % 60)

    def run(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            try:
                text = self.render()
            except Exception as exc:
                text = u'[面板渲染异常 %s: %s]' % (type(exc).__name__, exc)
            if self.plain:
                _emit(_encode(text) + b'\n\n')
            else:
                _emit(b'\033[H\033[J' + _encode(text) + b'\n')
            sys.stdout.flush()
            rate.sleep()


def _encode(text):
    """将面板文本编码为字节。"""
    if sys.version_info[0] == 2:
        return text.encode('utf-8') if isinstance(text, unicode) else text  # noqa: F821
    return text.encode('utf-8') if isinstance(text, str) else text


def _emit(payload):
    """兼容两种运行版本的标准输出。"""
    if isinstance(payload, bytes):
        if sys.version_info[0] == 2:
            sys.stdout.write(payload)
        else:
            sys.stdout.write(payload.decode('utf-8', 'replace'))
        return
    if sys.version_info[0] == 2:
        sys.stdout.write(payload.encode('utf-8'))
    else:
        sys.stdout.write(payload)


def self_test():
    """使用示例消息检查面板，无需启动仿真。"""
    dashboard = Dashboard(plain=True)
    samples = [
        (dashboard.on_task, {'phase': 'observe', 'index': 17,
                             'target': 'parking_2', 'stamp': 199.4, 'error': None}),
        (dashboard.on_streets, {'A': {'total': 8, 'resident': 7, 'visitor': 1,
                                      'street': 'A', 'instance_ids': []},
                                'B': {'total': 8, 'resident': 7, 'visitor': 1,
                                      'street': 'B', 'instance_ids': []}}),
        (dashboard.on_signal, {'light_id': 'light_1', 'state': 'green'}),
        (dashboard.on_signal, {'light_id': 'light_2', 'state': 'red'}),
        (dashboard.on_guard, {'forward_obstacle': False, 'left_free': True,
                              'right_free': True}),
        (dashboard.on_plates, {u'苏AB8Q62': 5, u'苏DB812A': 5, u'鄂DP8522': 5}),
        (dashboard.on_events, {'stamp': 199.4, 'detections': [
            {'label': 'resident_14', 'category': 'resident', 'confidence': 0.87},
            {'label': 'visitor_F2', 'category': 'visitor', 'confidence': 0.80},
            {'label': u'苏DB812A', 'category': 'plate', 'confidence': 0.92}]}),
    ]
    for callback, payload in samples:
        callback(type('M', (), {'data': json.dumps(payload, ensure_ascii=False)})())
    _emit(b'[self-test] ' + _encode(u'下面应是完整面板；若中文乱码或错位，'
                                    u'说明终端字体/编码需要调整\n'))
    _emit(_encode(dashboard.render()) + b'\n')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--plain', action='store_true',
                        help='不刷新屏幕，逐次打印（重定向到文件时用）')
    parser.add_argument('--log', default=None,
                        help='把每次检测追加写到该文件（UTF-8, TSV）')
    parser.add_argument('--hz', type=float, default=2.0,
                        help='面板刷新频率，默认 2 Hz')
    parser.add_argument('--self-test', action='store_true',
                        help='喂一份假数据渲染一次后退出，用来确认终端字体与对齐')
    args = parser.parse_args(argv)

    if args.self_test:
        return self_test()

    rospy.init_node('patrol_dashboard', anonymous=True)
    dashboard = Dashboard(plain=args.plain, log_path=args.log)
    if not args.plain:
        _emit(b'\033[H\033[J')
        sys.stdout.flush()
    dashboard.run(args.hz)
    return 0


if __name__ == '__main__':
    sys.exit(main())
