#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""patrol_dashboard.py -- 巡检过程的实时状态面板（纯订阅，可选工具）。

用途
----
把已经在发的话题汇总成一块**终端里实时刷新的面板**，方便：
  * 演示视频里让评委一眼看到识别结果（对应"视觉识别与检测"这一项）；
  * 现场盯盘时快速判断卡在哪一步。

★ 它只订阅、不发布，也不被 patrol.launch 拉起。
  所以：
    - 它不可能影响导航/避障/识别，任何时刻都能安全打开或关掉；
    - 跑成功率批次时**不要**开它（少一份 CPU 占用），
      跑出来的证据与开不开面板无关。

用法
----
    # 另开一个终端
    source /opt/ros/melodic/setup.bash
    source ~/smart_community_run6/catkin_ws/devel/setup.bash
    rosrun smart_community_semifinal patrol_dashboard.py

    # 同时把面板内容落盘，便于事后写报告
    rosrun smart_community_semifinal patrol_dashboard.py --log /tmp/dash.log

    # 不刷新屏幕，只打印（重定向到文件时用）
    rosrun smart_community_semifinal patrol_dashboard.py --plain

    # 不起仿真，用假数据自检面板渲染（部署后先跑这个确认字体/对齐正常）
    rosrun smart_community_semifinal patrol_dashboard.py --self-test

订阅的话题（都已在发，无需改任何节点）
    /semifinal/task_status      phase / index / target
    /semifinal/street_summary   A/B 街区人数
    /semifinal/events           每次检测（label + 置信度）
    /semifinal/plate_summary    车牌汇总
    /semifinal/visual_signal    红绿灯状态
    /semifinal/guard_status     避障与门状态
"""
from __future__ import print_function

import argparse
import io
import json
import sys
import time

import rospy
from std_msgs.msg import String

# ---------------------------------------------------------------- 显示宽度
WIDTH = 78
# 最近识别的滚动行数
RECENT_ROWS = 8

# 街区块的显示名
STREET_NAME = {'A': u'A 街区', 'B': u'B 街区'}
# 分类显示名
CATEGORY_NAME = {'resident': u'社区人员', 'visitor': u'非社区人员',
                 'plate': u'车牌', 'person': u'人偶'}
# 灯状态显示
STATE_NAME = {'red': u'红', 'green': u'绿', 'yellow': u'黄', 'unknown': u'?'}


def _decode(text):
    """std_msgs/String 在 py2 下可能是 str 也可能是 unicode。"""
    if isinstance(text, bytes):
        try:
            return text.decode('utf-8')
        except UnicodeDecodeError:
            return text.decode('utf-8', 'replace')
    return text


def _text(value):
    """把任意值安全地转成文本。

    ★ py2 陷阱：JSON 解出来的字符串是 unicode，而 py2 的 str(u'苏') 会尝试
    按 ASCII 编码，直接抛 UnicodeEncodeError。所以这里绝不能用 str()，
    要么原样返回 unicode，要么把 UTF-8 字节解码，数字/布尔才用 %s 格式化。
    """
    if value is None:
        return u''
    if isinstance(value, type(u'')):
        return value
    if isinstance(value, bytes):
        return _decode(value)
    return u'%s' % value


def _dwidth(text):
    """终端里的显示宽度：CJK 与全角字符占 2 列。"""
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
    """按显示宽度补空格，保证中英混排也对齐。"""
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
        self.recent = []            # [(wall_clock, category, label, confidence)]
        self.started = time.time()
        self.first_task = None
        # 用来判断"一条消息都没收到"：面板起太早（roscore 还没起）或
        # ROS_MASTER_URI 不一致时，订阅注册不上，界面会一直空着，
        # 看上去像坏了。记下最后收信时刻，超时就明确提示。
        self.last_rx = None
        self.log = io.open(log_path, 'a', encoding='utf-8') if log_path else None

        rospy.Subscriber('/semifinal/task_status', String, self.on_task)
        rospy.Subscriber('/semifinal/street_summary', String, self.on_streets)
        rospy.Subscriber('/semifinal/events', String, self.on_events)
        rospy.Subscriber('/semifinal/plate_summary', String, self.on_plates)
        rospy.Subscriber('/semifinal/visual_signal', String, self.on_signal)
        rospy.Subscriber('/semifinal/guard_status', String, self.on_guard)

    # ------------------------------------------------------------ 回调
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

    # ------------------------------------------------------------ 渲染
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
        # 一条消息都没收到（或已静默）时明确说明，别让人以为面板坏了
        silent = (self.last_rx is None and elapsed > 5.0)
        if self.last_rx is not None and time.time() - self.last_rx > 15.0:
            silent = True
        if silent:
            out.append(u' ⚠ 还没收到任何话题消息')
            out.append(u'   可能原因：面板起在 roscore 之前，或 ROS_MASTER_URI 不一致')
            out.append(u'   检查：rostopic list | grep semifinal')
            out.append(u'   解法：等场景起来后再启动面板')
            out.append(self.rule())

        # 航点 / 相位
        out.append(u' 航点   %s  phase=%s  target=%s' % (
            _pad(_text(index), 5), _pad(_text(phase), 10),
            _text(target)))
        out.append(u' 计时   %s' % self._hms(elapsed))

        # 红绿灯
        if self.signals:
            parts = []
            for lid in sorted(self.signals):
                s = self.signals[lid]
                parts.append(u'%s · %s' % (
                    lid, STATE_NAME.get(s.get('state'), s.get('state'))))
            out.append(u' 红绿灯 %s' % u'      '.join(parts))
        else:
            out.append(u' 红绿灯 (等待信号感知)')

        # 街区人数
        if self.streets:
            for key in ('A', 'B'):
                value = self.streets.get(key) or {}
                out.append(u' %s 共 %s 人（社区 %s · 非社区 %s）' % (
                    _pad(STREET_NAME.get(key, key), 7),
                    value.get('total', u'?'), value.get('resident', u'?'),
                    value.get('visitor', u'?')))
        else:
            out.append(u' 街区   (等待账本)')

        # 车牌
        # /semifinal/plate_summary 的值是 official_perception_node 里的
        # plate_results[label]，即一份 OCR 结果字典
        # {text, complete, pending_slots, ...}，不是计数。两种形态都兼容，
        # 免得以后换了发布端就打出原始 dict。
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

        # 避障
        if self.guard:
            out.append(u' 避障   前向障碍=%s 左侧=%s 右侧=%s' % (
                _pad(_text(self.guard.get('forward_obstacle')), 6),
                _pad(_text(self.guard.get('left_free')), 6),
                _text(self.guard.get('right_free'))))

        # 最近识别
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

    # ------------------------------------------------------------ 主循环
    def run(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            try:
                text = self.render()
            except Exception as exc:                     # 面板绝不能拖垮自己
                text = u'[面板渲染异常 %s: %s]' % (type(exc).__name__, exc)
            if self.plain:
                _emit(_encode(text) + b'\n\n')
            else:
                # 光标回左上角重画，避免闪烁
                _emit(b'\033[H\033[J' + _encode(text) + b'\n')
            sys.stdout.flush()
            rate.sleep()


def _encode(text):
    """把面板文本转成 UTF-8 字节（写文件/日志用）。"""
    if sys.version_info[0] == 2:
        return text.encode('utf-8') if isinstance(text, unicode) else text  # noqa: F821
    return text.encode('utf-8') if isinstance(text, str) else text


def _emit(payload):
    """把面板文本安全写到 stdout，bytes 与 str 都接受。

    py2 的 sys.stdout.write 收字节；py3 的只收 str，直接写字节会 TypeError。
    面板两种解释器都要能跑，所以统一走这里；入参放宽到两种类型，
    这样调用方不必关心 _encode 到底有没有转成字节。
    """
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
    """不依赖仿真：喂一份假消息，渲染一次，验证面板本身能跑。"""
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
        # 清屏一次，之后靠 ANSI 定位重画
        _emit(b'\033[H\033[J')
        sys.stdout.flush()
    dashboard.run(args.hz)
    return 0


if __name__ == '__main__':
    sys.exit(main())
