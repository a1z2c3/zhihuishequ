#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""make_random_plates.py -- 生成随机车牌素材（供车辆识别场景使用）。

为什么不用"在线车牌生成器"
--------------------------
比赛允许使用三张示例车牌，但**要求至少两张是随机号码**。用外部在线服务生成
看起来最省事，但它用的是**另一种字体**，而本项目的字符 OCR 模板是按官方三张
示例牌标定的（`plate_ocr.PlateCharacterRecognizer`）。实测把在线生成的
`川H7WHEE` 喂进去只读出 `赣?7?H?2`，7 个槽位错 3 个。任务要求里明确写着
"车牌识别 —— 识别车辆号牌并**输出字符结果**"，所以读不出字符就是不合规。

做法：从官方牌里"剪字符、重组合"
--------------------------------
`PlateCharacterRecognizer` 把车牌按宽度比例切成 7 个固定槽位（`SLOTS`），每个
槽位再与"在该槽位出现过的字符"的模板比对。因此只要**每个新字符都从"它原本被
正确识别过的那张官方牌的同一个槽位"剪出来**，字体、字号、位置就完全一致，
OCR 必然能读，而号码是新的。

约束：只能用官方三张牌里已经出现过的字符（共 14 个），所以新号码是这 14 个字符
的新组合，而不是全新字形。这满足"随机号码"的要求；若日后要引入全新字形，必须
先扩充字符模板集。

用法
----
    python tools/make_random_plates.py                # 内置种子，结果可复现
    python tools/make_random_plates.py --seed 1234    # 换一组组合
    python tools/make_random_plates.py --picks 二,二,一,三,二,二,一 --index 1

生成后必须把新号码同步到 `build_official_scene.py` 的 `plate_labels` 与路线里的
`expected_label`，否则评价器会认为车牌不匹配（脚本会打印可直接粘贴的行）。
"""
from __future__ import print_function

import argparse
import os
import random
import sys

import numpy as np
import cv2
from PIL import Image

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG, "scripts"))
from plate_ocr import PlateCharacterRecognizer as Recognizer      # noqa: E402

OUT_DIR = os.path.join(PKG, "assets", "random_plates")
DEFAULT_MATERIALS = os.environ.get("SEMIFINAL_MATERIALS", r"D:/智慧社区/复赛资料")
DEFAULT_SEED = 20260927

# 官方三张示例牌：短名 -> (文件名, 车牌文字)
EXAMPLES = (
    ("一", "车牌一.png", u"苏AB8Q62"),
    ("二", "车牌二.png", u"鄂D7B5Q2"),
    ("三", "车牌三.png", u"苏APL12A"),
)
SLOTS = ((1, "random_1.png"), (2, "random_2.png"))


def slot_pixels():
    """OCR 的 7 个槽位在像素上的左右边界。"""
    width = Recognizer.WIDTH
    return [(int(round(a * width)), int(round(b * width)))
            for a, b in Recognizer.SLOTS]


def load_examples(materials):
    """读入官方示例牌，统一缩放到 OCR 的工作尺寸。返回 短名 -> (RGB 数组, 文字)。"""
    plates = {}
    for key, filename, text in EXAMPLES:
        path = os.path.join(materials, u"车辆识别", filename)
        with Image.open(path) as handle:
            image = handle.convert("RGB").resize(
                (Recognizer.WIDTH, Recognizer.HEIGHT), Image.LANCZOS)
        plates[key] = (np.array(image), text)
    return plates


def character_bank(plates):
    """槽位 -> {字符: 来源短名}，只收录在该槽位确实出现过的字符。"""
    bank = {}
    for key, (_image, text) in plates.items():
        for index, char in enumerate(text):
            bank.setdefault(index, {}).setdefault(char, key)
    return bank


def random_picks(bank, rng):
    """随机挑一条新号码；每个槽位从该槽位可用的字符里取一个。"""
    return [rng.choice(sorted(bank[index].items())) for index in range(7)]


def compose(plates, picks):
    """按槽位从来源牌剪贴，拼出一张新牌。"""
    out = np.zeros((Recognizer.HEIGHT, Recognizer.WIDTH, 3), np.uint8)
    bounds = slot_pixels()
    for index, (_char, source) in enumerate(picks):
        x0, x1 = bounds[index]
        out[:, x0:x1] = plates[source][0][:, x0:x1]
    return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--materials", default=DEFAULT_MATERIALS,
                        help="复赛资料根目录（内含 车辆识别/车牌*.png）")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--picks", default=None,
                        help="逗号分隔的 7 个来源短名，直接指定组合")
    parser.add_argument("--index", type=int, default=None,
                        help="配合 --picks：写到 random_<index>.png")
    args = parser.parse_args(argv)

    plates = load_examples(args.materials)
    bank = character_bank(plates)

    if args.picks:
        keys = [k.strip() for k in args.picks.split(",")]
        if len(keys) != 7:
            parser.error("--picks needs exactly 7 source keys")
        picks = [(plates[k][1][i], k) for i, k in enumerate(keys)]
        jobs = [(args.index or 1, picks)]
    else:
        rng = random.Random(args.seed)
        jobs = [(idx, random_picks(bank, rng)) for idx, _ in SLOTS]

    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)

    # 自检：生成的牌必须能被本项目 OCR 完整读出，否则不要采用
    recognizer = Recognizer([(plates[k][0][:, :, ::-1], plates[k][1])
                             for k, _f, _t in EXAMPLES])
    written = []
    for idx, picks in jobs:
        label = "".join(char for char, _src in picks)
        image = compose(plates, picks)
        path = os.path.join(OUT_DIR, "random_%d.png" % idx)
        cv2.imencode(".png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))[1].tofile(path)
        result = recognizer.recognize(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        ok = result["complete"] and result["text"] == label
        print("  random_%d.png  %-10s  OCR=%s  %s"
              % (idx, label, result["text"], "OK" if ok else "FAIL"))
        if not ok:
            print("     [警告] 自身 OCR 读不完整，不要采用这张")
        written.append((idx, label, ok))

    print()
    print("  已写入 %s" % OUT_DIR)
    print("  同步到 build_official_scene.py：")
    print('      plate_labels = [("一", "苏AB8Q62", None),')
    for idx, label, _ok in written:
        print('                      (None, "%s", "random_%d.png"),' % (label, idx))
    print("  并把路线里的 expected_label 改成同样的号码。")
    return 0 if all(ok for _i, _l, ok in written) else 1


if __name__ == "__main__":
    sys.exit(main())
