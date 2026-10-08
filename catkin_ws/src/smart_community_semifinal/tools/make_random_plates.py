#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""使用给定字符样本生成车牌图案。"""
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

EXAMPLES = (
    ("一", "车牌一.png", u"苏AB8Q62"),
    ("二", "车牌二.png", u"鄂D7B5Q2"),
    ("三", "车牌三.png", u"苏APL12A"),
)
SLOTS = ((1, "random_1.png"), (2, "random_2.png"))


def slot_pixels():
    """返回七个字符槽位的像素边界。"""
    width = Recognizer.WIDTH
    return [(int(round(a * width)), int(round(b * width)))
            for a, b in Recognizer.SLOTS]


def load_examples(materials):
    """读取并统一示例车牌尺寸。"""
    plates = {}
    for key, filename, text in EXAMPLES:
        path = os.path.join(materials, u"车辆识别", filename)
        with Image.open(path) as handle:
            image = handle.convert("RGB").resize(
                (Recognizer.WIDTH, Recognizer.HEIGHT), Image.LANCZOS)
        plates[key] = (np.array(image), text)
    return plates


def character_bank(plates):
    """按槽位索引可用字符样本。"""
    bank = {}
    for key, (_image, text) in plates.items():
        for index, char in enumerate(text):
            bank.setdefault(index, {}).setdefault(char, key)
    return bank


def random_picks(bank, rng):
    """从各槽位的可用字符中随机组合新车牌。"""
    return [rng.choice(sorted(bank[index].items())) for index in range(7)]


def compose(plates, picks):
    """保留牌面背景，替换字符槽位。"""
    out = plates[picks[0][1]][0].copy()
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

    recognizer = Recognizer([(plates[k][0][:, :, ::-1], plates[k][1])
                             for k, _f, _t in EXAMPLES])
    prepared = []
    for idx, picks in jobs:
        label = "".join(char for char, _src in picks)
        image = compose(plates, picks)
        result = recognizer.recognize(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        ok = result["complete"] and result["text"] == label
        print("  random_%d.png  %-10s  OCR=%s  %s"
              % (idx, label, result["text"], "OK" if ok else "FAIL"))
        if not ok:
            print("     [警告] 自身 OCR 读不完整；未覆盖现有素材")
        prepared.append((idx, label, ok, image))

    if not all(ok for _i, _l, ok, _image in prepared):
        return 1
    written = []
    for idx, label, ok, image in prepared:
        path = os.path.join(OUT_DIR, "random_%d.png" % idx)
        encoded_ok, encoded = cv2.imencode(".png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        if not encoded_ok:
            raise IOError("PNG encoding failed: " + path)
        encoded.tofile(path)
        written.append((idx, label, ok))

    print()
    print("  已写入 %s" % OUT_DIR)
    print("  同步到 build_official_scene.py：")
    print('      PLATE_LABELS = (("一", "苏AB8Q62", None),')
    for idx, label, _ok in written:
        print('                      (None, "%s", "random_%d.png"),' % (label, idx))
    print('                     )')
    print("  并把路线里的 expected_label 改成同样的号码。")
    return 0 if all(ok for _i, _l, ok in written) else 1


if __name__ == "__main__":
    sys.exit(main())
