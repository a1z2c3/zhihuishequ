#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""汇总 vm_runs 下的所有整圈记录，输出成功率表与 JSON。

可在 Windows 或虚拟机里跑（只读 tar.gz，不需要 ROS）。

用法:
    python vm_summarize_runs.py [归档目录] [-o 汇总.json]
默认归档目录 = 本脚本同级的 vm_runs/

判据（三个分开报，再报联合）:
    完成     task.phase == 'done'
    无违规   body_violations 为空
    合规穿越 每次停止线穿越 pass 均为 true
    联合成功 三者同时成立   <- 表里的"成功"列用这一条
"""
import argparse
import io
import json
import math
import os
import sys
import tarfile


def comb(n, k):
    if k < 0 or k > n:
        return 0
    k = min(k, n - k)
    r = 1
    for i in range(k):
        r = r * (n - i) // (i + 1)
    return r


def cp_lower(k, n, alpha=0.05):
    """Clopper-Pearson 精确单侧下限。k/n 全成功时不是 1.0。"""
    if n == 0:
        return 0.0
    if k == 0:
        return 0.0
    if k == n:
        return (alpha / 2.0) ** (1.0 / n)
    target = alpha / 2.0
    lo, hi = 0.0, 1.0
    for _ in range(200):
        mid = (lo + hi) / 2.0
        tail = sum(comb(n, i) * mid ** i * (1 - mid) ** (n - i) for i in range(k, n + 1))
        # P(X >= k | p) 关于 p 单调【递增】：p 越大越容易成功 k 次以上。
        # 所以 tail < target 说明 mid 偏小（要往大走），tail > target 说明 mid 偏大。
        # 早先这里写反了，导致只要 k < n 就收敛到错误一侧：9/10 报 0%、8/10 报 100%。
        if tail < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def load_from_tar(path):
    # 旧版 vm_run_lap.sh 归档成 <RUN_ID>/evaluation/run_result.json；
    # 新版直接写在证据目录根下。两种都要认，否则新版证据会被静默跳过。
    fallback = None
    with tarfile.open(path, "r:gz") as tf:
        for member in tf.getmembers():
            if not member.name.endswith("run_result.json"):
                continue
            if member.name.endswith("/evaluation/run_result.json"):
                stream = tf.extractfile(member)
                if stream is None:
                    return None
                return json.loads(stream.read().decode("utf-8"))
            if fallback is None:
                fallback = member
        if fallback is not None:
            stream = tf.extractfile(fallback)
            if stream is None:
                return None
            return json.loads(stream.read().decode("utf-8"))
    return None


def load_from_dir(path):
    for candidate in (os.path.join(path, "evaluation", "run_result.json"),
                      os.path.join(path, "run_result.json")):
        if os.path.isfile(candidate):
            with io.open(candidate, encoding="utf-8") as stream:
                return json.load(stream)
    return None


def collect(root):
    runs = {}
    if not os.path.isdir(root):
        return runs
    for name in sorted(os.listdir(root)):
        full = os.path.join(root, name)
        if os.path.isdir(full):
            data = load_from_dir(full)
            if data is not None:
                runs[name] = data
    for name in sorted(os.listdir(root)):
        full = os.path.join(root, name)
        if os.path.isfile(full) and name.endswith(".tar.gz"):
            run_id = name[: -len(".tar.gz")]
            if run_id in runs:
                continue
            try:
                data = load_from_tar(full)
            except (tarfile.TarError, ValueError, OSError):
                data = None
            if data is not None:
                runs[run_id] = data
    return runs


def summarize(run_id, data):
    task = data.get("task") or {}
    crossings = data.get("stop_crossings") or []
    summary = data.get("street_summary") or {}
    events = data.get("events") or []
    labels = {}
    for event in events:
        for det in event.get("detections") or []:
            key = det.get("label")
            labels[key] = labels.get(key, 0) + 1
    plates = sorted(k for k in labels if k and not k.startswith(("resident_", "visitor_")))
    people = sorted(k for k in labels if k and k.startswith(("resident_", "visitor_")))
    cameras = data.get("camera_samples") or []
    wall = data.get("wall_seconds") or 0.0
    sim = data.get("simulated_elapsed") or 0.0
    street = {}
    for key in ("A", "B"):
        value = summary.get(key) or {}
        street[key] = {"resident": value.get("resident"), "visitor": value.get("visitor"), "total": value.get("total")}
    return {
        "run_id": run_id,
        "phase": task.get("phase"),
        "index": task.get("index"),
        "error": task.get("error"),
        "done": task.get("phase") == "done",
        "sim_seconds": round(sim, 2),
        "wall_seconds": round(wall, 1),
        "poses": len(data.get("poses") or []),
        "body_violations": len(data.get("body_violations") or []),
        "stop_crossings": len(crossings),
        "crossings_all_green": bool(crossings) and all(c.get("pass") for c in crossings),
        "street": street,
        "people_labels": len(people),
        "plate_labels": len(plates),
        "plates": plates,
        "events": len(events),
        "camera_samples": len(cameras),
        "camera_hz_wall": round(len(cameras) / wall, 3) if wall > 0 else None,
        "camera_hz_sim": round(len(cameras) / sim, 3) if sim > 0 else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "vm_runs"))
    parser.add_argument("-o", "--out", default=None)
    args = parser.parse_args()

    runs = collect(args.root)
    if not runs:
        print(u"在 %s 下没找到任何 run_result.json（.tar.gz 或已解压目录都可以）" % args.root)
        return 1

    rows = [summarize(run_id, data) for run_id, data in sorted(runs.items())]

    print("=" * 118)
    print("整圈记录汇总   目录: %s   共 %d 次" % (args.root, len(rows)))
    print("=" * 118)
    header = "%-18s %-6s %4s %7s %8s %6s %5s %5s %6s %6s %-14s"
    print(header % ("RUN_ID", "phase", "idx", "sim_s", "wall_s", "poses", "违规", "穿越", "人偶", "车牌", "街区A/B"))
    print("-" * 118)
    for row in rows:
        street = row["street"]
        ab = "A%s/%s B%s/%s" % (
            street["A"].get("resident"), street["A"].get("visitor"),
            street["B"].get("resident"), street["B"].get("visitor"))
        print(header % (
            row["run_id"][:18], row["phase"], row["index"] if row["index"] is not None else "-",
            row["sim_seconds"], row["wall_seconds"], row["poses"], row["body_violations"],
            row["stop_crossings"], row["people_labels"], row["plate_labels"], ab))

    n = len(rows)
    done = sum(1 for r in rows if r["done"])
    clean = sum(1 for r in rows if r["body_violations"] == 0)
    green = sum(1 for r in rows if r["crossings_all_green"])
    joint = sum(1 for r in rows if r["done"] and r["body_violations"] == 0 and r["crossings_all_green"])

    print("-" * 118)
    print("%-26s %2d/%2d = %5.1f%%   95%% 置信下限 %5.1f%%" % ("任务完成 (phase=done)", done, n, 100.0 * done / n, 100.0 * cp_lower(done, n)))
    print("%-26s %2d/%2d = %5.1f%%   95%% 置信下限 %5.1f%%" % ("零车身违规", clean, n, 100.0 * clean / n, 100.0 * cp_lower(clean, n)))
    print("%-26s %2d/%2d = %5.1f%%   95%% 置信下限 %5.1f%%" % ("停止线穿越全绿", green, n, 100.0 * green / n, 100.0 * cp_lower(green, n)))
    print("%-26s %2d/%2d = %5.1f%%   95%% 置信下限 %5.1f%%" % ("联合成功", joint, n, 100.0 * joint / n, 100.0 * cp_lower(joint, n)))
    if n < 30:
        print()
        print("提示: 目前只有 %d 次。计划是 30 次冻结版本；报告里要逐次列出，不能只放最好那一次。" % n)
    print()
    print(u"写进报告时的措辞示例（以实际数字为准，不要写「可靠性 99.9%」）：")
    print("  %d 次冻结版本整圈，联合成功 %d 次，成功率 %.1f%%，95%% 置信下限 %.1f%%。" % (n, joint, 100.0 * joint / n, 100.0 * cp_lower(joint, n)))

    if args.out:
        payload = {"runs": rows, "counts": {"n": n, "done": done, "clean": clean, "all_green": green, "joint": joint},
                   "cp_lower_95": {"done": cp_lower(done, n), "clean": cp_lower(clean, n),
                                   "all_green": cp_lower(green, n), "joint": cp_lower(joint, n)}}
        with io.open(args.out, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, indent=2))
        print("已写出: %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
