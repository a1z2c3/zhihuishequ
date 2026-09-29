#!/usr/bin/env bash
# vm_batch_laps.sh -- 冻结版本下重复整圈，收集成功率证据。
#
# 用法:
#   bash vm_batch_laps.sh <圈数> [证据目录]
#
# 例:
#   bash vm_batch_laps.sh 10 /mnt/hgfs/智慧社区/比赛交付物/vm_runs/batch3
#
# 设计要点（都是踩过的坑）:
#   1) 证据默认写到【共享目录】，不写 VM 本地盘 —— VM 只剩 1~2 GB，
#      30 圈的 run_result.json + perception/ 会把它撑爆，而盘满会让后续圈静默失败。
#   2) 每圈之间完整清场并等待，gazebo/roslaunch 没退干净时下一圈会抢端口。
#   3) 可断点续跑：已存在 run_result.json 的圈直接跳过。
#   4) 每圈记录墙钟耗时，用来盯 RTF 漂移（评价器上限 1800 s，余量要看得见）。
#   5) 全部跑完自动调用 vm_summarize_runs.py 给出联合成功率与 95% 置信下限。
set -uo pipefail

COUNT=${1:-10}
SHARE=${2:-/mnt/hgfs/智慧社区/比赛交付物/vm_runs/batch_$(date +%m%d_%H%M)}
PKG_SHARE=/mnt/hgfs/智慧社区/比赛交付物
ROOT=${ROOT:-$HOME/smart_community_run6}
MIN_FREE_MB=800          # 共享目录剩余空间下限；低于它就先停下来提醒清理

say(){ echo "[$(date +%H:%M:%S)] $*"; }

cleanup(){
  pkill -f gzserver     2>/dev/null
  pkill -f gzclient     2>/dev/null
  pkill -f roslaunch    2>/dev/null
  pkill -f rosmaster    2>/dev/null
  pkill -f patrol_node  2>/dev/null
  pkill -f evaluate_run 2>/dev/null
  pkill -f rviz         2>/dev/null
  pkill -f rqt_image_view 2>/dev/null
  pkill -f ffmpeg       2>/dev/null
  sleep 6
  # 确认真的退干净了
  for _ in $(seq 1 10); do
    pgrep -f 'gzserver|roslaunch|rosmaster' >/dev/null 2>&1 || return 0
    sleep 3
  done
  say "  [警告] 仍有进程未退出，强制清理"
  pkill -9 -f gzserver   2>/dev/null
  pkill -9 -f roslaunch  2>/dev/null
  pkill -9 -f rosmaster  2>/dev/null
  sleep 3
}

[ -f "$ROOT/vm_run_lap.sh" ] || { echo "找不到工程根: $ROOT" >&2; exit 2; }
mkdir -p "$SHARE"

say "批量跑圈开始"
say "  工程根   : $ROOT"
say "  证据目录 : $SHARE"
say "  计划圈数 : $COUNT"
df -h "$SHARE" | tail -1 | awk '{printf "  共享目录剩余: %s\n", $4}'
df -h /       | tail -1 | awk '{printf "  VM 根盘剩余  : %s\n", $4}'
echo

pass=0; fail=0; skip=0; times=""
for i in $(seq -w 1 "$COUNT"); do
  D="$SHARE/lap_$i"
  echo "──────────────────────────────────────────────────────────"
  if [ -f "$D/run_result.json" ]; then
    say "lap_$i 已有结果，跳过"
    skip=$((skip+1))
    continue
  fi

  # 共享目录空间检查：宁可停下让人清理，也不要跑出静默失败的圈
  FREE_MB=$(df -Pm "$SHARE" | awk 'NR==2{print $4}')
  if [ "$FREE_MB" -lt "$MIN_FREE_MB" ]; then
    say "  [停止] 共享目录只剩 ${FREE_MB}MB（下限 ${MIN_FREE_MB}MB），请先清理"
    break
  fi

  cleanup
  say "lap_$i 开始（共享目录剩余 ${FREE_MB}MB）"
  T0=$(date +%s)
  if bash "$ROOT/vm_run_lap.sh" "$D" > "$SHARE/lap_$i.console.log" 2>&1; then
    T1=$(date +%s); EL=$((T1-T0)); times="$times $EL"
    # run_result.json 没有 result 字段：判定与 vm_summarize_runs.py 完全一致
    #   done      = task.phase == 'done'
    #   clean     = body_violations 为空
    #   all_green = 每条停止线穿越都 pass
    #   joint     = 三者同时成立  <= 计入成功率的那个数
    LINE=$(python3 - "$D" <<'JUDGE' 2>/dev/null
import json, io, sys
try:
    d = json.load(io.open(sys.argv[1] + "/run_result.json", encoding="utf-8"))
    task = d.get("task") or {}
    crossings = d.get("stop_crossings") or []
    done = task.get("phase") == "done"
    clean = len(d.get("body_violations") or []) == 0
    green = bool(crossings) and all(c.get("pass") for c in crossings)
    joint = done and clean and green
    print("%s|%s|%.1f|%.1f|%s|%s" % (
        "JOINT-OK" if joint else "JOINT-FAIL",
        task.get("phase"), d.get("simulated_elapsed") or 0.0,
        d.get("wall_seconds") or 0.0, len(crossings), task.get("error")))
except Exception as exc:
    print("NO_RESULT|-|0|0|-|%s" % type(exc).__name__)
JUDGE
)
    R=${LINE%%|*}; REST=${LINE#*|}
    PHASE=$(echo "$REST" | cut -d'|' -f1); SIM=$(echo "$REST" | cut -d'|' -f2)
    EWALL=$(echo "$REST" | cut -d'|' -f3); NX=$(echo "$REST" | cut -d'|' -f4)
    ERR=$(echo "$REST" | cut -d'|' -f6)
    say "lap_$i 结束: $R   phase=${PHASE}   仿真 ${SIM}s   评价器墙钟 ${EWALL}s   越线 ${NX}   错误 ${ERR:-无}"
    if [ "$R" = "JOINT-OK" ]; then pass=$((pass+1)); else fail=$((fail+1)); fi
  else
    T1=$(date +%s); EL=$((T1-T0)); times="$times $EL"
    say "lap_$i 脚本非零退出（墙钟 ${EL}s），看 $SHARE/lap_$i.console.log"
    fail=$((fail+1))
  fi
done

cleanup
echo "──────────────────────────────────────────────────────────"
say "批量结束：PASS $pass   FAIL $fail   跳过 $skip"
[ -n "$times" ] && say "各圈墙钟(秒):$times"
echo
say "汇总："
if [ -f "$PKG_SHARE/vm_summarize_runs.py" ]; then
  python3 "$PKG_SHARE/vm_summarize_runs.py" "$SHARE" 2>&1 | tail -25
else
  echo "  找不到 vm_summarize_runs.py，手工汇总: $SHARE"
fi
echo
say "证据目录: $SHARE"
say "Windows 侧: D://智慧社区//比赛交付物//vm_runs//$(basename "$SHARE")"
