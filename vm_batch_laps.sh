#!/usr/bin/env bash
# vm_batch_laps.sh -- 冻结版本下重复整圈，收集成功率证据。
#
# 用法:
#   bash vm_batch_laps.sh <圈数> [归档目录]
#
# 例:
#   bash vm_batch_laps.sh 10 /mnt/hgfs/智慧社区/比赛交付物/vm_runs/batch3
#
# ★★ 2026-09-29 修掉一个会让每一圈都瞬间失败的 bug：
#    原版把证据目录直接设成 "$SHARE/lap_$i"，而 $SHARE 默认在【共享目录】——
#    那是中文路径（智慧社区 / 比赛交付物）。传给 vm_run_lap.sh 后就是
#    evidence_dir 含非 ASCII，roslaunch 在 Python 2 下按 ASCII 解 <arg>：
#      RLException: Invalid <arg> tag: 'ascii' codec can't decode byte 0xe6 ...
#    ⇒ 每一圈都在起场景时立刻失败。
#    现在：证据先写 VM 本地【纯 ASCII】目录 ~/vm_runs_batch，跑完再 cp 回共享目录归档。
#    （共享目录只用来归档，cp 不受影响；只有 roslaunch 解 arg 会炸。）
#
# 设计要点（都是踩过的坑）:
#   1) 证据先落本地 ASCII 目录（roslaunch 要求），再归档到共享目录（空间 + 留档）。
#      每圈归档成功后删除本地副本，避免 VM 只剩 1~2 GB 被撑爆。
#   2) 每圈之间完整清场并等待，gazebo/roslaunch 没退干净时下一圈会抢端口。
#   3) 可断点续跑：归档目录里已有 run_result.json 的圈直接跳过。
#   4) 每圈记录墙钟耗时，用来盯 RTF 漂移（评价器上限 1800 s，余量要看得见）。
#   5) 每圈打印【最长的 observe 阶段】—— 这是离 25 s 超时最近的指标。
#   6) 全部跑完自动调用 vm_summarize_runs.py 给出联合成功率与 95% 置信下限。
set -uo pipefail

COUNT=${1:-10}
SHARE=${2:-/mnt/hgfs/智慧社区/比赛交付物/vm_runs/batch_$(date +%m%d_%H%M)}
PKG_SHARE=/mnt/hgfs/智慧社区/比赛交付物
ROOT=${ROOT:-$HOME/smart_community_run6}
LOCAL=${LOCAL:-$HOME/vm_runs_batch}      # ★ 必须是纯 ASCII
MIN_FREE_MB=800          # 共享目录剩余空间下限
MIN_LOCAL_MB=500         # VM 根盘剩余下限

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
mkdir -p "$SHARE" "$LOCAL"

say "批量跑圈开始"
say "  工程根     : $ROOT"
say "  本地证据   : $LOCAL   (纯 ASCII，roslaunch 要求)"
say "  归档目录   : $SHARE"
say "  计划圈数   : $COUNT"
df -h "$SHARE" | tail -1 | awk '{printf "  归档盘剩余 : %s\n", $4}'
df -h /       | tail -1 | awk '{printf "  VM 根盘剩余: %s\n", $4}'
echo

pass=0; fail=0; skip=0; times=""
for i in $(seq -w 1 "$COUNT"); do
  A="$SHARE/lap_$i"
  D="$LOCAL/lap_$i"
  echo "──────────────────────────────────────────────────────────"
  if [ -f "$A/run_result.json" ]; then
    say "lap_$i 已有归档结果，跳过"
    skip=$((skip+1))
    continue
  fi

  FREE_MB=$(df -Pm "$SHARE" | awk 'NR==2{print $4}')
  if [ "$FREE_MB" -lt "$MIN_FREE_MB" ]; then
    say "  [停止] 归档盘只剩 ${FREE_MB}MB（下限 ${MIN_FREE_MB}MB），请先清理"
    break
  fi
  LOCAL_MB=$(df -Pm / | awk 'NR==2{print $4}')
  if [ "$LOCAL_MB" -lt "$MIN_LOCAL_MB" ]; then
    say "  [停止] VM 根盘只剩 ${LOCAL_MB}MB（下限 ${MIN_LOCAL_MB}MB），请先清理 $LOCAL"
    break
  fi

  cleanup
  rm -rf "$D"
  say "lap_$i 开始（归档盘剩 ${FREE_MB}MB，VM 根盘剩 ${LOCAL_MB}MB）"
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
    # 最长的 observe 阶段：这是离 25 s 超时最近的指标
    spans = {}
    cur = None
    for t in d.get("state_transitions") or []:
        if t.get("phase") == "observe" and t.get("target"):
            cur = (t["target"], t["stamp"])
        elif cur is not None:
            spans.setdefault(cur[0], []).append(t["stamp"] - cur[1])
            cur = None
    if cur is not None:
        spans.setdefault(cur[0], []).append(-1.0)
    worst = max(spans.items(), key=lambda kv: max(kv[1])) if spans else ("-", [0])
    lat = sorted(f["latency_wall_seconds"] for f in d.get("processed_frames") or []
                 if "latency_wall_seconds" in f)
    med = lat[len(lat)//2] if lat else 0.0
    print("%s|%s|%.1f|%.1f|%s|%s|%s|%d|%.3f" % (
        "JOINT-OK" if joint else "JOINT-FAIL",
        task.get("phase"), d.get("simulated_elapsed") or 0.0,
        d.get("wall_seconds") or 0.0, len(crossings), task.get("error"),
        "%s %.1fs" % (worst[0], max(worst[1])), len(lat), med))
except Exception as exc:
    print("NO_RESULT|-|0|0|-|%s|-|0|0" % type(exc).__name__)
JUDGE
)
    R=${LINE%%|*}; REST=${LINE#*|}
    PHASE=$(echo "$REST" | cut -d'|' -f1); SIM=$(echo "$REST" | cut -d'|' -f2)
    EWALL=$(echo "$REST" | cut -d'|' -f3); NX=$(echo "$REST" | cut -d'|' -f4)
    ERR=$(echo "$REST" | cut -d'|' -f6); OBS=$(echo "$REST" | cut -d'|' -f7)
    NLAT=$(echo "$REST" | cut -d'|' -f8); MED=$(echo "$REST" | cut -d'|' -f9)
    say "lap_$i 结束: $R   phase=${PHASE}   仿真 ${SIM}s   评价器墙钟 ${EWALL}s   越线 ${NX}   错误 ${ERR:-无}"
    say "          最长 observe = ${OBS}（上限 25 s）   单帧延迟 n=${NLAT} median=${MED}s"
    if [ "$R" = "JOINT-OK" ]; then pass=$((pass+1)); else fail=$((fail+1)); fi

    # 归档：确认共享目录里真的有 run_result.json 才删本地副本
    mkdir -p "$A"
    cp -a "$D/." "$A/" 2>/dev/null
    if [ -f "$A/run_result.json" ]; then
      rm -rf "$D"
      say "          已归档到 $A（本地副本已清）"
    else
      say "          [警告] 归档失败，本地副本保留在 $D"
    fi
  else
    T1=$(date +%s); EL=$((T1-T0)); times="$times $EL"
    say "lap_$i 脚本非零退出（墙钟 ${EL}s），看 $SHARE/lap_$i.console.log"
    say "          场景日志尾部："
    tail -6 "$D/scene.log" 2>/dev/null | sed 's/^/            /'
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
say "归档目录: $SHARE"
say "Windows 侧: D://智慧社区//比赛交付物//vm_runs//$(basename "$SHARE")"
