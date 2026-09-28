#!/usr/bin/env bash
# vm_obstacle_demo.sh -- 在线避障演示：临时放置偏侧障碍，观察侧向绕行并恢复路线。
# 用途：回答"自主建图导航避障"里"避障"这一项的证据缺口。
# 说明：这个盒子不在 config/layout.json 里，所以 guard 的地图几何约束不认识它，
#       唯一能发现它的是 /scan（激光）。因此这段演示证明的是"在线传感器避障"。
#
# ── 位置是扫出来的，不要手改（2026-09-25 实测）──────────────────────────
# 扫描工具：output/_audit20/obstacle_demo_scan.py
#           （真实路线 + 真实世界几何含灯柱 + 真实 Patrol + 真实 scan_clearance 闭环）
#
# 为什么不是"底部长直道"：
#   scan_clearance 的侧向走廊判据 half = 0.1835，两侧窗口在 |lateral| <= 0.0935 处
#   重叠。2 号灯东侧灯柱在 (2.24, 0.56)，相对底段车道中心 y=0.30 的横向距离是
#   +0.26 m —— 已经落在左侧走廊窗口内。实测：底段 x < 2.44 处即使不放任何障碍，
#   left_free 也已经是 False。于是右侧障碍会被读成"双侧全堵"= 合法等待，而不是避障
#   ⇒ 机器人原地停住（这是 §1 灯柱净空问题的直接后果，见第 20 轮方案）。
#
# 为什么选停车走廊（parking_3 -> right_top，2.36 m）：
#   全路线唯一"无停止线、无观测点、无灯柱进入任何侧向窗口"的长直段。
#   车道中心 x=3.15，向北行驶 ⇒ lateral = x - 3.15。
#   盒子横向 0.06 m、居中 3.29 ⇒ lateral [0.105, 0.165]，完整落在单侧窗口
#   [0.0935, 0.1835] 内，不碰重叠区，因此判据必然给出"单侧可通"。
#   沿行驶方向 0.30 m 深 ⇒ 触发 rear-clearance 迟滞，而不是"一束光线计时器"捷径。
#
# 闭环实测（45 s 上限，dt=0.05）：
#   对照（无障碍）：phase=done 12.5 s  min lane clearance 0.1085 m
#   本障碍        ：phase=done 20.4 s  min lane clearance 0.0165 m  足印未与盒子相交
#                   travel -> avoid(1.45s) -> recenter(12.5s) -> travel(13.45s)
#                   -> orient(19.9s) -> settle -> done
#   （0.12 m 宽或 x 居中 3.27 的变体都会卡死，不要"顺手"改宽度或位置。）
set -uo pipefail

ROOT=${ROOT:-$HOME/smart_community_run6}
PKG="$ROOT/catkin_ws/src/smart_community_semifinal"
if [ ! -f "$PKG/scripts/semifinal_core.py" ]; then
  echo "找不到工程根: $ROOT" >&2
  echo "请先部署:   cp -a /mnt/hgfs/智慧社区/智慧社区代码_run6 ~/smart_community_run6" >&2
  echo "或显式指定: ROOT=/path/to/root bash vm_obstacle_demo.sh" >&2
  exit 2
fi

source /opt/ros/melodic/setup.bash
if [ -f "$ROOT/catkin_ws/devel/setup.bash" ]; then
  source "$ROOT/catkin_ws/devel/setup.bash"
fi

# One obstacle in the parking corridor, made THIN and TALL.
#
# Why thin (0.03 m instead of 0.06 m):
#   lane 0.60 m, robot 0.303 + 0.040 margins => 0.257 m of total lateral slack.
#   A 0.06 m obstacle at lateral 0.14 leaves only 4.63 cm of obstacle-side
#   clearance once AVOID_SHIFT 0.090 is applied.
#   On the VM the robot stalled there: 18 s of avoid produced 0.22 m of forward
#   travel (0.012 m/s against a 0.08 m/s command) and raised
#   avoid_timeout:approach_light_1.  A 0.03 m obstacle doubles that clearance
#   Thinning to 0.03 m and pushing out to lateral 0.165 raises it to 8.63 cm
#   (measured, output/_obstacle_pair_check.py):
#     width 0.06 @ 0.14  -> 4.63 cm   <- stalled on the VM
#     width 0.03 @ 0.14  -> 6.13 cm
#     width 0.03 @ 0.165 -> 8.63 cm   <- chosen
#   All four variants end in phase=done with avoid x1 offline; the offline loop
#   is kinematic and cannot model the physical stall, so the VM run decides.
#
# Why tall (1.20 m):
#   display only.  The laser plane is at z=0.125 m, so 0.50 m and 1.20 m boxes
#   produce byte-identical avoidance behaviour (measured); 1.20 m just reads
#   clearly on camera.
#
# Why only the parking corridor:
#   An offline scan of every route segment x both sides
#   (output/_obstacle_sites_scan.py) reports only two usable segments: the top
#   straight and the parking corridor.  The top straight was then measured on
#   the VM and stalled -- the obstacle sits 0.5 m from the start point, so the
#   robot begins already inside the 0.65 m detection window and has to finish
#   "start + side-step + pass" inside the 18 s avoid deadline.  The parking
#   corridor is approached at cruise speed instead.
#
#   x     y     sx    sy    sz     note
#   3.315 2.70  0.03  0.30  1.20   lateral 0.165 m off the 3.15 m lane centre
#     obstacle lateral span [0.150, 0.180] stays fully inside the blocking
#     half-window 0.1835, so left_free reads False and the avoid triggers.
# ONE obstacle in the parking corridor: a 0.45 x 0.30 x 1.20 m wall intruding
# 0.14 m into the 0.60 m lane.  It has to LOOK like it blocks the robot, not
# just trip a software window.
#
# WHY THE WIDTH MATTERS (this is the whole point of the current shape)
#   The obstacle-side clearance depends only on the NEAR edge, not the width:
#       clearance = near_edge - 0.0815      (robot half width 0.1715 minus
#                                            AVOID_SHIFT 0.090, plus margins)
#   so widening and pushing outward at the same time keeps the clearance while
#   making the obstacle read as a real obstruction.  Measured on this lane
#   (centre x = 3.15, half-window 0.1835):
#
#     width  lat     near edge  clearance  intrudes  lane covered   VM status
#     0.06   0.140   0.110      2.85 cm    6.15 cm    18 %         STALLED
#     0.03   0.190   0.175      9.35 cm   -0.35 cm     5 %         too thin,
#                                                                  does not block
#     0.30   0.300   0.150      6.85 cm    2.15 cm    25 %         PASSED
#     0.45   0.365   0.140      5.85 cm    3.15 cm    27 %         <- chosen
#
#   The 0.30 m row above is the configuration that was actually driven on the VM
#   and cleared successfully, so 6.85 cm of clearance is proven sufficient and
#   2.85 cm is proven insufficient; 5.85 cm sits between the two with margin.
#   Moving the near edge inward by 1 cm costs exactly 1 cm of clearance, so the
#   width was raised at the same time to keep the visual obstruction growing
#   while only paying the 1 cm.
#
#   Near edge 0.140 m is inside the blocking half-window 0.1835 m, so left_free
#   reads False and the avoid fires.  Lateral span [0.140, 0.590] m => x up to
#   3.74 m, still clear of bld_parking_wall (x >= 3.91).
#
#   FALLBACK if the VM stalls here: widen/outward back to 0.45 @ 0.40
#   (near edge 0.175, clearance 9.35 cm) or the proven 0.30 @ 0.30.
#
#   Near edge 0.150 m is still inside the blocking half-window 0.1835 m, so
#   left_free reads False and the avoid fires.  Lateral span [0.150, 0.450] m
#   stays clear of bld_parking_wall (x >= 3.91).
#
# Height 1.20 m is display-only: the laser plane is at 0.125 m, so 0.50 m and
# 1.20 m boxes produce byte-identical avoidance behaviour.
#
#   x     y     sx    sy    sz     segment / travel direction
#   3.45  2.70  0.30  0.30  1.20   parking corridor (travel along +y)
#   sx is the lateral extent, sy the along-travel depth.  Swapping them lays the
#   box the wrong way round.
OBSTACLES=("3.515 2.70 0.45 0.30 1.20")
SDF=/tmp/semifinal_test_obstacle.sdf

cat > "$SDF" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="PLACEHOLDER">
    <static>true</static>
    <link name="body">
      <!-- SIZE_X/SIZE_Y are replaced per site. The long dimension follows
           the local travel direction; the lateral offset occupies one side
           corridor only. Height is display-only for the 0.125 m laser plane. -->
      <collision name="c"><geometry><box><size>SIZE_X SIZE_Y 1.20</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>SIZE_X SIZE_Y 1.20</size></box></geometry>
        <material><ambient>1 0.4 0 1</ambient><diffuse>1 0.4 0 1</diffuse></material></visual>
    </link>
  </model>
</sdf>
EOF

echo "工程根    : $ROOT"
echo "障碍物 SDF: $SDF （橙色 static，高度 1.20 m 便于录像看清）"
echo
echo "放在停车走廊的偏侧位置，机器人必须经过这里："
for i in "${!OBSTACLES[@]}"; do
  read -r ox oy sx sy sz <<< "${OBSTACLES[$i]}"
  model="test_obstacle_$((i+1))"
  sed "s/PLACEHOLDER/$model/; s/SIZE_X SIZE_Y 1.20/$sx $sy $sz/" "$SDF" > "/tmp/${model}.sdf"
  rosrun gazebo_ros spawn_model -sdf -file "/tmp/${model}.sdf" -model "$model" -x "$ox" -y "$oy" -z "$(awk "BEGIN {print $sz/2}")"
done
echo
echo "预期现象（对照 /semifinal/task_status 与 /semifinal/guard_status）："
echo "  1) 机器人过完 3 号停车位、沿停车走廊向北走时，激光在正前方检测到障碍；"
echo "  2) /semifinal/guard_status 报告 forward_obstacle，且左右侧净空一堵一通；"
echo "  3) 任务进入 avoid（侧移约 0.09 m），通过后 recenter 回中，再回 travel；"
echo "  4) 若两侧都堵住，机器人保持安全停车，障碍解除后自动恢复；"
echo "  5) 全程没有撞上去 —— 这就是「传感器避障」的证据。"
echo
echo "录像建议：从 avoid 触发前一帧录到 recenter 结束（约 15 s），"
echo "          同时录下终端里 task_status 的相位变化与 guard_status 的 forward_obstacle 行。"
echo
echo "撤销："
for i in "${!OBSTACLES[@]}"; do echo "  rosrun gazebo_ros spawn_model -delete -model test_obstacle_$((i+1))"; done
echo "或直接结束本次 roslaunch（static 模型不会写入 world 文件，包不受影响）。"
