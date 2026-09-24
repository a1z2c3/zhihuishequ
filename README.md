# 蒙的全队 · 智慧社区复赛 v2

2026-09-23。本版本在 v1 复核基础上补齐车身走廊监督、红灯相位安全边界、受限逐字符车牌识别和 Melodic 回归测试。下面的运行命令和能力边界以当前代码为准；开发验收不等于比赛验收。

本轮安全增强：车道约束按道路段带状区域并集和真实 footprint 四角判定，转弯处不再用圆形中心线距离误伤；预测横移将优先限幅致违规的横向分量，已越界时由 patrol 以 0.04 m/s 向内恢复，并在 30 秒内无法恢复时显式失败。车牌字符校验加入对称距离变换形状分数、位置槽约束和高门槛拒识；留一张牌不在模板库时输出 `?`，不把相似字伪报为确定结果。上述逻辑不绕过 guard，`/cmd_vel` 仍只有 guard 发布。

## 运行

已用教学虚拟机验证：Ubuntu 18.04、ROS Melodic、Python 2.7.17、OpenCV 3.2.0、Pillow 5.1.0、Gazebo 9.0.0。节点用系统 `python`，不要安装不存在的 `python3-rospy`、`python3-tf` 或混用 Noetic。Python 3 仅用于开发机资源生成与几何审计。

```bash
source /opt/ros/melodic/setup.bash
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gmapping \
  ros-melodic-map-server ros-melodic-robot-state-publisher \
  python-opencv python-numpy python-pil fonts-wqy-microhei
bash setup_and_check.sh
source catkin_ws/devel/setup.bash
roslaunch smart_community_semifinal patrol.launch
```

无桌面环境另需安装 Xvfb。仅 `gui:=false` 不提供相机所需的 OpenGL 显示：

```bash
Xvfb :99 -screen 0 1280x960x24 -nolisten tcp -ac &
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch smart_community_semifinal patrol.launch gui:=false verbose:=true
```

软件渲染不能达到 15 帧/墙钟秒。世界采用 1 ms 步长、200 Hz 物理更新上限，即**最多 0.2 倍实时速度**，为传感器处理留时间；配置 15 Hz 是仿真时间目标，实际值见日志。这是慢速集成测试配置，不是实时性能证明。提高物理更新率必须重测图像延迟与绿灯许可，不能简单放宽新鲜度阈值。

## 闭环与接口

`patrol_node` 用 `map→base_footprint` 按指定顺序行驶，在观测点停稳后等待至少 3 个不同时间戳的稳定参考匹配帧。`official_perception_node` 对相机像素做参考匹配，用物料尺寸、CameraInfo 和 PnP 估计位置，经同一时刻 TF 进入 A/B 空间账本；车牌还要通过当前车位的独立空间归属校验。车牌路线完成与字符 OCR 分离：整牌参考匹配负责稳定观测，逐字符模板分类独立给出置信度、拒识和跨帧一致性状态；低置信度不会被伪造成确定结果。相机改为 1280×960 RGB，保持 4:3 视场，当前不渲染未使用的深度图。

`signal_perception_node` 用灯具安装位置缩小搜索区，再从图像寻找亮灯圆和黑色外壳，发布 ROI 与灯色。安装位置不是灯色真值。`traffic_guard_node` 独占 `/cmd_vel`，检查定位、激光、命令超时、完整车身预测轨迹、车道走廊、灯柱、停止线及新鲜视觉许可。车道漆不进入激光地图；走廊约束由独立仲裁层根据 `lane_centerline`、真实车身 footprint 和预测扫掠轨迹执行。控制节点不订阅 `/gazebo/model_states` 或灯相位真值，不读 `scene_instances_for_evaluation_only.json`。

Gmapping 的运动噪声与匹配步长按这个小场地的理想仿真里程计设置。默认参数曾在返回段产生约 6 cm 地图误差并触发车身约束停车，现有参数的实际误差见运行记录，不能照搬为实车配置。若移动/转向连续 40 仿真秒没有足够进展，任务输出 `motion_stalled` 并停车；真实障碍等待另有 90 秒上限。持续安全保持、等灯和观测分别有 90/90/25 仿真秒的失败上限，并有 600/600/180 墙钟秒兜底；短暂安全保持暂停等灯/观测计时。patrol 和 guard 用墙钟节拍继续检查与发布零速，不依赖暂停的仿真 `/clock` 唤醒。

放行必须获得 3 个连续新鲜绿灯帧。绿灯起点来自目击有效红/黄→绿跳变；若此前已目击并锁定与配置周期一致的相位时钟，首次直见绿灯也可推算最近起点，但不会把当前帧误当作刚亮。未锁定时钟又未目击跳变则安全等待。丢帧/未知观测撤销当前许可，但不会凭空重置已验证的起点；红/黄立即撤销。短暂假红不能建立新绿灯起点。15 秒绿灯下界仅来自本仿真周期，**不是倒计时识别或任意灯具都成立的保证**。改灯 ID 需在 `signal_cycle.offsets_s` 显式配置对应偏移，否则相位时钟拒绝未知灯；修改周期还须同步检查世界插件和放行余时。进入路口后不会因灯离开视场而停在中央。标准 move_base 客户端在 approach 航点等待新鲜、匹配停止线的 guard `entry_ready` 后才发送过线 goal。

| 话题 | 发布者 | 内容 |
|---|---|---|
| `/semifinal/cmd_vel_requested` | patrol | 请求速度 |
| `/cmd_vel` | guard，唯一发布者 | 最终速度 |
| `/semifinal/active_stop` | patrol | 停止线和清空约束 |
| `/semifinal/observation` | patrol | 观测点、街区、停稳时间 |
| `/semifinal/task_status` | patrol | 路线索引、阶段、失败原因 |
| `/semifinal/signal_roi` | signal_perception | 图像灯框、来源时间戳 |
| `/semifinal/visual_signal` | signal_perception | 灯色、分数、来源时间戳 |
| `/semifinal/guard_status` | guard | 放行许可、停车原因 |
| `/semifinal/events` | official_perception | 对应证据图的检测记录 |
| `/semifinal/frame_status` | official_perception | 有效帧握手、墙钟耗时 |
| `/semifinal/street_summary` | official_perception | A/B 社区与非社区人数 |

标注图：`/semifinal/annotated`、`/semifinal/signal_annotated`。证据默认在 `~/semifinal_evidence_v2`，可用 `evidence_dir:=...` 指定，每次运行独立目录。JSON 与 PNG 共用来源帧编号/时间戳。分类分数未经概率校准。

## 验证

```bash
# 目标 VM：运行契约，包含车身/灯柱/红灯安全边界的路线运动学测试
python catkin_ws/src/smart_community_semifinal/tools/test_runtime.py
# 目标 VM：车道走廊、逐字符 OCR、避障和导航合同
python catkin_ws/src/smart_community_semifinal/tools/test_lane_geometry.py
python catkin_ws/src/smart_community_semifinal/tools/test_plate_ocr.py
python catkin_ws/src/smart_community_semifinal/tools/test_perception_contracts.py
python catkin_ws/src/smart_community_semifinal/tools/test_obstacle_closed_loop.py
python catkin_ws/src/smart_community_semifinal/tools/test_plan_route.py
python catkin_ws/src/smart_community_semifinal/tools/test_navigation_contracts.py
# 开发机 Python 3 + NumPy/OpenCV/Pillow
python3 catkin_ws/src/smart_community_semifinal/tools/validate_geometry.py --out geometry.json
```

当前 Melodic Python 2.7 回归基线为：运行契约 60 条、车道几何 10 条、车牌 OCR 12 条、感知契约 10 条、避障闭环 4 条、A* 规划 8 条、导航合同 5 条，共 109 条。车牌测试同时覆盖已知模板的旧 Gazebo 矫正裁片（3/3 正确）、留一牌未知字符拒识、形状候选保留和小角度旋转；这不是陌生牌开放集准确率。教学 VM 的整圈评价必须看每次 `INDEX.md`：路线完成要求参考匹配 quorum、独立车身/红灯越线和街区计数全部满足；OCR 作为独立质量通道单独报告，离线测试不替代集成验收。

人物朝向按官方示意图的箭头约束建模：A 社区覆盖北、南、西三个方向，B 社区覆盖北、东两个方向。当前场景保持 A/B 各 8 人（每个社区 7 名社区人员、1 名非社区人员），并为侧向卡片配置独立观察航点；`layout.json` 中的 `person_orientation_policy` 与方向回归测试用于防止重新生成场景时退回单一朝向。

几何工具检查四角投影、朝向、像素与车身，不达标退出 1；不是动力学或随机定位误差证明。Gazebo 放置服务采集的静态测试与全程自主测试分开报告。地图来源与覆盖见包内地图说明。

另一个已 source 环境的终端可启动独立评价：

```bash
python catkin_ws/src/smart_community_semifinal/tools/evaluate_run.py _output_dir:=$HOME/semifinal_evaluation
```

评价端只读 Gazebo 真值，并记录状态、实际轨迹、灯相位、定位估计、检测事件与相机延迟。应在启动任务前运行以覆盖起点；它在任务完成/失败、700 仿真秒或1500墙钟秒时结束。`run_result.json` 的 `task.phase` 必须为 `done` 才能称任务完成，不能把评价进程退出当作成功。

教学 VM 上的一圈留证入口为 `bash vm_run_lap.sh`（可传保存目录；无桌面时先配置 Xvfb 和 `DISPLAY`）。标准导航链可用 `RUN_MODE=navigation bash vm_run_lap.sh` 单独留证，默认仍是 patrol。脚本先启动场景并等待机器人出现，再启动独立评价器并确认订阅，最后启动任务；每次创建独立 `docs/evidence/patrol_*` 或 `navigation_*`，保留场景/任务/评价日志、相机标注、源码 SHA-256、`run_result.json` 和结果索引。只有任务完成、两次绿灯越线、零实际车身/红灯跨线违规、本场景总计 14 个社区人员 + 2 个非社区人员以及三牌逐字核验齐全才标 PASS；侵入额外 2 cm 工程余量但未压实际车道线的位姿单列为告警。旧教学 VM PASS 证据对应历史场景，当前场景人数和几何已更新，必须重新跑圈后才能作为当前版本证据。

可选的保存地图全局规划器位于 `catkin_ws/src/smart_community_semifinal/tools/plan_route.py`。它从 `slam_map.pgm` 读取占用栅格，按车体半径和安全余量做膨胀，显式约束场地边界与规则区，使用禁止对角穿角的 A* 在语义航点之间生成中间航点，并保留红绿灯门控与观测元数据。规划结果写入 `config/layout_astar.json`；`navigation.launch` 默认仍使用经过运动学契约验证的 `layout.json`，验证通过后可传入 `route_layout:=...` 启用 A* 路线。全局 costmap 的可选 `navigation_map.pgm/.yaml` 是规则区叠加图，AMCL 仍使用原始 SLAM 图定位。

## 能力边界

- 未标注坐标、人物位置和 0.145 m 身高是有状态标记的重建假设。
- 21 张图库含 18 个人物图案和 3 张车牌；当前场景是 16 个人偶（14 个社区人员、2 个非社区人员）和 3 张车牌。车牌字符识别是受限物料域的透视切分/模板分类，以孔洞和左边缘形状排除已知易混字，并与整牌参考匹配交叉核验；低置信字符会拒识，陌生牌仍不等同于开放集通用 OCR。参考牌与识别牌不符会标记 `unexpected_label`，不会静默丢弃检测。
- 底盘是 Gazebo 简化平面运动插件，里程计理想化；未验证真实麦轮接触动力学、打滑和实车定位。
- 激光地图只反映真实几何碰撞体，不把地面线虚构成墙；规则区域另行建模。
- 未完成冻结版本的 30 次随机相位整圈、断流故障注入及正式比赛视频。参赛编号、队员和最终四件套仍需补齐。
