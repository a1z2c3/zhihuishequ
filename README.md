# 智慧社区自主巡检代码

蒙的全队智慧社区复赛工程。此文件夹整理了 `batch1_run6` 所用版本的 ROS 功能包、编译脚本和运行资源。

整理时已核对该圈启动日志中的功能包路径，并逐文件比较教学虚拟机与本机源码，内容一致。运行代码保持原样；编译脚本限制为两路编译，便于教学虚拟机运行。

## 代码能做什么

机器人在 Gazebo 场景中从起点出发，按指定顺序经过 19 个航点，识别两处交通灯，在 A、B 街区停车观察并统计社区/非社区人员，再识别三个车位的车牌，最后返回起点。

- **建图与定位**：激光雷达配合 Gmapping，使用 `map → base_footprint` 变换控制行驶。
- **人物和车牌识别**：1280×960 RGB 相机；ORB 特征匹配、RANSAC 几何筛选和图案内容复核。根据物料尺寸与 PnP 估计人员位置，跨视角合并同一街区内的同一实例。
- **交通灯识别**：从相机图像定位灯框、判断颜色；观察到红/黄转绿并连续确认新鲜绿灯后，检查是否有足够时间通过路口。
- **任务控制**：依次行驶、转向、等待、观察、返回；观察至少停留 2 个仿真秒，并等待不少于 3 个不同时间戳的有效完成帧。
- **速度约束**：`traffic_guard_node.py` 是唯一 `/cmd_vel` 发布者，检查车身、配置禁区、灯柱、停止线、传感器与命令的新鲜度。
- **结果输出**：人员统计、车牌、带框图像和 JSON 事件；独立评价器记录任务状态、轨迹和停止线穿越情况。

## 文件结构

```text
智慧社区代码_run6/
├── README.md
├── setup_and_check.sh                 # 环境检查、测试、编译和启动文件解析
└── catkin_ws/
    └── src/
        └── smart_community_semifinal/
            ├── CMakeLists.txt
            ├── package.xml
            ├── scripts/              # Python 节点与算法模块
            ├── src/signal_plugin.cpp # Gazebo 信号灯插件
            ├── launch/               # 场景、建图、感知、约束与巡检启动入口
            ├── config/               # 路线、规则、场景配置
            ├── urdf/                 # 机器人与传感器模型
            ├── worlds/               # Gazebo 世界
            ├── models/               # 场景模型、网格与贴图
            ├── assets/               # 人物/车牌参考图库、资源清单
            ├── maps/                 # 已保存的激光地图 PGM/YAML
            └── tools/                # 运行评价、测试、几何校验和场景生成代码
```

主要入口是 `launch/patrol.launch`。`patrol_node.py` 管任务，`official_perception_node.py` 管人物/车牌与空间账本，`signal_perception_node.py` 管交通灯，`traffic_guard_node.py` 管最终速度。`tools/evaluate_run.py` 只负责评价，Gazebo 真值不进入控制和识别节点。

图片、模型和地图是代码所用资源。文件夹不包含技术方案、论文、答辩稿、历史压缩包、运行日志或证据截图。

## 环境与编译

已运行环境：Ubuntu 18.04、ROS Melodic、Python 2.7.17、Gazebo 9、OpenCV 3.2.0、NumPy、Pillow 5.1.0。ROS 节点使用系统 Python 2.7；`build_official_scene.py`、`validate_geometry.py` 和 `validate_semifinal.py` 是 Python 3 开发工具，正常启动巡检不需要运行它们。

将整个文件夹复制到虚拟机的 Linux 本地目录，例如 `~/smart_community_run6`。不要直接在 VMware 共享目录 `/mnt/hgfs` 内编译，以免符号链接或权限受限。如果共享目录名称仍为“智慧社区”，可执行：

```bash
cp -a /mnt/hgfs/智慧社区/智慧社区代码_run6 ~/smart_community_run6
cd ~/smart_community_run6
```

目标目录应是新目录。教学 VM 已有依赖时直接编译；缺少依赖时安装：

```bash
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gmapping \
  ros-melodic-map-server ros-melodic-robot-state-publisher \
  python-opencv python-numpy python-pil fonts-wqy-microhei

source /opt/ros/melodic/setup.bash
bash setup_and_check.sh
source catkin_ws/devel/setup.bash
```

`setup_and_check.sh` 恢复节点的可执行权限，执行 14 项运行契约、编译 C++ 插件并解析启动文件。复制到新目录后需重新编译，本文件夹不携带旧的 `build/`、`devel/` 或 `.pyc`。

## 启动一圈

在虚拟机桌面终端执行：

```bash
cd ~/smart_community_run6
source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
roslaunch smart_community_semifinal patrol.launch \
  evidence_dir:="$HOME/semifinal_runs/run_$(date +%Y%m%d_%H%M%S)/evidence"
```

启动后自动完成巡检。任务状态可在另一个已加载环境的终端查看：

```bash
rostopic echo /semifinal/task_status
rostopic echo /semifinal/street_summary
```

每条 `rostopic echo` 会持续显示，按 `Ctrl+C` 后再执行下一条。任务完成时状态为 `phase: done`、`index: 19`、`target: finish`。

若无桌面显示，先安装并启动 Xvfb，再用 `gui:=false`：

```bash
sudo apt install xvfb
Xvfb :99 -screen 0 1280x960x24 -nolisten tcp -ac &
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch smart_community_semifinal patrol.launch gui:=false \
  evidence_dir:="$HOME/semifinal_runs/run_$(date +%Y%m%d_%H%M%S)/evidence"
```

软件渲染下世界配置最多为 0.2 倍实时速度，一圈约需 18 分钟实际时间。`gui:=false` 只关闭 Gazebo 界面，相机仍需要可用的显示/OpenGL 环境。

## 同时记录完整评价

只需看机器人运行时，使用上面的启动方式即可。需要像 `batch1_run6` 一样记录完整轨迹与评价时，先结束上一轮，在三个终端中依次执行下列步骤。每个终端都先执行：

```bash
cd ~/smart_community_run6
source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
```

终端 1：启动场景与各节点，暂不启动任务。下面用 `manual_run_01` 作为本次目录名，每次运行换一个新名称。

```bash
roslaunch smart_community_semifinal patrol.launch start_task:=false \
  evidence_dir:="$HOME/semifinal_runs/manual_run_01/evidence"
```

终端 2：等场景加载后先启动评价器。

```bash
python catkin_ws/src/smart_community_semifinal/tools/evaluate_run.py \
  _output_dir:="$HOME/semifinal_runs/manual_run_01/evaluation"
```

终端 3：检查地图变换，看到连续输出后按 `Ctrl+C`，再启动任务。

```bash
rosrun tf tf_echo map base_footprint

rosrun smart_community_semifinal patrol_node.py \
  _layout:="$(rospack find smart_community_semifinal)/config/layout.json"
```

完成后查看 `~/semifinal_runs/manual_run_01/evaluation/run_result.json`。评价器退出不一定代表成功，应确认 `task.phase == "done"`；同时检查 `body_violations` 和 `stop_crossings`。结束场景时回到终端 1 按 `Ctrl+C`。

## 刚才跑的 batch1_run6

以下数字重新读取自 `batch1_run6.tar.gz` 中的原始评价记录和图像事件，未使用上一轮 218.55 秒的结果。

| 项目 | 本圈记录 |
|---|---|
| 最终任务 | `done`，完成 19 个航点，回到起终点 |
| 评价器记录的仿真历时 | 210.653 秒，约 211 秒 |
| 完成时的仿真时间戳 | 210.955 秒 |
| 墙钟历时 | 1065.633 秒，约 17 分 46 秒 |
| A 街区 | 社区 4 人、非社区 1 人，共 5 人 |
| B 街区 | 社区 4 人、非社区 1 人，共 5 人 |
| 三个车位 | 苏AB8Q62、鄂D7B5Q2、苏APL12A |
| 停止线穿越 | 11.618 秒、116.615 秒，两次均为绿灯 |
| 车身约束检查 | 2240 个实际位姿采样，配置禁区/灯柱相交 0 次 |
| 归档图像 | 41 张物料图、5 张信号灯图 |

实际路线为：起点 → 1 号路口 → A 北侧 → 左上角/左侧车道 → B 西侧 → A 南侧 → B 东侧 → 2 号路口 → 右下角/右侧车道 → 1、2、3 号车位 → 右上角 → 返回起点。

七个观测点的实际识别记录如下。时间为图像采集的仿真时间戳，帧数按不同来源图像计数。

| 观测点 | 图像时间范围（秒） | 帧数 | 识别内容 |
|---|---:|---:|---|
| A 北侧 `street_a_north` | 29.555–30.997 | 6 | resident_7、resident_1 |
| B 西侧 `street_b_west` | 67.187–68.780 | 6 | resident_9、resident_6、resident_2 |
| A 南侧 `street_a_south` | 81.238–82.758 | 6 | visitor_F1、resident_10、resident_16 |
| B 东侧 `street_b_east` | 90.557–91.965 | 6 | resident_14、visitor_F2、resident_9 |
| 1 号车位 `parking_1` | 144.585–145.985 | 6 | 苏AB8Q62 |
| 2 号车位 `parking_2` | 159.193–160.433 | 5 | 鄂D7B5Q2 |
| 3 号车位 `parking_3` | 174.243–175.611 | 6 | 苏APL12A |

`resident_9` 在 B 街区两个视角中都被看到，空间账本最终只计一次。代码不是固定拍 6 帧：本圈 2 号车位实际为 5 帧，其他观测点为 6 帧；离开条件由停留时间、有效完成帧和人员空间确认共同决定。

这里描述的是本圈实际结果。当前车牌识别依赖已知参考图，并非任意车牌 OCR；底盘与里程计是简化仿真，慢速运行也不等于实时或实车性能。`maps/` 保留此前同场景自主运行保存的 Gmapping 地图，不是本圈重新导出的地图；正常巡检会重新建图。

人物、号牌、场景贴图来自提供的官方复赛物料，来源信息保留在 `assets/manifest.json`，按赛事授权范围使用。ROS、Gazebo、OpenCV 等依赖需要安装，不作为二进制随此文件夹分发。
