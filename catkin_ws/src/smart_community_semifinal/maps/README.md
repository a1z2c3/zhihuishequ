# 实测激光地图

来源：教学 VM 中这次完成全部 19 个路线目标的 Gmapping 运行，结束后由 `map_server/map_saver` 导出。地图像素 288×288，配置如下。

```yaml
image: slam_map.pgm
resolution: 0.020000
origin: [-1.000000, -1.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

```

`image` 使用相对文件名，允许源码包搬移。地图包含激光扫描面实际可见的立牌、车辆背景、灯柱，以及街区禁行岛内的静态体量与停车位背墙；地面车道线不虚构为墙，比赛规则另由 `config/layout.json` 提供。

禁行岛内的静态体量（`worlds/official_semifinal.world` 里的 `building_a` / `building_b` / `building_a_north` / `parking_wall`）是本次地图信息量的主要来源。它们的每个外廓都由 `_诊断工具/audit17_lidar_neutrality.py` 三重闸门验收：gate 0 不与任何卡片重叠，gate 1 不侵入任何合法位姿的足印，gate 2 在 4000 个合法位姿 × 2 个速度下 `forward_obstacle` / `left_free` / `right_free` 变化均为 0。离线射线占用统计为 1047 个栅格（0.419 m²），是不含这些体量时的 2.8 倍。改动任何一个外廓都必须重跑该闸门与 `audit_occlusion.py`。

地图是当前重建场景的实测结果，不是主办方发布的精确地图，也未做实车验证。运行 `mapping.launch` 时由 gmapping 发布地图，禁止同时用 map_server 覆盖 `/map`。仅离线查看时可运行 `rosrun map_server map_server slam_map.yaml`。
