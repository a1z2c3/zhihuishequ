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

`image` 使用相对文件名，允许源码包搬移。地图包含激光扫描面实际可见的立牌、车辆背景与灯柱；地面车道线不虚构为墙，比赛规则另由 `config/layout.json` 提供。

地图是当前重建场景的实测结果，不是主办方发布的精确地图，也未做实车验证。运行 `mapping.launch` 时由 gmapping 发布地图，禁止同时用 map_server 覆盖 `/map`。仅离线查看时可运行 `rosrun map_server map_server slam_map.yaml`。
