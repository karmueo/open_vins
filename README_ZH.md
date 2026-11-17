# OpenVINS 中文使用说明

本仓库支持 MCAP 离线计算并导出 TUM 轨迹、MCAP 按采集速度回放，以及 ROS 2 在线订阅。目标环境为 Ubuntu 24.04 + ROS 2 Jazzy，需具备仓库构建依赖、`rosbag2_storage_mcap` 和 `rviz2`；以下从仓库根目录操作，`/path/to/...` 均须替换为实际路径。系统不包含回环与全局优化，轨迹会累积漂移，稀疏点不是持久地图；绝对精度需另行评测。

## 1. 环境与构建

```bash
# 加载 ROS 2 Jazzy 环境
source \
  /opt/ros/jazzy/setup.bash
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 脚本路径（位置参数） | 指定环境脚本 | 输入文件 | 必填，无默认值；此处为 Jazzy 或下文工作区的 `setup.bash` |

```bash
# 构建处理节点及自定义消息接口
MAKEFLAGS=-j4 colcon build \
  --packages-up-to openvins_slam ego_msgs \
  --parallel-workers 1 \
  --cmake-args \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DBoost_DIR=/usr/lib/x86_64-linux-gnu/cmake/Boost-1.83.0 \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

OpenCV 路径用于与系统 `cv_bridge` 保持一致；构建设置依据见[包说明](openvins_slam/README.md#构建)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `MAKEFLAGS`（环境变量） | 限制 Make 并行任务数 | 行为控制项 | 未设置时依赖构建环境；示例 `-j4` |
| `--packages-up-to` | 选择目标包及依赖 | 输入值 | 未指定时按 colcon 包选择规则；示例为 `openvins_slam ego_msgs` |
| `--parallel-workers` | 限制同时构建的包数 | 行为控制项 | 未指定时依赖执行器；示例为 `1` |
| `--cmake-args` | 向 CMake 传递后续参数 | 输入值 | 未指定时不额外传参 |
| `-DOpenCV_DIR`（CMake 配置项） | 指定 OpenCV CMake 配置目录 | 输入目录 | 未设置时由 CMake 查找；示例为 Ubuntu 系统路径 |
| `-DPython3_EXECUTABLE`、`-DPYTHON_EXECUTABLE`（CMake 配置项） | 指定 Python 解释器 | 输入文件 | 未设置时由构建配置查找；示例为 `/usr/bin/python3` |

```bash
# 加载已构建的工作区
source \
  install/setup.bash
```

每个新终端都要在仓库根目录先加载 Jazzy，再加载工作区环境；`source` 参数见上表。

## 2. 输入约定

MCAP 流程使用 `/stereo/image_raw`（`sensor_msgs/msg/CompressedImage`，横向拼接，左半 cam0、右半 cam1）和 `/imu`（`sensor_msgs/msg/Imu`，角速度 rad/s、加速度 m/s²）。准备含双目 fisheye 内参、`T_cam_imu`、`resolution`、IMU 偏置及噪声的 `calib.yaml`；程序将平移 mm 转 m、陀螺偏置 deg/s 转 rad/s，字段要求见[标定加载实现](openvins_slam/src/calibration.cpp)。在线订阅使用原始图像和另一套配置，见第 4.2 节。

**换数据包必须重新核对 IMU 时间偏移。** 下文 `imu_time_offset_sec:=0.025` 仅为 EGO `job_004.mcap` 示例：原标定约 −5.089 ms 曾导致发散，独立诊断范围约 +25～30 ms，依据见[包说明](openvins_slam/README.md#参数)。未显式传入时使用 `ROS.ImuTimeOffsetSec`，标定未设置时为 `0.0`。发散时先检查时间偏移，再检查外参与偏置约定。

## 3. MCAP 离线计算并导出 TUM 轨迹

先编辑 [slam_from_bag.yaml](openvins_slam/config/slam_from_bag.yaml) 中 `slam_from_bag.ros__parameters` 下的配置，再从仓库根目录启动。设置 `rviz2: true` 可同时打开 RViz2，不改变 `realtime` 指定的处理速度。

**显式指定输出目录时，同名旧 MCAP 和轨迹会在成功结束时被原子替换，需保留时先另存。** 输出包保留全部原始消息并追加结果，请预留大于输入包的磁盘空间。省略 `outdir` 时自动创建带时间的结果目录。TUM 始终保存；仅在 `save_mcap=true` 时保存 MCAP。默认 `save_mcap=true`、`rviz2=false`。

```bash
# 使用源码目录中的 YAML，离线处理 MCAP 并导出 TUM 轨迹
ros2 launch openvins_slam slam_from_bag.launch.py \
  config:="$(pwd)/openvins_slam/config/slam_from_bag.yaml" \
  in_bag:=/path/to/job_002.mcap
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `config` | 选择 ROS 2 参数 YAML | 输入文件 | 默认包安装目录的 `config/slam_from_bag.yaml` |
| `in_bag` | 指定输入 MCAP | 输入文件 | 必填，必须是存在的普通文件 |
| `cameras` | 指定相机与 IMU 标定 | 输入文件 | 选传；省略或为空时查找输入包同目录的 `calib.yaml`，找不到时报错 |
| `outdir` | 指定结果目录 | 输出目录 | 选传；省略或为空时创建 `<启动工作目录>/output/<输入包名>_<YYYYMMDDHHMMSS>`；显式指定时直接使用该目录 |

以上路径均支持绝对路径、相对启动工作目录的路径和 `~` 展开。可将 YAML 复制到其他位置后传给 `config`。省略该参数会读取安装副本；修改源码 YAML 后需重新构建安装，才能更新安装副本。上面的显式路径会直接读取编辑后的源码文件。

默认输出目录例如 `output/job_002_20261116143025`，输入包名去掉最后一个扩展名，时间使用启动时的系统本地时间。自动生成的目录已存在时会报错，避免同秒启动覆盖结果。自动生成目录仅适用于本节 launch；直接运行节点和 `bag_slam.launch.py` 仍要求显式传入 `outdir`。

旧 launch 参数 `params_file` 已改为 `config`。`in_bag`、`cameras`、`outdir` 不再放入 YAML；旧参数或 YAML 中仍包含这些路径项时会报错并提示迁移。

### YAML 配置项

文件采用 ROS 2 标准结构，不是相机标定文件，也不是 OpenVINS 的 `estimator_config.yaml`：

```yaml
slam_from_bag:
  ros__parameters:
    save_mcap: true
    rviz2: false
    realtime: false
```

结果名为 `openvins_<输入包名>_<配置文件名>_trajectory.txt`，可选 MCAP 为 `openvins_<输入包名>_<配置文件名>.mcap`，两者去掉最后一个扩展名。没有配置文件时配置名为空并保留下划线。旧输出参数已移除，详情见[包说明](openvins_slam/README.md)。

此处仅展示结构，完整配置见链接文件。YAML 仅存放处理参数。字符串空值写成 `""`，不要写成 YAML 的 `null`；布尔值使用 `true` / `false`，数值不加引号，秒数使用小数形式。

随包调优配置基于 EGO 800Hz `job_002.mcap`，使用 `-0.007 s` 时间修正；默认基线配置读取标定时间修正，换包必须重新核对。表中的“示例值”来自随包 YAML，“省略时”指删除对应键后节点采用的行为。

<a id="6-参数速查"></a>

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 示例值 / 省略时的行为 |
| --- | --- | --- | --- |
| `save_mcap` | 保存原始消息与结果 MCAP | 行为控制项 | 默认 `true`；设为 `false` 可仅保存轨迹 |
| `rviz2` | 启动 RViz2 | 行为控制项 | 默认 `false`；保持 `realtime` 速度，EOF 后保留画面 |
| `stereo_topic` | 压缩拼接双目话题 | 输入值 | 示例及省略时均为 `/stereo/image_raw`；必须非空且与 IMU 话题不同 |
| `imu_topic` | IMU 话题 | 输入值 | 示例及省略时均为 `/imu`；必须非空且与双目话题不同 |
| `realtime` | 按采集时间回放或尽快计算 | 行为控制项 | 示例及省略时均为 `false`；设为 `true` 时按时间回放，EOF 后保持展示并等待 Ctrl+C |
| `use_sim_time` | 使用 ROS 仿真时钟 | 行为控制项 | `realtime=true` 或 `rviz2=true` 时节点自动启用，并发布 `/clock` |
| `half_resolution` | 每目宽高减半后跟踪 | 行为控制项 | 示例及省略时均为 `true`，如 1920×1080 → 960×540 |
| `apply_imu_bias` | 预减标定偏置 | 行为控制项 | 示例及省略时均为 `true`；输入已扣偏置时改为 `false` |
| `record_debug` | 将跟踪图与稀疏点写包 | 行为控制项 | 示例及省略时均为 `false`；开启会明显增大文件体积 |
| `imu_time_offset_sec` | 固定 IMU 时间修正，单位秒 | 输入值 | 调优示例 `-0.007`；省略时取标定的 `ROS.ImuTimeOffsetSec`，标定缺失时为 `0.0`；必须有限且在 `[-10, 10]` 内 |
| `try_zupt` | 零速度更新开关 | 行为控制项 | 示例 `false`；省略时继承估计器配置，当前 `false` |
| `num_pts` | 运行阶段两目合计目标特征数 | 输入值 | 示例 `400`；省略时继承估计器配置，当前 `200`，每目分配 `floor(num_pts/2)` |
| `init_max_features` | 初始化阶段两目合计目标特征数 | 输入值 | 整数且至少 `15`；示例 `100`；省略时继承估计器配置，当前 `50`，每目分配向下取整的一半 |
| `max_clones` | 滑窗克隆数 | 输入值 | 示例 `15`；省略时继承估计器配置，当前 `11` |
| `max_slam` | 状态内 SLAM 特征上限 | 输入值 | 示例 `100`；省略时继承估计器配置，当前 `50`；允许 `0` |
| `max_slam_in_update` | 单批 SLAM 更新特征上限 | 输入值 | 示例 `50`；省略时继承估计器配置，当前 `25` |
| `max_msckf_in_update` | 单次 MSCKF 更新特征上限 | 输入值 | 示例 `80`；省略时继承估计器配置，当前 `40` |
| `fast_threshold` | FAST 检测阈值 | 输入值 | 示例 `20`；省略时继承估计器配置，当前 `20` |
| `grid_x` | 水平特征提取网格数 | 输入值 | 示例 `5`；省略时继承估计器配置，当前 `5` |
| `grid_y` | 垂直特征提取网格数 | 输入值 | 示例 `5`；省略时继承估计器配置，当前 `5` |
| `min_px_dist` | 跟踪分辨率下的最小特征间距 | 输入值 | 示例 `20.0`；省略时继承估计器配置，当前 `10`；有限正数向上取整为整数像素 |
| `calib_cam_intrinsics` | 在线相机内参标定 | 行为控制项 | 示例 `false`；省略时继承估计器配置，当前 `true` |
| `calib_cam_extrinsics` | 在线相机与 IMU 外参标定 | 行为控制项 | 示例 `false`；省略时继承估计器配置，当前 `true` |
| `calib_cam_timeoffset` | 在线相机与 IMU 时间偏移标定 | 行为控制项 | 示例 `false`；省略时继承估计器配置，当前 `true` |

数量参数须为整数：`init_max_features` 至少 15，`num_pts` 及克隆数至少 2，`max_slam` 至少 0，其余至少 1。网格数不能超过对应跟踪图像尺寸，像素间距不能超过跟踪图像短边。非法数值在输出创建前报错。IMU 内参、重力敏感项及动态初始化中的标定优化保持关闭。

随包调优配置已使用增强视觉约束。其他配置需要对照时，可将同一 `ros__parameters` 下的以下项改为实验值，其余项保留：

```yaml
num_pts: 400
init_max_features: 100
max_clones: 15
max_slam: 100
max_msckf_in_update: 80
```

需要复现历史固定标定行为时，将以下三项改为 `false`：

```yaml
calib_cam_intrinsics: false
calib_cam_extrinsics: false
calib_cam_timeoffset: false
```

外参在线标定开启时，相机位姿与协方差跟随估计状态，相机边使用动态 `/tf`；关闭时使用 `/tf_static`。固定时间修正作用于 IMU 计算副本，在线残差从零开始，原图像、结果及 TUM 时间戳不变。结束日志输出固定修正 `s`、残差 `d`、等效修正 `s-d` 和最终内外参，内参标明跟踪分辨率。

TUM 无表头，每行 `timestamp tx ty tz qx qy qz qw`，坐标系为 `odom → imu`，平移单位米；时间为图像原始 header 的绝对秒数，固定九位小数。每次有效视觉更新一行，初始化和跳过的图像无轨迹行。输出父目录自动创建；末尾缺少 IMU 覆盖的图像会报告并跳过。同时输出 MCAP 与 TUM 时先提交 MCAP，再提交轨迹，若最后一步失败，已完成的 MCAP 会保留，详见[包说明](openvins_slam/README.md#tum-格式与校验)。

成功的离线处理打印 `PERFORMANCE`：`average_fps` 包含读写解码，`slam_average_fps` 仅统计估计器调用耗时。更多依据见[节点实现](openvins_slam/src/main.cpp)、[参数处理](openvins_slam/src/runtime_options.cpp)及[轨迹写入实现](openvins_slam/src/trajectory_writer.cpp)。

### 校验结果包

当前 Python 环境需已安装 `mcap` 和 `mcap-ros2-support`。

```bash
# 比较原始消息并检查结果包中的位姿、轨迹、里程计及 TF
ros2 run openvins_slam verify_mcap.py \
  /path/to/job_004.mcap \
  /path/to/openvins_job_004_slam_from_bag.mcap
```

通过时打印 `PASS: schemas, original messages, poses, trajectories, odometry, and TF`；依据见[校验脚本](openvins_slam/test/verify_mcap.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `source`（第一个位置参数） | 指定原始包 | 输入文件 | 必填，无默认值；MCAP 文件 |
| `output`（第二个位置参数） | 指定待校验结果包 | 输入文件 | 必填，无默认值；MCAP 文件 |
| `-h`、`--help` | 显示帮助 | 行为控制项 | 默认不启用 |

## 4. 接入 ROS2 实时数据

### 4.1 回放式（MCAP 按采集速度播放）

此流程读取 MCAP，不订阅传感器。`realtime=true` 时发布 `/clock` 并使用仿真时间；计算落后时允许减速，不保证硬实时。

```bash
# 按采集速度处理 MCAP，同时启动 RViz2 并导出轨迹
ros2 launch openvins_slam bag_slam.launch.py \
  cameras:=/path/to/calib.yaml \
  in_bag:=/path/to/job_004.mcap \
  realtime:=true \
  imu_time_offset_sec:=0.025 \
  outdir:=/tmp/openvins_results \
  rviz2:=true
```

到达包末尾后提交轨迹、保留最后结果，按 Ctrl+C 退出。默认同时写包，仅保存轨迹时加 `save_mcap:=false`；只跑节点时设 `rviz2:=false`。RViz2 默认关闭图像显示，需要时在 Displays 面板勾选 `Stereo feature tracks`。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 第 3 节全部节点参数（以 `名称:=值` 传入） | 设置 MCAP 处理选项 | 按第 3 节各项定义 | 默认值及约束同第 3 节，唯 `realtime` 的 launch 默认值为 `true` |
| `rviz2` | 控制 RViz2 启动 | 行为控制项 | 默认 `false`；支持 `true`、`false` |

参数与默认值依据见[launch 文件](openvins_slam/launch/bag_slam.launch.py)。

### 4.2 在线订阅（接真实传感器）

准备 EuRoC 风格的 `estimator_config.yaml` 及其引用的 `kalibr_*_chain.yaml`，参照[仓库示例](config/euroc_mav/estimator_config.yaml)。此配置与 MCAP 流程的 `calib.yaml` 格式不同。传感器需按真实采集速度发布原始图像（`sensor_msgs/msg/Image`）和 IMU。

```bash
# 启动双目在线订阅节点与 RViz2
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/path/to/estimator_config.yaml \
  use_stereo:=true \
  max_cameras:=2 \
  rviz_enable:=true \
  verbosity:=INFO
```

参数依据见[在线 launch 文件](ov_msckf/launch/subscribe.launch.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `config_path` | 指定估计器配置 | 输入文件 | 默认空：由 `config` 选择；显式路径须为存在的文件 |
| `config` | 选择安装包内配置目录 | 输入值 | 默认 `euroc_mav`；支持安装包 `config` 下存在的目录；`config_path` 非空时忽略 |
| `namespace` | 设置节点命名空间 | 输入值 | 默认 `ov_msckf` |
| `ov_enable` | 控制估计器启动 | 行为控制项 | 默认 `true`；支持 `true`、`false` |
| `rviz_enable` | 控制 RViz2 启动 | 行为控制项 | 默认 `false`；支持 `true`、`false` |
| `verbosity` | 设置日志等级 | 行为控制项 | 默认 `INFO`；支持 `ALL`、`DEBUG`、`INFO`、`WARNING`、`ERROR`、`SILENT` |
| `use_stereo` | 开启双目约束 | 行为控制项 | 默认 `true`；支持 `true`、`false` |
| `max_cameras` | 设置相机数量 | 输入值 | 默认 `2`；`1` 为单目、`2` 为双目、`>2` 为多相机单目跟踪 |
| `save_total_state` | 保存完整状态与标准差，非 TUM | 行为控制项 | 默认 `false`；支持 `true`、`false`；默认写入进程工作目录的 `state_estimate.txt`、`state_deviation.txt` |

需要直接覆盖话题时，可改用以下节点调用。默认话题优先取配置的 `rostopic`；回退值为 `/imu0`、`/cam0/image_raw`、`/cam1/image_raw`。如需 TUM，可录制 `/ov_msckf/poseimu` 后自行转换；若录下原始数据再走第 3 节，必须满足第 2 节的压缩拼接图像与标定格式要求。

```bash
# 使用显式话题名启动在线订阅节点
ros2 run ov_msckf run_subscribe_msckf \
  --ros-args \
  -p config_path:=/path/to/estimator_config.yaml \
  -p topic_imu:=/imu \
  -p topic_camera0:=/cam0/image_raw \
  -p topic_camera1:=/cam1/image_raw
```

直接调用需显式提供可用配置。参数与优先级依据见[节点入口](ov_msckf/src/run_subscribe_msckf.cpp)、[订阅及状态输出实现](ov_msckf/src/ros/ROS2Visualizer.cpp)；其余估计器配置见[示例配置](config/euroc_mav/estimator_config.yaml)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--ros-args`、`-p 名称:=值` | 传递节点参数 | 行为控制项 | 语法同第 3 节 |
| `config_path` | 指定估计器配置 | 输入文件 | 此调用须显式设置，无可用默认路径 |
| `topic_imu`、`topic_camera0`、`topic_camera1` | 覆盖配置中的话题 | 输入值 | 默认取配置 `rostopic`，回退值见上文；支持 ROS 2 话题名 |
| `imu0_rostopic`、`cam0_rostopic`、`cam1_rostopic` | 按传感器名覆盖话题 | 输入值 | 默认未设置；设置时优先于对应 `topic_*` 参数 |
| `save_total_state` | 保存完整状态与标准差 | 行为控制项 | 默认 `false`；支持 `true`、`false`；非 TUM |
| `filepath_est`、`filepath_std` | 指定完整状态、标准差文件 | 输出文件 | 默认 `state_estimate.txt`、`state_deviation.txt`；仅保存状态时生效，已有文件会删除重写 |

## 5. 可视化

### 回放已生成的结果包

实时计算直接使用第 4.1 节。回放已有结果包时，在两个已加载环境的终端分别执行：

```bash
# 回放结果包并发布仿真时钟
ros2 bag play \
  /path/to/openvins_job_004_slam_from_bag.mcap \
  --clock
```

包内每条 `/openvins/path_*` 仅录制一条最终完整轨迹，到最后一帧有效位姿才出现；逐帧 Pose、odom 和动态 TF 可显示此前运动过程。[输出约定](openvins_slam/README.md#结果话题)还包括 `/openvins/pose_imu`、`/openvins/pose_cam0`、`/openvins/pose_cam1`、`/openvins/odom`、`/tf` 和 `/tf_static`。`/openvins/odom` 每次有效视觉更新一条，`frame_id=odom`、`child_frame_id=imu`，包含 IMU 系速度，无需 `record_debug`。跟踪图与稀疏点仅在 `record_debug=true` 时写包。离线模式不发布到 DDS，`realtime=true` 时才发布。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 包路径（位置参数） | 指定回放包 | 输入文件 | 此处必填；已生成的结果 MCAP |
| `--clock` | 发布仿真时钟 | 行为控制项 | 默认不启用；此处启用 |
| `--remap` | 重映射话题 | 输入值 | 默认无重映射；例如 `/openvins/odom:=/odom`；节点无 `odom_topic` 参数 |

```bash
# 用项目配置打开 RViz2 并使用仿真时间
rviz2 \
  -d "$(ros2 pkg prefix --share openvins_slam)/launch/bag_slam.rviz" \
  --ros-args \
  -p use_sim_time:=true
```

`rviz2` 参数：

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `-d` | 加载显示配置 | 输入文件 | 未指定时使用 RViz2 自身默认配置；此处为项目 `bag_slam.rviz` |
| `--ros-args`、`-p use_sim_time:=值` | 设置仿真时间参数 | 行为控制项 | `use_sim_time` 默认 `false`；支持 `true`、`false` |

命令替换中的 `ros2 pkg prefix` 参数：

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 包名（位置参数） | 指定已安装的 ROS 2 包 | 输入值 | 必填，无默认值；此处为 `openvins_slam` |
| `--share` | 返回包的共享资源路径 | 行为控制项 | 默认返回安装前缀；此处返回 share 目录 |

### 用 matplotlib 快速查看 TUM

在已安装 NumPy 和 matplotlib 的 Python 环境中运行，先把文件名改为实际 TUM 路径；需至少两行有效数据。

```python
# 绘制 TUM 轨迹的 XY 平面与高度变化
import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt("job_004.txt")
fig, ax = plt.subplots(1, 2, figsize=(9, 3.5))
ax[0].plot(d[:, 1], d[:, 2])
ax[0].set(xlabel="x [m]", ylabel="y [m]", title="XY", aspect="equal")
ax[1].plot(d[:, 0] - d[0, 0], d[:, 3])
ax[1].set(xlabel="t [s]", ylabel="z [m]", title="Z")
plt.tight_layout()
plt.show()
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `np.loadtxt` 的文件路径（代码配置项） | 指定 TUM 轨迹 | 输入文件 | 必须设置；示例 `job_004.txt` 相对于 Python 进程工作目录；八列数值，无表头 |
| `figsize`（代码配置项） | 设置画布尺寸（英寸） | 行为控制项 | 示例显式设为 `(9, 3.5)`，非库默认值 |

<a id="7-限制与排错"></a>

发生错误时按对应步骤中的输入、时间偏移和输出路径要求核对；详细实现与已有验收记录见[包说明](openvins_slam/README.md)和[验收报告](openvins_slam/ACCEPTANCE.md)。
