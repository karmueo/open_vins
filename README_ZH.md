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

**输出 MCAP 和轨迹的同名旧文件会在成功结束时被原子替换，需保留时先另存。** 输出包保留全部原始消息并追加结果，请预留大于输入包的磁盘空间。离线模式必须提供 `outbag`，不能只写轨迹；只需轨迹时仍须指定临时结果包，事后自行删除。

```bash
# 离线处理 MCAP 并导出 IMU 的 TUM 轨迹
ros2 run openvins_slam slam_from_bag \
  --ros-args \
  -p cameras:=/path/to/calib.yaml \
  -p in_bag:=/path/to/job_004.mcap \
  -p outbag:=/path/to/job_004_openvins.mcap \
  -p save_trajectory:=true \
  -p trajectory_file:=/path/to/job_004.txt \
  -p realtime:=false \
  -p imu_time_offset_sec:=0.025
```

可省略 `cameras` 和 `trajectory_file`，按下表自动选取路径；父目录不存在时自动创建。成功的离线处理打印 `PERFORMANCE`，其中 `average_fps` 包含读写解码，`slam_average_fps` 仅统计估计器调用耗时。参数依据：[节点实现](openvins_slam/src/main.cpp)、[轨迹写入实现](openvins_slam/src/trajectory_writer.cpp)。

需要启用零速度更新时，在命令末尾追加 `-p try_zupt:=true`；未传时使用估计器配置值。

<a id="6-参数速查"></a>

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--ros-args`、`-p 名称:=值` | 传递 ROS 2 节点参数，可重复 `-p` | 行为控制项 | 未传时采用节点默认值；本表下列项均通过 `-p` 设置 |
| `cameras` | 指定标定文件 | 输入文件 | 默认空：查找输入 MCAP 同目录的 `calib.yaml`，找不到时报错 |
| `in_bag` | 指定输入包 | 输入文件 | 必填，无有效默认值；支持 MCAP 文件 |
| `outbag` | 指定结果包 | 输出文件 | 默认空；`realtime=false` 时必填；必须为文件路径而非目录 |
| `realtime` | 选择按时间回放或尽快计算 | 行为控制项 | 默认 `false`；支持 `true`、`false` |
| `save_trajectory` | 开启 TUM 导出 | 行为控制项 | 默认 `false`；支持 `true`、`false` |
| `trajectory_file` | 指定 TUM 路径 | 输出文件 | 默认空：在 `outbag` 同目录生成 `<输入包名去扩展名>_openvins_trajectory.txt`；未设 `outbag` 时用输入包目录；不得与输入或输出 MCAP 同路径 |
| `imu_time_offset_sec` | 覆盖计算用 IMU 时间偏移（秒） | 输入值 | 默认标定的 `ROS.ImuTimeOffsetSec`，缺失时 `0.0`；支持有限数值，范围 `[-10, 10]` |
| `try_zupt` | 覆盖估计器的零速度更新开关 | 行为控制项 | 默认估计器配置值（当前 `false`）；支持 `true`、`false` |
| `half_resolution` | 每目宽高减半后跟踪 | 行为控制项 | 默认 `true`；支持 `true`、`false`；如 1920×1080 → 960×540 |
| `apply_imu_bias` | 预减标定偏置 | 行为控制项 | 默认 `true`；支持 `true`、`false`；输入已扣偏置时设 `false` |
| `record_debug` | 将跟踪图与稀疏点写包，会增大体积 | 行为控制项 | 默认 `false`；支持 `true`、`false` |
| `stereo_topic` | 指定压缩拼接双目话题 | 输入值 | 默认 `/stereo/image_raw`；非空话题名，须与 IMU 话题不同 |
| `imu_topic` | 指定 IMU 话题 | 输入值 | 默认 `/imu`；非空话题名，须与双目话题不同 |

TUM 无表头，每行 `timestamp tx ty tz qx qy qz qw`，坐标系为 `odom → imu`，平移单位米。时间为图像原始 header 的绝对秒数，固定九位小数，不应用 IMU 时间偏移。每次有效视觉更新一行；初始化与跳过的图像无对应行。静止起步或低动态运动更易初始化，末尾缺少 IMU 覆盖的图像会报告并跳过。同时写两种输出时先提交 MCAP，再提交轨迹；若最后一步失败，已完成的 MCAP 会保留，详见[包说明](openvins_slam/README.md#导出-tum-轨迹)。

### 校验结果包

当前 Python 环境需已安装 `mcap` 和 `mcap-ros2-support`。

```bash
# 比较原始消息并检查结果包中的位姿、轨迹、里程计及 TF
ros2 run openvins_slam verify_mcap.py \
  /path/to/job_004.mcap \
  /path/to/job_004_openvins.mcap
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
  save_trajectory:=true \
  rviz_enable:=true
```

到达包末尾后提交轨迹、保留最后结果，按 Ctrl+C 退出。需要同时写包时加 `outbag:=/path/to/job_004_openvins.mcap`；只跑节点时设 `rviz_enable:=false`。RViz2 默认关闭图像显示，需要时在 Displays 面板勾选 `Stereo feature tracks`。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 第 3 节全部节点参数（以 `名称:=值` 传入） | 设置 MCAP 处理选项 | 按第 3 节各项定义 | 默认值及约束同第 3 节，唯 `realtime` 的 launch 默认值为 `true` |
| `rviz_enable` | 控制 RViz2 启动 | 行为控制项 | 默认 `true`；支持 `true`、`false` |

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
  /path/to/job_004_openvins.mcap \
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
