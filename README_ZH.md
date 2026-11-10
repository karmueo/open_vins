# OpenVINS 中文使用说明

本仓库在 [OpenVINS](https://github.com/rpng/open_vins)（MSCKF 视觉惯性估计器）基础上附带 `openvins_slam` 包，提供三条使用路径：

| 用途 | 入口 | 输入 | 输出 |
| --- | --- | --- | --- |
| 离线计算 + 导出 TUM 轨迹 | `openvins_slam/slam_from_bag` | MCAP 文件 | TUM `.txt`、可选结果 MCAP |
| 按采集速度回放观察 | 同上，`realtime:=true` | MCAP 文件 | 实时话题 + RViz2 |
| 接入 ROS2 实时数据 | `ov_msckf/run_subscribe_msckf` | 在线话题 | 位姿/轨迹话题 + RViz2 |

> 注意：`slam_from_bag` **只读取 MCAP 文件**，不订阅实时话题；真正的在线数据流请走第三行。系统不包含回环与全局优化，轨迹会累积漂移。

---

## 1. 环境与构建

目标环境：Ubuntu 24.04 + ROS 2 Jazzy，另需 `rosbag2_storage_mcap`、`rviz2`。

```bash
source /opt/ros/jazzy/setup.bash
cd <仓库根目录>
MAKEFLAGS=-j4 colcon build --packages-up-to openvins_slam ego_msgs --parallel-workers 1 \
  --cmake-args -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`-DOpenCV_DIR` 使编译所用 OpenCV 与系统 `cv_bridge` 一致，避免运行时崩溃。每个新终端都要先 `source` Jazzy 与工作区的 `setup.bash`。

---

## 2. 输入约定

| 项目 | 约定 |
| --- | --- |
| 图像话题 | `/stereo/image_raw`，`sensor_msgs/msg/CompressedImage`，单话题横向拼接，**左半 cam0、右半 cam1** |
| IMU 话题 | `/imu`，`sensor_msgs/msg/Imu`，角速度 rad/s、加速度 m/s² |
| 标定文件 | `calib.yaml`：`cam0`/`cam1` 内参（fisheye）、`T_cam_imu`、`resolution`、IMU 偏置与噪声、`ROS.ImuTimeOffsetSec` |
| 单位换算 | 平移 mm→m，陀螺偏置 deg/s→rad/s（由程序完成） |

时间偏移是最容易出错的一项：`calib.yaml` 的偏移可能不可信。例如 EGO `job_004.mcap` 的标定值约 **−5.089 ms** 会导致严重发散，独立对齐诊断得到 **+25～30 ms**，示例统一使用 `imu_time_offset_sec:=0.025`。**换数据包必须重新核对，不要沿用该值。**

---

## 3. MCAP 离线计算并导出 TUM 轨迹

```bash
ros2 run openvins_slam slam_from_bag --ros-args \
  -p cameras:=/path/to/calib.yaml \
  -p in_bag:=/path/to/job_004.mcap \
  -p outbag:=/path/to/job_004_openvins.mcap \
  -p save_trajectory:=true \
  -p trajectory_file:=/path/to/job_004.txt \
  -p realtime:=false -p imu_time_offset_sec:=0.025
```

关键点：

- `realtime:=false` 时 `outbag` **必填**，且必须是**文件路径**（非目录）；已存在的同名文件会在处理成功后原子替换，父目录不存在时自动创建。输出会保留输入的全部原始话题并追加定位结果。
- `save_trajectory` 默认关闭；开启后若省略 `trajectory_file`，输出到输入 MCAP 同目录、同名 `.txt`（如 `job_004.mcap` → `job_004.txt`）。输出目录不存在时会自动创建，已存在的同名轨迹文件会在成功结束时原子替换。
- 轨迹导出依附于离线流程：`realtime:=false` 必须给出 `outbag`，**无法只写轨迹**。若只要轨迹，可指定一个临时输出包路径，事后删除。
- 输出包包含输入的全部原始消息，因此**大于输入包**（2 GiB 输入约产生 2 GiB 以上输出），请预留磁盘空间。

成功后终端会打印 `PERFORMANCE` 行，包含 `average_fps`（含读写解码的总帧率）与 `slam_average_fps`（纯估计耗时帧率）两种口径。

### TUM 文件格式

无表头，每行空格分隔八列：

```text
timestamp tx ty tz qx qy qz qw
```

- 每次**有效视觉更新**一行，初始化与跳过的图像无对应行。
- 坐标系 `odom → imu`，平移单位米。
- 时间戳为图像原始 header 的绝对时间，秒、固定九位小数；由整数纳秒直接转换，**不应用** IMU 时间偏移。

### 校验结果包

需要 Python 环境装有 `mcap` 与 `mcap-ros2-support`：

```bash
ros2 run openvins_slam verify_mcap.py 输入.mcap 输出.mcap
```

---

## 4. 接入 ROS2 实时数据

### 4.1 回放式（MCAP 按采集速度播放）

适合"没有实时传感器、但要看实时效果"的场景。`realtime:=true` 时节点发布 `/clock` 并使用仿真时间，落后时允许减速，因此**不保证硬实时**。

```bash
ros2 launch openvins_slam bag_slam.launch.py \
  cameras:=/path/to/calib.yaml \
  in_bag:=/path/to/job_004.mcap \
  realtime:=true imu_time_offset_sec:=0.025 \
  save_trajectory:=true rviz_enable:=true
```

一条命令同时启动处理节点与 RViz2。加 `outbag:=/path/to/结果.mcap` 可顺带写包；`rviz_enable:=false` 只跑节点。到达包末尾后保留最后结果，按 Ctrl+C 退出。

### 4.2 在线订阅（接真实传感器）

`slam_from_bag` 不支持在线订阅，实时数据请用 OpenVINS 上游节点 `run_subscribe_msckf`。它直接订阅**原始图像**（`sensor_msgs/msg/Image`）与 IMU，配置使用 EuRoC 风格的 `estimator_config.yaml` + `kalibr_*_chain.yaml`，与 `calib.yaml` 不是同一套格式。

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/path/to/estimator_config.yaml \
  use_stereo:=true max_cameras:=2 \
  rviz_enable:=true verbosity:=INFO
```

保持传感器真实采集速度发布话题即可。若默认话题不匹配，用参数覆盖：

```bash
ros2 run ov_msckf run_subscribe_msckf --ros-args \
  -p config_path:=/path/to/estimator_config.yaml \
  -p topic_imu:=/imu -p topic_camera0:=/cam0/image_raw -p topic_camera1:=/cam1/image_raw
```

话题默认值依次回退到 `/imu0`、`/cam0/image_raw`、`/cam1/image_raw`；也可在 `kalibr_*_chain.yaml` 的 `rostopic` 字段中指定。加 `-p save_total_state:=true`（launch 用 `save_total_state:=true`）会在当前目录写出 `state_estimate.txt`，含标定与特征的完整状态，**非 TUM 格式**。

**需要 TUM 文件时**：在线节点不直接导出 TUM，建议用 `ros2 bag record` 录下话题后走第 3 节，或录 `/ov_msckf/poseimu` 自行转换为 TUM。

---

## 5. 可视化

### 一键启动（推荐）

```bash
ros2 launch openvins_slam bag_slam.launch.py \
  cameras:=/path/to/calib.yaml in_bag:=/path/to/job_004.mcap \
  realtime:=true imu_time_offset_sec:=0.025
```

RViz2 默认关闭图像显示以降低渲染开销；需要看跟踪效果时，在 Displays 面板勾选 `Stereo feature tracks`。

### 回放已生成的结果包

```bash
ros2 bag play /path/to/job_004_openvins.mcap --clock
```

```bash
rviz2 -d "$(ros2 pkg prefix --share openvins_slam)/launch/bag_slam.rviz" \
  --ros-args -p use_sim_time:=true
```

包内每条 `path_*` 话题只存**一条最终完整轨迹**，因此标准回放时完整轨迹要到最后一帧有效位姿才出现；逐帧 `pose`、`odom` 与动态 TF 可用于观察整个运动过程。

### 输出话题

| 话题 | 内容 |
| --- | --- |
| `/openvins/pose_imu`、`pose_cam0`、`pose_cam1` | 位姿与协方差 |
| `/openvins/path_imu`、`path_cam0`、`path_cam1` | 完整轨迹 |
| `/openvins/odom` | IMU 位姿与 IMU 系速度 |
| `/tf`、`/tf_static` | `odom → imu → cam0_optical / cam1_optical` |
| `/openvins/tracking_image`、`/openvins/points` | 跟踪图与稀疏特征（`record_debug:=true` 才写包） |

`/openvins/odom`（`nav_msgs/msg/Odometry`）每条有效视觉更新一条，`frame_id=odom`、`child_frame_id=imu`，无需 `record_debug` 即录制。话题名硬编码，**没有** `odom_topic` 参数；需要标准名时在回放中重映射：`ros2 bag play 结果.mcap --remap /openvins/odom:=/odom`。离线模式只把消息写入包内，不发布到 DDS；`realtime:=true` 时才真正在线发布。

### 用 matplotlib 快速查看 TUM

```python
import numpy as np, matplotlib.pyplot as plt
d = np.loadtxt("job_004.txt")          # timestamp tx ty tz qx qy qz qw
fig, ax = plt.subplots(1, 2, figsize=(9, 3.5))
ax[0].plot(d[:, 1], d[:, 2]); ax[0].set(xlabel="x [m]", ylabel="y [m]", title="XY", aspect="equal")
ax[1].plot(d[:, 0] - d[0, 0], d[:, 3]); ax[1].set(xlabel="t [s]", ylabel="z [m]", title="Z")
plt.tight_layout(); plt.show()
```

---

## 6. 参数速查

`slam_from_bag` / `bag_slam.launch.py`：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `cameras` | 必填 | 标定文件路径 |
| `in_bag` | 必填 | 输入 MCAP 路径 |
| `outbag` | 空 | 输出 MCAP；`realtime:=false` 时必填 |
| `realtime` | `false`（launch 默认 `true`） | 按时间回放 / 尽快离线处理 |
| `save_trajectory` | `false` | 导出 TUM 轨迹 |
| `trajectory_file` | 空 | 轨迹路径；空则用输入包同名 `.txt` |
| `imu_time_offset_sec` | 不覆盖 | 覆盖 `ROS.ImuTimeOffsetSec` |
| `half_resolution` | `true` | 每目降采样一半后跟踪（本数据 1920×1080 → 960×540） |
| `apply_imu_bias` | `true` | 预减标定偏置；数据已扣偏置时设 `false` |
| `record_debug` | `false` | 写跟踪图与稀疏点，明显增大体积 |
| `stereo_topic` / `imu_topic` | `/stereo/image_raw` / `/imu` | 输入话题 |
| `rviz_enable` | `true` | 仅 launch：是否启动 RViz2 |

---

## 7. 限制与排错

- **有漂移**：纯里程计，无回环、无持久地图；稀疏点是当前跟踪特征而非地图。本仓库未提供真值评测，绝对精度指标需另行评测。
- **`outbag` 覆盖行为**：同名输出文件会被原子替换，父目录缺失会自动创建；需保留旧结果时请先另存。
- **初始化慢**：初始化前的图像参与处理但不产生轨迹行；静止起步或低动态运动更易收敛。
- **尾帧被跳过**：末尾图像缺少足够 IMU 覆盖时会报告并跳过。
- **轨迹发散**：优先怀疑 `imu_time_offset_sec`，其次检查标定外参与 bias 约定。

更详细的实现、性能数据与验收记录见 [`openvins_slam/README.md`](openvins_slam/README.md) 与 [`openvins_slam/ACCEPTANCE.md`](openvins_slam/ACCEPTANCE.md)。
