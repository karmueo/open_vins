# OpenVINS 中文使用说明

<!-- 说明 OpenVINS 的构建、数据处理、在线订阅和结果回放流程。 -->

## 1.1 概述

在 Ubuntu 24.04、ROS 2 Jazzy 下构建 OpenVINS，离线处理 MCAP 或 RAW 并导出 TUM 轨迹，或通过 ROS 2 在线订阅。预先安装工作区构建依赖、`rosbag2_storage_mcap` 和 `rviz2`。从仓库根目录操作，将 `/path/to/...` 换成实际路径。

## 1.2 环境与构建

```bash
# 加载 ROS 2 Jazzy 环境
source \
  /opt/ros/jazzy/setup.bash
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| 脚本路径（位置参数） | 指定环境脚本 | 输入文件 | 必填。使用 Jazzy 或工作区的 `setup.bash` |
| 脚本实参（可选位置参数） | 设置被加载脚本的位置参数 | 输入值 | 省略时沿用当前 Shell 的位置参数 |

```bash
# 构建 OpenVINS 处理节点、在线订阅节点和自定义消息接口
MAKEFLAGS=-j4 colcon build \
  --packages-up-to openvins_slam ego_msgs \
  --parallel-workers 1 \
  --cmake-args \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DBoost_DIR=/usr/lib/x86_64-linux-gnu/cmake/Boost-1.83.0 \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

命令使用 x86_64 系统库路径。OpenCV 应与系统 `cv_bridge` 使用的版本一致。参见 [包构建说明](openvins_slam/README.md#构建)和 [CMake 依赖](openvins_slam/CMakeLists.txt)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `MAKEFLAGS`（环境变量） | 限制单包的 Make 并行任务数 | 行为控制项 | 省略时依赖构建环境，示例为 `-j4` |
| `--packages-up-to` | 选择目标包及其递归依赖 | 输入值 | 省略且无其他包筛选条件时处理发现的全部包。接受空格分隔的包名 |
| `--parallel-workers` | 限制并行执行器同时构建的包数 | 行为控制项 | 默认按可用 CPU 数设置。接受非负整数，`0` 表示不限 |
| `--cmake-args` | 向 CMake 传递后续配置参数 | 输入值 | 省略时无额外参数 |
| `-DOpenCV_DIR`（CMake 配置项） | 指定 OpenCV CMake 配置目录 | 输入目录 | 未设置时由 CMake 查找。示例为系统 OpenCV 4 目录 |
| `-DBoost_DIR`（CMake 配置项） | 指定 Boost CMake 配置目录 | 输入目录 | 未设置时由 CMake 查找。示例为系统 Boost 1.83.0 目录 |
| `-DPython3_EXECUTABLE`（CMake 配置项） | 指定 `FindPython3` 使用的解释器 | 输入文件 | 未设置时由构建配置查找，示例为 `/usr/bin/python3` |
| `-DPYTHON_EXECUTABLE`（CMake 配置项） | 指定旧版 `FindPythonInterp` 使用的解释器 | 输入文件 | 未设置时由构建配置查找，示例为 `/usr/bin/python3` |

```bash
# 加载已构建的工作区
source \
  install/setup.bash
```

每个新终端都依次加载 Jazzy 和工作区环境，`source` 参数见本节首表。

## 1.3 离线计算

为 MCAP 或 RAW 数据准备 `calib.yaml`，其中包含双目 fisheye 内参、`resolution`、`T_cam_imu`、IMU 偏置和噪声。外参平移按毫米输入，陀螺偏置按 deg/s 输入。**换数据必须重新核对 IMU 时间偏移、外参和偏置约定。** 参见 [标定加载实现](openvins_slam/src/calibration.cpp)和[包参数说明](openvins_slam/README.md#参数)。

**显式指定 `outdir` 时，成功提交会原子替换各同名旧结果文件，需保留时先另存。** 保存结果 MCAP 时预留足够空间，MCAP 输入的全部原始消息会随结果写入。

<a id="6-参数速查"></a>

### 1.3.1 配置离线处理参数

编辑 [slam_from_bag.yaml](openvins_slam/config/slam_from_bag.yaml) 的 `slam_from_bag.ros__parameters`。空字符串写成 `""`，布尔值和数字直接填写。随包配置针对 EGO 800 Hz 的 `job_002.mcap`。“默认”指省略键时的值，“继承值”来自 [EuRoC 配置](config/euroc_mav/estimator_config.yaml)。参见 [节点参数](openvins_slam/src/main.cpp)和 [参数校验](openvins_slam/src/runtime_options.cpp)。

```yaml
slam_from_bag:
  ros__parameters:
    save_mcap: true
    rviz2: false
    realtime: false
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `save_mcap` | 保存原始消息与定位结果的 MCAP | 行为控制项 | 默认 `true`，设为 `false` 时仅保存 TUM |
| `rviz2` | 启动 RViz2，处理速度仍由 `realtime` 控制 | 行为控制项 | 默认 `false`，开启后处理结束保留最后画面 |
| `stereo_topic` | 指定拼接双目话题 | 输入值 | 默认 `/stereo/image_raw`，须非空且与 IMU 话题不同 |
| `imu_topic` | 指定 IMU 话题 | 输入值 | 默认 `/imu`，须非空且与双目话题不同 |
| `realtime` | 按采集时间处理 | 行为控制项 | 默认 `false`，尽快计算。设为 `true` 时按采集时间处理并在结束后等待 Ctrl+C |
| `use_sim_time` | 使用 ROS 仿真时间 | 行为控制项 | 默认 `false`。`realtime` 或 `rviz2` 为 `true` 时节点自动启用并发布 `/clock` |
| `half_resolution` | 将每目宽高各减半后跟踪 | 行为控制项 | 默认 `true`，随包设为 `false` |
| `apply_imu_bias` | 预减标定偏置 | 行为控制项 | 默认 `true`，输入已扣偏置时设为 `false` |
| `record_debug` | 将跟踪图和稀疏点写入结果包 | 行为控制项 | 默认 `false`，开启后会增大文件 |
| `record_points` | 将稀疏点写入结果包 | 行为控制项 | 默认 `true`，关闭点云录制时同时将 `record_debug` 设为 `false` |
| `imu_time_offset_sec` | 对计算用 IMU 时间施加固定修正，单位秒 | 输入值 | 默认取 `ROS.ImuTimeOffsetSec`，标定缺失时为 `0.0`。支持 `[-10, 10]` 内有限数。模板中的 `-0.007` 是 `job_002.mcap` 的示例，取消注释后生效 |
| `try_zupt` | 开启零速度更新 | 行为控制项 | 继承值 `false`，随包为 `true` |
| `num_pts` | 运行阶段两目合计目标特征数 | 输入值 | 继承值 `200`，随包为 `400`。整数且至少为 2，每目分配向下取整的一半 |
| `init_max_features` | 初始化阶段两目合计目标特征数 | 输入值 | 继承值 `50`，随包为 `100`。整数且至少为 15，每目分配向下取整的一半 |
| `max_clones` | 滑窗克隆数上限 | 输入值 | 继承值 `11`，随包为 `15`。整数且至少为 2 |
| `max_slam` | 状态内 SLAM 特征上限 | 输入值 | 继承值 `50`，随包为 `100`。非负整数 |
| `max_slam_in_update` | 单批 SLAM 更新特征上限 | 输入值 | 继承值 `25`，随包为 `50`。正整数 |
| `max_msckf_in_update` | 单次 MSCKF 更新特征上限 | 输入值 | 继承值 `40`，随包为 `80`。正整数 |
| `fast_threshold` | FAST 检测阈值 | 输入值 | 继承值与随包均为 `20`。正整数 |
| `grid_x` | 水平特征提取网格数 | 输入值 | 继承值与随包均为 `5`。正整数，最多为跟踪图像宽度 |
| `grid_y` | 垂直特征提取网格数 | 输入值 | 继承值与随包均为 `5`。正整数，最多为跟踪图像高度 |
| `min_px_dist` | 最小特征间距，单位为跟踪图像像素 | 输入值 | 继承值 `10`，随包为 `20.0`。有限正数，向上取整后不得超过跟踪图像短边 |
| `calib_cam_intrinsics` | 在线估计相机内参 | 行为控制项 | 继承值 `true`，随包为 `false` |
| `calib_cam_extrinsics` | 在线估计相机与 IMU 外参 | 行为控制项 | 继承值 `true`，随包为 `false` |
| `calib_cam_timeoffset` | 在线估计相机与 IMU 的剩余时间偏移 | 行为控制项 | 继承值 `true`，随包为 `false` |

整数参数上限为 `INT_MAX`。需要复现固定相机标定时，将三个 `calib_cam_*` 选项都设为 `false`。外参在线估计使用动态 `/tf`，固定外参使用 `/tf_static`。按需从 `FINAL_CALIBRATION` 查看等效 IMU 时间修正和最终相机标定。参见 [在线标定参数说明](openvins_slam/README.md#视觉参数与在线标定)。

### 1.3.2 MCAP 离线计算

输入包含 CDR 编码的 `/stereo/image_raw`（`sensor_msgs/msg/CompressedImage`，横向拼接，左半为 cam0，右半为 cam1）和 `/imu`（`sensor_msgs/msg/Imu`，角速度 rad/s，加速度 m/s²）。

```bash
# 使用源码中的 YAML 处理 MCAP 并导出 TUM 轨迹
ros2 launch openvins_slam slam_from_bag.launch.py \
  config:="$(pwd)/openvins_slam/config/slam_from_bag.yaml" \
  in_bag:=/path/to/job_002.mcap
```

路径支持绝对路径、相对启动目录的路径和 `~` 展开。上例读取当前源码 YAML，省略 `config` 则读取构建安装时的副本。旧参数 `params_file` 改为 `config`。将旧 YAML 中的 `in_bag`、`cameras` 和 `outdir` 移到命令行。参见 [MCAP launch 实现](openvins_slam/launch/slam_from_bag.launch.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `config` | 选择 ROS 2 参数 YAML | 输入文件 | 默认使用包安装目录的 `config/slam_from_bag.yaml` |
| `in_bag` | 指定待处理的 MCAP | 输入文件 | 必填，须为存在的普通文件 |
| `cameras` | 指定双目与 IMU 标定 | 输入文件 | 省略或留空时查找输入包同目录的 `calib.yaml`，缺失时报错 |
| `outdir` | 指定结果目录 | 输出目录 | 省略或留空时创建 `<启动目录>/output/<包名>_<YYYYMMDDHHMMSS>`，使用系统本地时间，同名目录已存在时报错。显式传入时直接使用指定目录 |

输出为 `openvins_<包名>_<配置名>_trajectory.txt`，若 `save_mcap: true` 则同时生成 `openvins_<包名>_<配置名>.mcap`。包名和配置名均去掉最后一个扩展名。TUM 无表头，每次有效视觉更新写一行 `timestamp tx ty tz qx qy qz qw`，对应 `odom → imu`（米），时间取图像 header 的绝对秒数，保留九位小数。IMU 覆盖不足的尾帧会报告并跳过。轨迹提交失败时保留已提交的 MCAP。参见 [TUM 格式与校验](openvins_slam/README.md#tum-格式与校验)。

离线计算成功时查看 `PERFORMANCE`，其中 `average_fps` 包含读写和解码，`slam_average_fps` 只计估计器耗时。启用 RViz2 或按时间回放时，保存结果后按 Ctrl+C 退出。参见 [MCAP 节点实现](openvins_slam/src/main.cpp)。

#### 1.3.2.1 校验结果包

校验环境须安装 Python 包 `mcap` 和 `mcap-ros2-support`。

```bash
# 校验原始消息与结果包中的位姿、轨迹、里程计和 TF
ros2 run openvins_slam verify_mcap.py \
  /path/to/job_004.mcap \
  /path/to/openvins_job_004_slam_from_bag.mcap
```

通过时打印 `PASS: schemas, original messages, poses, trajectories, odometry, and TF`。参见 [校验脚本](openvins_slam/test/verify_mcap.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `source`（第一个位置参数） | 指定原始包 | 输入文件 | 必填，MCAP 文件 |
| `output`（第二个位置参数） | 指定待校验结果包 | 输入文件 | 必填，MCAP 文件 |
| `-h`、`--help` | 显示帮助 | 行为控制项 | 默认关闭 |

### 1.3.3 RAW 数据离线计算

`in_raw` 使用 `ego_data_collector` 的采集目录，其中有 `images/` 和同处于 `aligned/` 或采集目录根下的 `image_timestamps.txt`、`imu.csv`。图片索引须含相对 `images/` 的 `image_file` 和 `cam_time_ns`，首帧的 `ts_ns`、`stc_ns` 可为 `-1` 或 `NA`。IMU 文件须含 `time_ns`、`gyro_*_rad_s` 和 `acc_*_m_s2` 列。参见 [RAW 读取实现](openvins_slam/src/raw_io.cpp)。

```bash
# 直接处理 RAW 采集目录并导出 TUM 轨迹和可选 MCAP
ros2 launch openvins_slam slam_from_raw.launch.py \
  config:="$(pwd)/openvins_slam/config/slam_from_bag.yaml" \
  in_raw:=/path/to/raw_dir \
  cameras:=/path/to/calib.yaml
```

开启缓存前为全部压缩图片预留内存，内存不足时传入 `cache:=false`。两种模式均预读图片索引和 IMU 数据，处理时逐帧解码。旧 `params_file` 改为 `config`，路径项使用启动参数，`save_mcap` 写入所选 YAML。参见 [RAW launch 实现](openvins_slam/launch/slam_from_raw.launch.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `config` | 选择 ROS 2 参数 YAML | 输入文件 | 默认与 MCAP 离线计算相同。优先使用有效的 `slam_from_raw` 参数组，也支持 `slam_from_bag` 参数组 |
| `in_raw` | 指定 RAW 采集目录 | 输入目录 | 必填，目录结构见上文 |
| `cameras` | 指定双目与 IMU 标定 | 输入文件 | 省略或留空时依次查找采集目录和上级目录中的 `calib.yaml`，均缺失时报错 |
| `outdir` | 指定结果目录 | 输出目录 | 省略或留空时创建 `<启动目录>/output/<输入目录名>_<YYYYMMDDHHMMSS>`，同名目录已存在时报错。显式传入时直接使用指定目录 |
| `cache`（launch 参数或 YAML 配置项） | 预读全部 JPEG/PNG 压缩图片 | 行为控制项 | 默认 `true`，命令行覆盖 YAML。`false` 表示处理时逐帧读图 |
| `stereo_topic`（YAML 配置项） | 设置 RAW 转写 MCAP 时的双目消息话题 | 行为控制项 | 默认 `/stereo/image_raw`，须非空且与 IMU 话题不同 |
| `imu_topic`（YAML 配置项） | 设置 RAW 转写 MCAP 时的 IMU 消息话题 | 行为控制项 | 默认 `/imu`，须非空且与双目话题不同 |

其余 YAML 配置见 [1.3.1 节](#131-配置离线处理参数)。输出命名沿用 [1.3.2 节](#132-mcap-离线计算)，以输入目录名作为包名。成功时查看 `PERFORMANCE`，`slam_average_fps` 按双目帧对统计，仅计估计器处理相机和 IMU 的耗时。参见 [RAW 节点实现](openvins_slam/src/slam_from_raw.cpp)。

## 1.4 回放式处理与在线订阅

### 1.4.1 按采集速度处理 MCAP

`0.025` 秒是 EGO `job_004.mcap` 的时间修正示例。更换数据时先核对标定和偏移，显式指定输出目录时先另存需保留的同名结果。

```bash
# 按采集时间处理 MCAP，同时开启 RViz2 并导出轨迹
ros2 launch openvins_slam bag_slam.launch.py \
  cameras:=/path/to/calib.yaml \
  in_bag:=/path/to/job_004.mcap \
  realtime:=true \
  imu_time_offset_sec:=0.025 \
  outdir:=/tmp/openvins_results \
  rviz2:=true
```

计算落后时回放可减速。保存后按 Ctrl+C 退出。在 RViz2 的 Displays 面板勾选 `Stereo feature tracks` 可看跟踪图。输出命名见 [1.3.2 节](#132-mcap-离线计算)，此处配置名为空。参见 [逐项传参 launch 实现](openvins_slam/launch/bag_slam.launch.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `cameras` | 指定双目与 IMU 标定 | 输入文件 | 省略时查找 MCAP 同目录的 `calib.yaml` |
| `in_bag` | 指定待处理的 MCAP | 输入文件 | 必填，MCAP 文件 |
| `outdir` | 指定结果目录 | 输出目录 | 必填，直接使用指定目录 |
| `realtime` | 按采集时间处理并发布 `/clock` | 行为控制项 | 此入口默认为 `true`，支持 `true` 和 `false` |
| `save_mcap` | 保存结果 MCAP | 行为控制项 | 默认 `true` |
| `rviz2` | 启动 RViz2 | 行为控制项 | 默认 `false` |
| `half_resolution` | 每目宽高减半后跟踪 | 行为控制项 | 默认 `true` |
| `apply_imu_bias` | 预减标定偏置 | 行为控制项 | 默认 `true` |
| `record_debug` | 录制跟踪图和稀疏点 | 行为控制项 | 默认 `false` |
| `stereo_topic` | 指定双目输入话题 | 输入值 | 默认 `/stereo/image_raw` |
| `imu_topic` | 指定 IMU 输入话题 | 输入值 | 默认 `/imu` |
| `imu_time_offset_sec` | 设置固定 IMU 时间修正 | 输入值 | 见 1.3.1 节默认值和范围 |
| `try_zupt` | 开启零速度更新 | 行为控制项 | 见 1.3.1 节继承值 |
| `num_pts` | 运行阶段目标特征数 | 输入值 | 见 1.3.1 节继承值及范围 |
| `init_max_features` | 初始化目标特征数 | 输入值 | 见 1.3.1 节继承值及范围 |
| `max_clones` | 滑窗克隆数 | 输入值 | 见 1.3.1 节继承值及范围 |
| `max_slam` | 状态内 SLAM 特征上限 | 输入值 | 见 1.3.1 节继承值及范围 |
| `max_slam_in_update` | 单批 SLAM 更新特征上限 | 输入值 | 见 1.3.1 节继承值及范围 |
| `max_msckf_in_update` | 单次 MSCKF 更新特征上限 | 输入值 | 见 1.3.1 节继承值及范围 |
| `fast_threshold` | FAST 检测阈值 | 输入值 | 见 1.3.1 节继承值及范围 |
| `grid_x` | 水平特征网格数 | 输入值 | 见 1.3.1 节继承值及范围 |
| `grid_y` | 竖直特征网格数 | 输入值 | 见 1.3.1 节继承值及范围 |
| `min_px_dist` | 最小特征间距 | 输入值 | 见 1.3.1 节继承值及范围 |
| `calib_cam_intrinsics` | 在线估计相机内参 | 行为控制项 | 见 1.3.1 节继承值 |
| `calib_cam_extrinsics` | 在线估计相机与 IMU 外参 | 行为控制项 | 见 1.3.1 节继承值 |
| `calib_cam_timeoffset` | 在线估计相机与 IMU 时间偏移 | 行为控制项 | 见 1.3.1 节继承值 |

布尔项支持 `true` 和 `false`。话题约束见 1.3.1 节。设置 `record_points` 或 `use_sim_time` 请走 [YAML 入口](#13-离线计算)。旧 `outbag`、`trajectory_file`、`save_trajectory` 和 `rviz_enable` 已移除，改用 `outdir`、`save_mcap` 和 `rviz2`。

### 1.4.2 在线订阅

准备 EuRoC 格式的 `estimator_config.yaml` 及其引用的 `kalibr_*_chain.yaml`，传感器按真实采集速度发布原始图像（`sensor_msgs/msg/Image`）与 IMU。参见 [EuRoC 配置示例](config/euroc_mav/estimator_config.yaml)。

```bash
# 启动双目惯性在线订阅节点和 RViz2
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/path/to/estimator_config.yaml \
  use_stereo:=true \
  max_cameras:=2 \
  rviz_enable:=true \
  verbosity:=INFO
```

需要 TUM 时，可录制 `/ov_msckf/poseimu` 后自行转换。若录下传感器数据再离线处理，须满足 [MCAP 输入约定](#132-mcap-离线计算)。参见 [在线 launch 实现](ov_msckf/launch/subscribe.launch.py)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `config_path` | 指定估计器配置 | 输入文件 | 默认空，由 `config` 选择。显式路径须为存在的文件 |
| `config` | 选择安装包内的配置目录 | 输入值 | 默认 `euroc_mav`，接受安装包 `config` 下的配置目录名。`config_path` 非空时以该路径为准 |
| `namespace` | 设置节点命名空间 | 输入值 | 默认 `ov_msckf`，接受 ROS 2 命名空间 |
| `ov_enable` | 启动估计器 | 行为控制项 | 默认 `true`，支持 `true` 和 `false` |
| `rviz_enable` | 启动 RViz2 | 行为控制项 | 默认 `false`，支持 `true` 和 `false` |
| `verbosity` | 设置日志等级 | 行为控制项 | 默认 `INFO`，支持 `ALL`、`DEBUG`、`INFO`、`WARNING`、`ERROR`、`SILENT` |
| `use_stereo` | 开启双目约束 | 行为控制项 | 默认 `true`，支持 `true` 和 `false` |
| `max_cameras` | 设置相机数量 | 输入值 | 默认 `2`。`1` 为单目，`2` 为双目，大于 `2` 时每台相机独立跟踪 |
| `save_total_state` | 保存完整状态与标准差 | 行为控制项 | 默认 `false`。开启时在进程工作目录写入 `state_estimate.txt`、`state_deviation.txt`，会替换已有同名文件 |

#### 1.4.2.1 覆盖传感器话题

```bash
# 直启在线订阅节点并覆盖 IMU 和双目图像话题
ros2 run ov_msckf run_subscribe_msckf \
  --ros-args \
  -p config_path:=/path/to/estimator_config.yaml \
  -p topic_imu:=/imu \
  -p topic_camera0:=/cam0/image_raw \
  -p topic_camera1:=/cam1/image_raw
```

估计器可覆盖的配置见 [EuRoC 配置示例](config/euroc_mav/estimator_config.yaml)，话题优先级和状态输出见 [节点入口](ov_msckf/src/run_subscribe_msckf.cpp)和 [ROS 2 订阅实现](ov_msckf/src/ros/ROS2Visualizer.cpp)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `--ros-args` | 开始传入 ROS 专用参数 | 行为控制项 | 直接运行且无 ROS 参数时可省略 |
| `-p`、`--param` | 按 `名称:=值` 设置节点参数 | 输入值 | 默认无覆盖，可重复使用 |
| `--params-file` | 从 ROS 2 参数文件读取节点参数 | 输入文件 | 默认不读取 ROS 2 参数文件 |
| `config_path`（节点参数或第一个位置参数） | 指定估计器配置 | 输入文件 | 须提供有效路径，节点参数优先于位置参数 |
| `topic_imu` | 覆盖 IMU 话题 | 输入值 | 标定链的 `rostopic`，回退为 `/imu0` |
| `topic_camera0` | 覆盖左目图像话题 | 输入值 | 标定链的 `rostopic`，回退为 `/cam0/image_raw` |
| `topic_camera1` | 覆盖右目图像话题 | 输入值 | 标定链的 `rostopic`，回退为 `/cam1/image_raw` |
| `imu0_rostopic` | 优先覆盖 IMU 话题 | 输入值 | 默认未设置，显式设置时优先于 `topic_imu` |
| `cam0_rostopic` | 优先覆盖左目话题 | 输入值 | 默认未设置，显式设置时优先于 `topic_camera0` |
| `cam1_rostopic` | 优先覆盖右目话题 | 输入值 | 默认未设置，显式设置时优先于 `topic_camera1` |
| `save_total_state` | 保存完整状态与标准差 | 行为控制项 | 默认 `false`，支持 `true` 和 `false` |
| `filepath_est` | 指定完整状态文件 | 输出文件 | 默认 `state_estimate.txt`，保存状态时生效，替换已有同名文件 |
| `filepath_std` | 指定标准差文件 | 输出文件 | 默认 `state_deviation.txt`，保存状态时生效，替换已有同名文件 |
| `publish_global_to_imu_tf` | 发布全局到 IMU 的 TF | 行为控制项 | 默认 `true`，支持 `true` 和 `false` |
| `publish_calibration_tf` | 发布传感器标定 TF | 行为控制项 | 默认 `true`，支持 `true` 和 `false` |
| `path_gt` | 加载参考轨迹 | 输入文件 | 默认未设置，支持 ASL 格式轨迹 |

## 1.5 回放结果 MCAP

### 1.5.1 在 RViz2 查看结果

新终端先执行 [1.2 节](#12-环境与构建)的两条 `source` 命令。需要显示稀疏点时，生成结果包前在 YAML 中保留 `record_points: true`，或设置 `record_debug: true`。

```bash
# 回放已生成的 MCAP 并在 RViz2 中查看结果
ros2 launch openvins_slam replay_mcap.launch.py \
  in_bag:=/path/to/result.mcap
```

默认显示 IMU 实时轨迹 `/openvins/path_imu_live`、IMU 位姿、稀疏点云和双目压缩视频。回放结束后保留画面，关闭 RViz2 窗口退出。参见 [回放 launch 实现](openvins_slam/launch/replay_mcap.launch.py)和 [默认 RViz 配置](openvins_slam/launch/replay_mcap.rviz)。

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `in_bag` | 指定结果 MCAP | 输入文件 | 必填，须为存在的文件，支持相对路径和 `~` 展开 |
| `rate` | 设置回放倍速 | 行为控制项 | 默认 `1.0`，须为有限正数 |
| `loop` | 循环回放 | 行为控制项 | 默认 `false`，支持 `true` 和 `false` |
| `start_paused` | 从暂停状态启动 | 行为控制项 | 默认 `false`，支持 `true` 和 `false` |
| `delay_sec` | 为 RViz2 留出订阅时间，单位秒 | 行为控制项 | 默认 `1.5`，须为有限非负数 |
| `stereo_topic` | 指定包内压缩图话题并映射到 `/stereo/image_raw/compressed` | 输入值 | 默认 `/stereo/image_raw`，接受绝对话题名，可带 `/compressed` 后缀 |
| `rviz_config` | 选择 RViz2 配置 | 输入文件 | 默认包安装目录的 `launch/replay_mcap.rviz`，支持相对路径和 `~` 展开 |

### 1.5.2 恢复或暂停播放

等待播放器启动后，在另一个已按 [1.2 节](#12-环境与构建)加载环境的终端调用服务。设置 `start_paused:=true` 时，先恢复播放。

```bash
# 恢复播放
ros2 service call \
  /openvins_replay_player/resume \
  rosbag2_interfaces/srv/Resume \
  '{}'
```

| 支持的参数／配置项 | 参数作用 | 输入/输出 | 默认值/支持的值 |
| --- | --- | --- | --- |
| `service_name`（位置参数） | 选择播放器的服务 | 输入值 | 必填。恢复用 `/openvins_replay_player/resume`，暂停用 `/openvins_replay_player/pause` |
| `service_type`（位置参数） | 指定服务类型 | 输入值 | 必填。恢复用 `rosbag2_interfaces/srv/Resume`，暂停用 `rosbag2_interfaces/srv/Pause` |
| `values`（位置参数） | 指定 YAML 格式的服务请求 | 输入值 | 默认 `{}`，两个服务均使用空请求 |
| `--stdin` | 从标准输入读取 YAML 请求 | 输入值 | 默认关闭，与 `values` 互斥 |
| `-r`、`--rate` | 按指定频率重复调用服务，单位 Hz | 行为控制项 | 默认仅调用一次，启用时须大于 `0` |
| `-h`、`--help` | 显示帮助 | 行为控制项 | 默认关闭 |

```bash
# 暂停播放
ros2 service call \
  /openvins_replay_player/pause \
  rosbag2_interfaces/srv/Pause \
  '{}'
```

暂停调用沿用上表的 `ros2 service call` 参数。
