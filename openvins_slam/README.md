# OpenVINS 双目惯性 MCAP 处理

本包在 ROS 2 中运行 OpenVINS 双目惯性里程计，支持读取横向拼接双目图像和 IMU 的 MCAP、离线输出带定位结果的单个 MCAP，或按采集速度回放并在 RViz2 中观察。它不包含回环检测或全局地图优化，轨迹会累积漂移；稀疏点是当前跟踪特征，而非持久地图。

## 构建

目标环境为 Ubuntu 24.04 和 ROS 2 Jazzy，需安装 ROS 2 的 `rosbag2_storage_mcap`、`rviz2` 及仓库已有构建依赖。在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j4 colcon build --packages-up-to openvins_slam ego_msgs --parallel-workers 1 \
  --cmake-args -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

这些参数选择与系统 `cv_bridge` 一致的 OpenCV 和 Jazzy 使用的系统 Python，避免误用其他版本。新建终端运行命令前，需先加载 Jazzy 和工作区的 `setup.bash`。

## 使用 YAML 处理 MCAP

编辑 `config/slam_from_bag.yaml`，然后运行：

```bash
ros2 launch openvins_slam slam_from_bag.launch.py \
  config:="$(pwd)/openvins_slam/config/slam_from_bag.yaml" \
  in_bag:=/path/to/job_002.mcap
```

`in_bag` 必填。省略 `config` 时读取安装目录的配置副本；修改源码配置后可显式传入源码路径。`config`、`in_bag`、`cameras`、`outdir` 均支持相对启动工作目录的路径和 `~` 展开。

`cameras` 可选，省略或为空时查找输入包同目录的 `calib.yaml`；找不到则报错。`outdir` 可选，省略或为空时，在启动命令的工作目录下创建 `output/<输入包名>_<YYYYMMDDHHMMSS>`，例如 `output/job_002_20261116143025`。包名去掉最后一个扩展名，时间使用系统本地时间；自动生成的目录已存在时会报错，避免同秒启动覆盖结果。显式传入 `outdir` 时直接使用该目录，不添加时间子目录，不存在时由节点自动创建：

```bash
ros2 launch openvins_slam slam_from_bag.launch.py \
  in_bag:=/path/to/job_002.mcap \
  cameras:=/path/to/calib.yaml \
  outdir:=/tmp/openvins_results
```

YAML 仅保留处理配置，例如：

```yaml
slam_from_bag:
  ros__parameters:
    save_mcap: true
    rviz2: false
    realtime: false
```

旧 launch 参数 `params_file` 已改为 `config`；YAML 中的 `in_bag`、`cameras`、`outdir` 也必须删除，改为 launch 参数。使用旧接口时会报错并提示迁移。

TUM 始终导出；默认 `save_mcap: true`，额外保存包含全部原始消息和定位结果的单个 MCAP；设为 `false` 可仅保存轨迹。输入包和参数 YAML 均取去掉最后一个扩展名的文件名，例如 `job_002.mcap` 与 `slam_from_bag.yaml` 在输出目录中生成：

- `openvins_job_002_slam_from_bag_trajectory.txt`
- `openvins_job_002_slam_from_bag.mcap`（开启保存时）

重命名或复制 YAML 会改变结果中的配置名；符号链接使用传入链接的名称。同名旧结果在成功结束后原子替换。失败或中断会清理未提交的临时输出。两种输出同时启用时，先准备轨迹、提交 MCAP、再提交轨迹；最后一步失败时保留已提交 MCAP，并报告路径。

旧的 `outbag`、`trajectory_file`、`save_trajectory` 参数已移除，请改用 `outdir` 和 `save_mcap`。

## 使用 YAML 处理原始采集目录

也可跳过打包 MCAP，直接读取 `ego_data_collector` 输出的原始目录（包含 `images/` 与 `aligned/image_timestamps.txt`、`aligned/imu.csv`，或直接放在根目录下的同名文件）：

```bash
ros2 launch openvins_slam slam_from_raw.launch.py \
  config:="$(pwd)/openvins_slam/config/slam_from_bag.yaml" \
  in_raw:=/path/to/raw_dir \
  cameras:=/path/to/calib.yaml
```

- 图片路径逐行读取 `image_timestamps.txt` 中的 `image_file`（到 `<in_raw>/images/` 下加载），时间戳严格使用 `cam_time_ns` 列（不从文件名解析时间，允许首帧 `ts_ns`/`stc_ns` 为 `-1` 或 `NA`）。
- IMU 逐行读取 `imu.csv` 的 `time_ns`、`gyro_*_rad_s` 和 `acc_*_m_s2`，与 `cam_time_ns` 在同一纳秒时间轴上按时间顺序配对。
- `cameras` 省略时依次查找 `<in_raw>/calib.yaml` 与 `<in_raw>/../calib.yaml`；`slam_from_raw.launch.py` 同时支持 `slam_from_raw:` 或 `slam_from_bag:` 节点名配置。
- `cache` 默认开启，计算前将全部 JPEG/PNG 压缩字节、图片索引和 IMU 样本存入内存。关闭后继续预加载索引和 IMU，计算时逐帧从磁盘读图。开启缓存需要为整批压缩图片预留内存。
- 在 `config` 选择的 YAML 的 `ros__parameters` 中设置 `save_mcap`。默认开启，设为 `false` 时只输出 TUM 轨迹。缓存可通过启动参数 `cache:=false` 覆盖 YAML。
- RAW 处理成功后，`PERFORMANCE` 中的 `slam_average_fps` 为双目帧数除以 OpenVINS 处理相机与 IMU 输入的累计耗时。该指标排除预加载、读图、解码、写文件、ROS 发布和回放等待。

## RViz2 与处理速度

YAML 设置 `rviz2: true` 会启动一个 RViz2 并显示轨迹、位姿、TF 和稀疏特征。跟踪图默认隐藏，需要时勾选 `Stereo feature tracks`。`record_debug` 仅控制调试数据是否写入 MCAP，不影响实时显示。

`realtime: false` 尽快计算，`true` 按包内时间回放；开启 RViz 不改变此设置。启用显示或按时间回放时，节点发布 `/clock` 并使用仿真时间；处理结束先保存文件，再保持最后结果，按 Ctrl+C 退出。离线且关闭显示时，处理完成直接退出。`PERFORMANCE` 日志中的总时间包括启用的文件输出，不包括 EOF 后保持画面的时间；估计器时间只统计 IMU 和相机输入的计算调用。

## 逐项传参

```bash
ros2 run openvins_slam slam_from_bag --ros-args \
  -p cameras:=/path/to/calib.yaml -p in_bag:=/path/to/job_004.mcap \
  -p outdir:=/tmp/openvins_results -p save_mcap:=true
```

没有参数文件时配置名为空，生成 `openvins_job_004__trajectory.txt` 和 `openvins_job_004_.mcap`。直接运行时也可传 `--params-file /path/to/config.yaml`，配置名取最后一个参数文件名。内部 `config_name` 用于 launch 传递原始配置名，通常无需设置。直接运行节点的 `rviz2=true` 只启用发布和 EOF 保持，需要另行打开 RViz。

直接运行节点及下方 `bag_slam.launch.py` 仍要求显式传入 `outdir`；自动生成时间目录仅适用于 `slam_from_bag.launch.py`。

原有逐项传参 launch 也使用统一接口，未使用 YAML 时配置名为空：

```bash
ros2 launch openvins_slam bag_slam.launch.py \
  cameras:=/path/to/calib.yaml in_bag:=/path/to/job_004.mcap \
  outdir:=/tmp/openvins_results save_mcap:=true rviz2:=true realtime:=false
```

两个 launch 的 `rviz2` 默认均为 `false`。`bag_slam.launch.py` 保留 `realtime=true` 的回放默认值，其余节点默认值相同。

## TUM 格式与校验

轨迹无表头，每行格式为 `timestamp tx ty tz qx qy qz qw`，对应有效视觉更新的 `odom → imu` 位姿，平移单位米。时间为图像原始 header 的绝对时间，保留九位小数，不应用 IMU 时间修正；初始化和跳过的图像没有轨迹行。

同时保存 MCAP 时，位姿、里程计及动态 TF 逐帧录制，三条完整 Path 仅在结束时各写入一条，避免历史轨迹重复写入。实时发布的完整 Path 随视觉更新发布。可使用现有校验工具检查原消息和定位结果：

```bash
ros2 run openvins_slam verify_mcap.py /path/to/job_004.mcap \
  /tmp/openvins_results/openvins_job_004_.mcap
```

工具需要 Python `mcap` 和 `mcap-ros2-support`。回放生成的 MCAP 可直接使用一键启动脚本（同时拉起 `rviz2` 和 `ros2 bag play --clock`）：

```bash
ros2 launch openvins_slam replay_mcap.launch.py \
  in_bag:=/path/to/result.mcap
```

支持可选参数 `rate:=1.0`、`loop:=false`、`start_paused:=false`、`delay_sec:=1.5`、`stereo_topic:=/stereo/image_raw`、`rviz_config:=...`。默认使用 [replay_mcap.rviz](launch/replay_mcap.rviz)，仅显示 IMU 轨迹（由 `/openvins/pose_imu` 实时累加生成 `/openvins/path_imu_live`）、IMU 位姿、稀疏点云与双目压缩视频；也可手动分终端运行 `ros2 bag play <结果.mcap> --clock` 与 `rviz2 -d "$(ros2 pkg prefix --share openvins_slam)/launch/bag_slam.rviz" --ros-args -p use_sim_time:=true`。

`stereo_topic` 指定包内的压缩图输入话题，统一映射为默认 RViz 订阅的 `/stereo/image_raw/compressed`，输入可已带 `/compressed` 后缀。自定义 RViz 配置应订阅此输出话题。

播放器通过 ROS 服务控制。使用 `start_paused:=true` 时，等待播放器启动后，在另一个已加载 ROS 环境的终端恢复或暂停；启动日志也会显示命令：

```bash
# 恢复播放
ros2 service call /openvins_replay_player/resume rosbag2_interfaces/srv/Resume '{}'
# 暂停播放
ros2 service call /openvins_replay_player/pause rosbag2_interfaces/srv/Pause '{}'
```

## 参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `config` | 安装目录的 `config/slam_from_bag.yaml` | `slam_from_bag.launch.py` 使用的 ROS 2 参数文件。 |
| `cameras` | 输入包同目录的 `calib.yaml` | 标定路径，找不到时报错。 |
| `in_bag` | 必填 | 输入 MCAP 文件。 |
| `outdir` | YAML launch：`<启动工作目录>/output/<包名>_<YYYYMMDDHHMMSS>`；逐项调用必填 | TUM 和可选 MCAP 输出目录。 |
| `save_mcap` | `true` | 保存原消息和定位结果；设为 `false` 可仅保存轨迹。 |
| `rviz2` | `false` | launch 开窗；节点启用发布和 EOF 保持。 |
| `realtime` | `false`；逐项 launch 为 `true` | 按包内时间回放。 |
| `stereo_topic` / `imu_topic` | `/stereo/image_raw` / `/imu` | 拼接双目压缩图和 IMU 话题。 |
| `half_resolution` | `true` | 每目宽高减半后跟踪。 |
| `apply_imu_bias` | `true` | 对估计输入预减标定偏置。 |
| `imu_time_offset_sec` | 标定值，缺失为 `0.0` | 计算副本的固定 IMU 时间修正，不修改原消息。 |
| `try_zupt` | 估计器配置值 | 零速度更新。 |
| `record_debug` | `false` | 将跟踪图和稀疏点写入 MCAP。 |
| `record_points` | `true` | 在录制用 YAML 中配置，将稀疏点写入 MCAP。关闭点云录制须同时将 `record_debug` 设为 `false`。 |

输入为 JPEG 压缩拼接双目，左半 cam0、右半 cam1；IMU 角速度 rad/s，加速度 m/s²。标定平移按毫米转米、陀螺偏置按 deg/s 转 rad/s。换包必须核对时间偏移和偏置约定：`job_004.mcap` 曾验证的 `+0.025 s` 不适用于所有数据。与参考轨迹比较前应以整数纳秒配对，并处理坐标系和空间对齐。

## 视觉参数与在线标定

以下参数同时支持 `ros2 run ... --ros-args -p 名称:=值` 和 `bag_slam.launch.py 名称:=值`。未指定时继承安装包中的 EuRoC 估计器配置，launch 不覆盖这些默认值。

| 参数 | 当前配置值 | 说明 |
| --- | ---: | --- |
| `num_pts` | 200 | 初始化后两目合计的目标跟踪数，每目 `floor(num_pts/2)` |
| `init_max_features` | 50 | 初始化时两目合计的目标跟踪数，整数且至少 15 |
| `max_clones` | 11 | 滑窗克隆数 |
| `max_slam` | 50 | 状态内 SLAM 特征上限，允许 0 |
| `max_slam_in_update` | 25 | 单批 SLAM 更新特征上限 |
| `max_msckf_in_update` | 40 | 单次 MSCKF 更新特征上限 |
| `fast_threshold` | 20 | FAST 检测阈值 |
| `grid_x` / `grid_y` | 5 / 5 | 特征提取网格数 |
| `min_px_dist` | 10 | 跟踪分辨率下的最小间距；有限正数，向上取整为核心使用的整数像素 |
| `calib_cam_intrinsics` | true | 在线相机内参标定 |
| `calib_cam_extrinsics` | true | 在线相机与 IMU 外参标定 |
| `calib_cam_timeoffset` | true | 在线相机与 IMU 时间偏移标定 |

除 `min_px_dist` 外数量参数必须为整数：`init_max_features` 至少 15，`num_pts` 和 `max_clones` 至少 2，`max_slam` 至少 0，其余至少 1。网格数不能超过对应跟踪图像尺寸，像素间距不能超过跟踪图像短边。参数在输出文件创建前校验；启动日志打印最终生效值和每目目标跟踪数。增大参数可能增加耗时和内存，目标数不保证都能检测到或被更新接受。

**默认行为变化：** 相机内参、外参、时间偏移不再被入口强制关闭，而是遵循上述配置。IMU 内参和重力敏感项仍关闭；动态初始化中的标定优化仍关闭。需要复现历史固定标定行为时，在原命令后追加：

```bash
-p calib_cam_intrinsics:=false \
-p calib_cam_extrinsics:=false \
-p calib_cam_timeoffset:=false
```

增强视觉配置示例，在原有输入、输出参数后追加；这些是实验值，默认视觉参数保持不变：

```bash
-p num_pts:=400 \
-p init_max_features:=100 \
-p max_clones:=15 \
-p max_slam:=100 \
-p max_msckf_in_update:=80
```

已有 `half_resolution:=false` 可用于全分辨率对照。更高分辨率和更多特征是否提高精度需要在相同数据上评测。

固定时间修正 `s=imu_time_offset_sec` 仍只应用到 IMU 计算副本：`t_imu_corrected=t_imu_raw+s`。在线时间残差 `d` 从零开始，OpenVINS 使用 `t_imu_corrected=t_cam+d`；图像等待 IMU 覆盖此时刻，角速度也在该时刻插值。原包、结果及 TUM 图像时间戳不变。首尾覆盖不足的图像计入跳过统计，内部历史覆盖丢失、非有限残差或传播倒退会报错并清理未提交输出。

结束时 `FINAL_CALIBRATION` 输出固定修正 `s`、残差 `d`、等效原始 IMU 修正 `s-d`，以及两目最终内参、外参。内参对应日志中标明的跟踪分辨率；外参格式为 `q_ItoC(x y z w), p_IinC(x y z)`，平移单位米。日志不会覆盖输入标定文件。

外参在线标定开启时，相机位姿及协方差使用当前估计，`imu → cam*_optical` 随有效位姿写入 `/tf`；关闭时继续使用 `/tf_static`。同一相机边不会同时发布为动态和静态 TF。

## 结果话题

| 话题 | 内容 |
| --- | --- |
| `/openvins/pose_imu`、`/openvins/pose_cam0`、`/openvins/pose_cam1` | IMU 和相机位姿及协方差。 |
| `/openvins/path_imu`、`/openvins/path_cam0`、`/openvins/path_cam1` | 对应的完整轨迹。 |
| `/openvins/odom` | IMU 位姿和 IMU 坐标系下的速度。 |
| `/tf`、`/tf_static` | `odom → imu → cam0_optical / cam1_optical` 坐标树；相机边按外参标定开关使用动态或静态 TF。 |
| `/openvins/tracking_image` | 在线跟踪图，录制时通过 `record_debug: true` 写入 MCAP。 |
| `/openvins/points` | 当前稀疏特征，录制时通过 `record_points: true`（默认）或 `record_debug: true` 写入 MCAP。 |

实时模式的定位结果和完整 Path 仍随有效视觉更新发布。录制包中的 Path 每条话题只有一条最终完整轨迹，因此标准回放时完整轨迹在最后一次有效位姿的时间才出现；逐帧 Pose、odom 和动态 TF 可用于观察整个运动过程。输入数据没有真值轨迹时，无法据此给出绝对位置精度指标。

本次实包和回归检查的结果见[独立验收报告](ACCEPTANCE.md)。
