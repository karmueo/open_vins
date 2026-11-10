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

## 离线处理

```bash
ros2 run openvins_slam slam_from_bag --ros-args \
  -p cameras:=/data/scl_data/slam/stereo_live_bags/EGO/400hz/calib.yaml \
  -p in_bag:=/data/scl_data/slam/stereo_live_bags/EGO/400hz/job_004.mcap \
  -p outbag:=/tmp/job_004_openvins.mcap \
  -p realtime:=false -p imu_time_offset_sec:=0.025
```

示例中的 `+0.025 s` 是对该数据包的经验时间校正；不提供此参数时，节点仍使用标定文件中的时间偏移。该校正不会修改标定文件或原包，具体依据见下文。

`outbag` 为单个 MCAP 文件路径，不是目录；目标父目录不存在时会自动创建，已存在的同名文件会在成功结束时被原子替换。成功后，包中包含输入的全部原始话题和估计得到的位姿、轨迹、里程计及 TF。MCAP 写入期间失败或中断时不会生成表示处理完成的目标文件；同时导出轨迹文本时的提交顺序和例外见下节。请预留足够磁盘空间，包含完整原始数据和轨迹的文件可能大于输入包。位姿、里程计及动态 TF 逐帧录制；三条完整 Path 仅在处理成功结束时各写入一条，消息时间和 MCAP 记录时间均为最后一次有效位姿的图像时间，避免重复录制历史轨迹导致文件按帧数平方增长。

离线输出包成功写入后，`PERFORMANCE` 行报告两种平均帧率。`processed_frames` 是实际送入估计器的双目图像对数（含初始化帧，不含跳过帧，一对计一帧）：

- `total_seconds` 是开始遍历输入到所有启用的输出成功提交的耗时，包含读取、解码、计算、序列化、写包及启用时的轨迹文件写入；`average_fps` 为帧数除以该耗时。它不包含节点、标定和估计器的创建时间。
- `slam_compute_seconds` 只累计向估计器送入 IMU 和相机数据的调用耗时，包含算法初始化，不包含调用外的图像解码和消息序列化；`slam_average_fps` 为帧数除以该耗时。

比较结果时，应使用相同输入、参数、调试录制设置和相近的机器负载。实时模式以及失败或中断的运行不输出 `PERFORMANCE` 行。

## 导出 TUM 轨迹

离线或实时处理时加上 `-p save_trajectory:=true`（launch 使用 `save_trajectory:=true`），即可导出有效 IMU 位姿。默认关闭；开启时若不指定 `trajectory_file`，输出在输入 MCAP 同一目录，文件名的扩展名替换为 `.txt`。例如 `job_004.mcap` 对应 `job_004.txt`。也可使用 `-p trajectory_file:=/tmp/job_004_trajectory.txt` 显式指定路径。输出目录不存在时会自动创建，已存在的同名轨迹文件会在成功结束时被原子替换；轨迹文件不能与输入或输出 MCAP 相同。之前的 `trajectory_csv` 路径参数已移除，请改用 `trajectory_file`。

文件没有表头，每行以空格分隔，格式为 `timestamp tx ty tz qx qy qz qw`。每行对应一次有效视觉状态更新，轨迹坐标系是 `odom → imu`，平移单位为米；初始化或跳过的图像没有轨迹行。时间戳为对应图像原始 header 的绝对时间，单位秒并固定保留九位小数；时间由整数纳秒直接转换，不应用 IMU 时间偏移。

与 TagSLAM 参考 CSV 比较时，应将 TUM 时间按十进制定点解析并还原为整数纳秒，再按参考 CSV 的 `timestamp_ns` 交集配对，而不是按行号配对或用浮点秒做精确匹配。时间对齐不等于空间坐标对齐：参考结果为 `map → rig`，需要另行处理坐标系和空间对齐。

轨迹使用同目录临时文件，处理完成后同步落盘并原子替换同名文件。失败或完成前中断会清理临时轨迹文件；实时模式在输入结束后先提交文件，再保持最后画面。若同时输出 MCAP，先准备轨迹文件、再提交 MCAP、最后提交轨迹文件；如果最后一步失败，已完成的 MCAP 会保留，错误信息会报告其路径。

处理结束后可回放并在另一终端中使用同一 RViz 配置：

```bash
ros2 bag play /tmp/job_004_openvins.mcap --clock
```

```bash
rviz2 -d "$(ros2 pkg prefix --share openvins_slam)/launch/bag_slam.rviz" \
  --ros-args -p use_sim_time:=true
```

也可对照输入包检查输出的原始消息与定位结果。此工具需要当前 Python 环境中安装 `mcap` 和 `mcap-ros2-support`：

```bash
ros2 run openvins_slam verify_mcap.py \
  /data/scl_data/slam/stereo_live_bags/EGO/400hz/job_004.mcap \
  /tmp/job_004_openvins.mcap
```

输入包中的 `/cam_time`、`/imu_time` 使用 `ego_msgs/msg/ImuTime`。工作区附带的最小 `ego_msgs` 接口根据 `job_004.mcap` 内嵌 schema 提供；编译并加载工作区后，ROS 2 可回放这些自定义话题。估计器不依赖这些辅助话题重建时间。

## 回放并显示

```bash
ros2 launch openvins_slam bag_slam.launch.py \
  cameras:=/data/scl_data/slam/stereo_live_bags/EGO/400hz/calib.yaml \
  in_bag:=/data/scl_data/slam/stereo_live_bags/EGO/400hz/job_004.mcap \
  realtime:=true imu_time_offset_sec:=0.025
```

这会启动处理节点和 RViz2，并发布仿真时钟。计算落后时允许回放减速，保留有效图像对；因此不保证硬实时。到达包末尾后保留最后结果，按 Ctrl+C 退出。如需同时写包，增加 `outbag:=/目标目录/结果.mcap`；只运行节点可设置 `rviz_enable:=false`。

RViz2 默认关闭图像显示，以减少渲染开销。需要查看双目跟踪图时，在 Displays 面板中勾选 `Stereo feature tracks`。

## 参数

以下参数可通过 `ros2 run ... --ros-args -p 参数:=值` 或同名 launch 参数传入：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `cameras` | 必填 | 标定文件路径。 |
| `in_bag` | 必填 | 输入 MCAP 文件路径。 |
| `outbag` | 空 | 输出 MCAP 文件路径；离线模式必填，实时模式可选。 |
| `realtime` | `false`（launch 默认 `true`） | `true` 按时间回放；`false` 尽快离线处理。 |
| `stereo_topic` | `/stereo/image_raw` | `sensor_msgs/msg/CompressedImage` 拼接双目话题。 |
| `imu_topic` | `/imu` | `sensor_msgs/msg/Imu` 话题。 |
| `half_resolution` | `true` | 每目由 1920×1080 降为 960×540 后跟踪；不改变原包图像。 |
| `apply_imu_bias` | `true` | 对估计输入预减标定偏置；若数据已扣偏置可设为 `false`。 |
| `imu_time_offset_sec` | 不覆盖 | 显式指定计算用 IMU 时间偏移，单位秒；未提供时使用 `calib.yaml` 的 `ROS.ImuTimeOffsetSec`。 |
| `record_debug` | `false` | 同时将跟踪图和稀疏点写入输出包，可能明显增加文件体积。 |
| `save_trajectory` | `false` | 将有效 IMU 位姿写为 TUM 格式文本。 |
| `trajectory_file` | 空 | 轨迹输出路径；为空时使用输入 MCAP 同目录同名 `.txt`。 |
| `rviz_enable` | `true` | launch 参数：是否启动 RViz2。 |

当前输入约定为一条 JPEG 压缩图像话题，每帧横向拼接两幅图，左半为 cam0、右半为 cam1；IMU 的角速度和加速度分别使用 rad/s 和 m/s²。标定中的平移按毫米转米，陀螺偏置按 deg/s 转 rad/s。标定文件指定的 IMU 时间偏移应用于计算副本；输出包保留原始时间和消息。

采集器源码的子模块在现有导出目录中缺失，无法最终确认采集端是否已经应用固定时间偏移。对 `job_004.mcap`，使用标定文件中的约 **−5.089 ms** 偏移时观测到轨迹严重发散；独立使用全包双目 PnP 与陀螺仪数据进行时间对齐诊断，最低旋转误差对应的偏移约为 **+25～30 ms**；示例采用已经完整运行验证的 +25 ms。该范围是传感器一致性诊断，并非精确真值或带真值的定位精度验证。对其他数据请重新核对采集端时间和偏置约定，不要直接沿用该诊断值。某些末尾图像可能缺少足够 IMU 覆盖，程序会报告并跳过这些帧。

## 结果话题

| 话题 | 内容 |
| --- | --- |
| `/openvins/pose_imu`、`/openvins/pose_cam0`、`/openvins/pose_cam1` | IMU 和相机位姿及协方差。 |
| `/openvins/path_imu`、`/openvins/path_cam0`、`/openvins/path_cam1` | 对应的完整轨迹。 |
| `/openvins/odom` | IMU 位姿和 IMU 坐标系下的速度。 |
| `/tf`、`/tf_static` | `odom → imu → cam0_optical / cam1_optical` 坐标树。 |
| `/openvins/tracking_image`、`/openvins/points` | 在线跟踪图及当前稀疏特征；仅在 `record_debug=true` 时写包。 |

实时模式的定位结果和完整 Path 仍随有效视觉更新发布。录制包中的 Path 每条话题只有一条最终完整轨迹，因此标准回放时完整轨迹在最后一次有效位姿的时间才出现；逐帧 Pose、odom 和动态 TF 可用于观察整个运动过程。输入数据没有真值轨迹时，无法据此给出绝对位置精度指标。

本次实包和回归检查的结果见[独立验收报告](ACCEPTANCE.md)。
