# OpenVINS ROS2 EuRoC Vicon Room 运行指南

本文档记录如何在当前机器上使用已下载并转换好的 EuRoC MAV Vicon Room 数据集，通过 ROS2 跑通 OpenVINS。

## 1. 数据集位置

ROS2 bag 已在本机准备好：

- `/mnt/data/slam/vicon_room1_ros2/V1_01_easy`
- `/mnt/data/slam/vicon_room1_ros2/V1_02_medium`
- `/mnt/data/slam/vicon_room1_ros2/V1_03_difficult`

这些数据集的话题和项目内 `config/euroc_mav` 配置匹配：

- `/cam0/image_raw`
- `/cam1/image_raw`
- `/imu0`

bag 中还包含 `/fcu/motor_speed` 等 OpenVINS 不需要的话题。播放时建议只播放上面 3 个话题，避免缺少自定义消息类型时影响运行。

## 2. 构建环境

当前验证使用 ROS2 Jazzy。由于机器上存在 conda 和 `/usr/local` 下的 OpenCV，构建前建议清理环境变量，避免 CMake 链接到不兼容的库。

### 2.1 清理 shell 环境

在仓库根目录执行下面的环境清理。后续无论是非全量构建还是全量构建，都建议先执行这一段：

```bash
cd /home/scl/work/slam/open_vins

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
```

### 2.2 推荐：只构建 OpenVINS 运行所需包

如果只是运行本文档中的 EuRoC Vicon Room 数据集，不需要全量构建 `third_party/`。推荐只构建 OpenVINS 运行链路需要的包：

```bash
colcon build --cmake-clean-cache \
  --packages-up-to ov_data ov_eval ov_msckf \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    "-DCMAKE_IGNORE_PREFIX_PATH=/usr/local;/home/ai/anaconda3"
```

这个命令会构建 `ov_data`、`ov_eval`、`ov_msckf` 以及它们需要的 OpenVINS 依赖包，例如 `ov_core`、`ov_init`。它不会编译 `third_party/image_pipeline` 和 `third_party/isaac_ros_image_pipeline`，因此不会要求安装这些第三方处理链的额外依赖。

### 2.3 可选：全量构建 third_party

只有在需要同时从源码构建 `third_party/image_pipeline` 或 `third_party/isaac_ros_image_pipeline` 时，才使用全量构建。全量构建会扫描仓库内所有 ROS2 包，因此需要额外安装第三方依赖。

先初始化并使用 `rosdep` 安装依赖：

```bash
sudo apt update
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro jazzy -y -r
```

如果 `rosdep` 没有覆盖到本机缺失项，至少需要安装 `image_pipeline` 常见依赖。你遇到的 `camera_info_managerConfig.cmake` 缺失对应下面第一个包：

```bash
sudo apt install \
  ros-jazzy-camera-info-manager \
  ros-jazzy-camera-calibration-parsers \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-geometry \
  ros-jazzy-image-transport \
  ros-jazzy-message-filters \
  ros-jazzy-rclcpp-components \
  ros-jazzy-stereo-msgs \
  ros-jazzy-tf2-eigen \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-ros \
  python3-opencv \
  python3-pytest \
  python3-requests \
  python3-semver
```

如果还要全量构建 Isaac ROS 相关源码包，需要先按 NVIDIA Isaac ROS 4.4 / ROS2 Jazzy 文档安装 VPI、NITROS、GXF、CUDA/CV-CUDA 等依赖。当前仓库内 `third_party/isaac_ros_image_pipeline` 会和 `/opt/ros/jazzy` 下部分 Isaac ROS 包重名；如果确认要覆盖这些 underlay 包，构建时需要显式允许覆盖：

```bash
colcon build --cmake-clean-cache \
  --allow-overriding gxf_isaac_tensorops isaac_ros_vpi_utils \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    "-DCMAKE_IGNORE_PREFIX_PATH=/usr/local;/home/ai/anaconda3"
```

如果只是 OpenVINS EuRoC 运行，不建议使用上面的全量构建命令，使用第 2.2 节即可。

构建完成后 source 当前工作区：

```bash
source install/setup.bash
```

构建完成后确认 `ov_msckf` 可执行文件存在：

```bash
ros2 pkg executables ov_msckf
```

应能看到：

```text
ov_msckf run_subscribe_msckf
ov_msckf run_simulation
ov_msckf test_sim_meas
ov_msckf test_sim_repeat
```

## 3. SSH 无头远程运行 V1_01_easy

OpenVINS 算法本身不需要 GUI 桌面。通过 SSH 远程连接服务器时，保持 `rviz_enable:=false` 即可在纯终端环境运行。

打开两个终端。

SSH 终端 1：启动 OpenVINS。

```bash
cd /home/scl/work/slam/open_vins

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav \
  verbosity:=INFO \
  rviz_enable:=false
```

SSH 终端 2：播放 ROS2 bag。

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

运行正常时，终端 1 会持续输出类似下面的信息：

```text
[TIME]: ...
q_GtoI = ...
p_IinG = ...
```

SSH 终端 3：检查里程计输出。

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/open_vins/install/setup.bash

ros2 topic echo /ov_msckf/odomimu --once
```

能打印 `nav_msgs/msg/Odometry` 即表示 OpenVINS 已经收到相机和 IMU 数据并发布估计结果。

也可以查看话题列表确认节点正在发布：

```bash
ros2 topic list | grep /ov_msckf
```

## 4. 有桌面环境的 RViz 可视化运行

如果本机有桌面环境，或者已经配置好 SSH X11/Wayland 转发，可以启动 RViz 可视化。推荐先按第 3 节确认无头模式能跑通，再打开 RViz。

推荐使用一键 launch 同时启动 OpenVINS、RViz、真值轨迹对齐和 ROS2 bag 回放。

```bash
cd /home/scl/work/slam/open_vins

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=INFO
```

该 launch 默认使用下面参数：

- `bag_root:=/mnt/data/slam/vicon_room1_ros2`
- `config:=euroc_mav`
- `rviz_enable:=true`
- `play_bag:=true`
- `bag_start_delay:=2.0`

如果只想启动 OpenVINS 和 RViz，不在 launch 内播放 bag，可以设置 `play_bag:=false`，再手动播放数据：

```bash
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=INFO \
  play_bag:=false
```

另一个终端播放数据：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

RViz 打开后可以观察路径、位姿、特征点等可视化内容。如果远程 SSH 没有配置图形转发，不要使用 `rviz_enable:=true`，否则 RViz 会因为没有显示环境而启动失败。此时可运行：

```bash
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=INFO \
  rviz_enable:=false
```

## 5. 运行其他序列

使用一键 launch 时只需要替换 `dataset` 参数：

```bash
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_02_medium \
  verbosity:=INFO
```

```bash
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_03_difficult \
  verbosity:=INFO
```

## 6. 快速冒烟测试

限制播放 20 秒：

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0 \
  --playback-duration 20
```

## 7. 和真值定量对比

`ros2 topic echo /ov_msckf/odomimu --once` 只能说明系统正在发布估计结果。要判断效果好不好，建议跑完整序列，保存估计轨迹，然后和 EuRoC 真值计算 ATE/RPE。

### 7.1 保存估计轨迹

终端 1：启动 OpenVINS，并打开 `save_total_state`。

```bash
cd /home/scl/work/slam/open_vins

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav \
  verbosity:=INFO \
  rviz_enable:=false \
  save_total_state:=true
```

终端 2：播放完整数据集。

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

播放结束后，OpenVINS 默认会保存估计轨迹到：

```text
/tmp/ov_estimate.txt
```

同时会保存状态标准差到：

```text
/tmp/ov_estimate_std.txt
```

确认文件存在：

```bash
ls -lh /tmp/ov_estimate.txt /tmp/ov_estimate_std.txt
```

如果启动时出现 `boost::filesystem::create_directories: Invalid argument`，说明当前运行的 `ov_msckf` 还是旧构建。重新执行第 2 节的 `colcon build`，然后重新 `source install/setup.bash`。

### 7.2 使用 ov_eval 计算误差

V1_01_easy 的真值文件在：

```text
/home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.txt
```

注意：`save_total_state` 写出的 `/tmp/ov_estimate.txt` 前 8 列格式是 `timestamp qx qy qz qw px py pz`，而 `ov_eval error_singlerun` 读取的是 `timestamp px py pz qx qy qz qw`。评估前需要先转换列顺序：

```bash
awk 'BEGIN{OFS=" "} /^#/ {next} NF>=8 {print $1,$6,$7,$8,$2,$3,$4,$5}' \
  /tmp/ov_estimate.txt > /tmp/ov_estimate_eval.txt
```

然后运行单序列评估：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/open_vins/install/setup.bash

ros2 run ov_eval error_singlerun \
  posyaw \
  /home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.txt \
  /tmp/ov_estimate_eval.txt
```

重点查看输出中的两部分：

```text
Absolute Trajectory Error
rmse_pos = ...
mean_pos = ...
max_pos  = ...

Relative Pose Error
seg 8  - median_pos = ...
seg 16 - median_pos = ...
```

含义：

- `rmse_pos`：绝对轨迹误差的位置 RMSE，越小越好。
- `mean_pos`：平均位置误差，越小越好。
- `max_pos`：最大位置误差，用来观察是否有局部跑飞。
- `Relative Pose Error`：固定距离片段上的相对位姿误差，用来观察局部漂移。

### 7.3 其他序列对应真值

如果评估其他两个 Vicon Room 序列，只替换 bag 路径和真值文件：

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_02_medium \
  --topics /cam0/image_raw /cam1/image_raw /imu0

ros2 run ov_eval error_singlerun \
  posyaw \
  /home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_02_medium.txt \
  /tmp/ov_estimate_eval.txt
```

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_03_difficult \
  --topics /cam0/image_raw /cam1/image_raw /imu0

ros2 run ov_eval error_singlerun \
  posyaw \
  /home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_03_difficult.txt \
  /tmp/ov_estimate_eval.txt
```

每次切换序列或重新从头播放同一序列前，建议先重启 OpenVINS，保证一次 bag 实验对应一次干净的估计器进程。

### 7.4 如何判断结果是否正常

一般看这几项：

- 终端日志持续输出 `[TIME]`、`q_GtoI`、`p_IinG`，没有大量 `ERROR` 或 `FATAL`。
- RViz 或 `/ov_msckf/odomimu` 中轨迹连续，不突然跳变或快速发散。
- `error_singlerun` 中 `rmse_pos` 越小越好；如果达到数米甚至更大，通常说明轨迹效果较差、初始化异常、播放流程不干净或配置/依赖环境有问题。
- V1_01_easy 通常比 V1_03_difficult 更容易，建议先用 V1_01_easy 作为基准。

## 8. 常见问题

### 找不到 `ov_msckf`

确认已经 source 当前工作区：

```bash
source /opt/ros/jazzy/setup.bash
source /home/scl/work/slam/open_vins/install/setup.bash
```

然后检查：

```bash
ros2 pkg executables ov_msckf
```

### 构建时出现 conda、libcurl、OpenCV 冲突

使用本文第 2 节的干净环境构建命令。重点是：

- `unset LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH`
- `unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH`
- `"-DCMAKE_IGNORE_PREFIX_PATH=/usr/local;/home/ai/anaconda3"`

### 播放 bag 时遇到自定义消息类型

不要播放全部话题，只播放 OpenVINS 需要的 3 个话题：

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

### SSH 下是否需要 GUI

不需要。使用第 3 节的无头运行方式即可。只有使用 RViz 可视化时才需要桌面环境或图形转发。

### SSH 下想看 RViz

需要先确保远程图形环境可用，例如 X11 转发、远程桌面或本机直接登录桌面。否则使用 `rviz_enable:=false`。
