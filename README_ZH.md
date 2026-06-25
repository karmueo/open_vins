# OpenVINS ROS2 EuRoC Vicon Room 与 TUM VI 运行指南

本文档记录如何在当前机器上使用已下载并转换好的 EuRoC MAV Vicon Room 数据集，以及如何把本机 TUM VI ROS1 bag 转成 ROS2 bag 后通过 ROS2 跑通 OpenVINS。

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

### 1.1 TUM VI ROS1 bag 转 ROS2 bag

本机已下载的 TUM VI ROS1 bag 位于：

```text
/mnt/data/slam/TUM
```

当前目录内包含下面 5 个序列：

```text
dataset-room2_512_16.bag
dataset-room3_512_16.bag
dataset-room4_512_16.bag
dataset-room5_512_16.bag
dataset-room6_512_16.bag
```

这些 bag 的话题和项目内 `config/tum_vi` 配置匹配：

- `/cam0/image_raw`
- `/cam1/image_raw`
- `/imu0`

bag 中还包含 `/vrpn_client/raw_transform`。OpenVINS 运行只需要上面 3 个图像和 IMU 话题；真值文件已经在 `ov_data/tum_vi/` 中提供，例如 `dataset-room2_512_16.csv` 和 `dataset-room2_512_16.txt`。

当前机器已有独立的 `rosbags` conda 环境，转换工具路径为：

```text
/home/scl/.conda/envs/rosbags/bin/rosbags-convert
```

推荐把转换后的 ROS2 bag 输出到单独目录：

```bash
mkdir -p /mnt/data/slam/TUM_ros2

for bag in /mnt/data/slam/TUM/*.bag; do
  name="$(basename "$bag" .bag)"
  dst="/mnt/data/slam/TUM_ros2/$name"

  if [ -e "$dst" ]; then
    echo "SKIP existing: $dst"
    continue
  fi

  /home/scl/.conda/envs/rosbags/bin/rosbags-convert \
    --src "$bag" \
    --dst "$dst" \
    --dst-typestore ros2_jazzy \
    --include-topic /cam0/image_raw /cam1/image_raw /imu0
done
```

转换完成后检查 ROS2 bag 元信息：

```bash
find /mnt/data/slam/TUM_ros2 -maxdepth 2 -name metadata.yaml -print

source /opt/ros/jazzy/setup.bash
ros2 bag info /mnt/data/slam/TUM_ros2/dataset-room2_512_16
```

如果只想先验证一个序列，可以单独转换：

```bash
mkdir -p /mnt/data/slam/TUM_ros2

/home/scl/.conda/envs/rosbags/bin/rosbags-convert \
  --src /mnt/data/slam/TUM/dataset-room2_512_16.bag \
  --dst /mnt/data/slam/TUM_ros2/dataset-room2_512_16 \
  --dst-typestore ros2_jazzy \
  --include-topic /cam0/image_raw /cam1/image_raw /imu0
```

### 1.2 UZH-FPV ROS1 bag 转 ROS2 bag

本机已下载的 UZH-FPV Drone Racing ROS1 bag 位于：

```text
/mnt/data/slam/uzh_fpv
```

当前目录按传感器配置分成 4 类：

```text
/mnt/data/slam/uzh_fpv/uzhfpv_indoor
/mnt/data/slam/uzh_fpv/uzhfpv_indoor_45
/mnt/data/slam/uzh_fpv/uzhfpv_outdoor
/mnt/data/slam/uzh_fpv/uzhfpv_outdoor_45
```

项目内已经提供对应配置：

```text
config/uzhfpv_indoor
config/uzhfpv_indoor_45
config/uzhfpv_outdoor
config/uzhfpv_outdoor_45
```

下载得到的 `.bag` 文件是 ROS1 bag，文件头为 `#ROSBAG V2.0`，不能直接用 `ros2 bag play` 播放。ROS2 下运行前需要先转换成 ROS2 bag 目录。OpenVINS 运行主要需要下面 3 个输入话题：

- `/snappy_cam/stereo_l`
- `/snappy_cam/stereo_r`
- `/snappy_imu`

bag 中还包含真值话题，例如 `/groundtruth/odometry` 和 `/groundtruth/pose`。如果只做 OpenVINS 在线播放，可以只转换上面 3 个输入话题；如果后续需要对比真值，可以把真值话题也一起保留。

推荐把转换后的 ROS2 bag 输出到单独目录：

```bash
mkdir -p /mnt/data/slam/uzh_fpv_ros2

for bag in /mnt/data/slam/uzh_fpv/*/*.bag; do
  group="$(basename "$(dirname "$bag")")"
  name="$(basename "$bag" .bag)"
  dst="/mnt/data/slam/uzh_fpv_ros2/$group/$name"

  if [ -e "$dst" ]; then
    echo "SKIP existing: $dst"
    continue
  fi

  mkdir -p "$(dirname "$dst")"

  /home/scl/.conda/envs/rosbags/bin/rosbags-convert \
    --src "$bag" \
    --dst "$dst" \
    --dst-typestore ros2_jazzy \
    --include-topic \
      /snappy_cam/stereo_l \
      /snappy_cam/stereo_r \
      /snappy_imu \
      /groundtruth/odometry \
      /groundtruth/pose
done
```

转换完成后检查 ROS2 bag 元信息：

```bash
find /mnt/data/slam/uzh_fpv_ros2 -maxdepth 3 -name metadata.yaml -print

source /opt/ros/jazzy/setup.bash
ros2 bag info /mnt/data/slam/uzh_fpv_ros2/uzhfpv_indoor/indoor_forward_7_snapdragon_with_gt
```

如果只想先验证一个序列，可以单独转换：

```bash
mkdir -p /mnt/data/slam/uzh_fpv_ros2/uzhfpv_indoor

/home/scl/.conda/envs/rosbags/bin/rosbags-convert \
  --src /mnt/data/slam/uzh_fpv/uzhfpv_indoor/indoor_forward_7_snapdragon_with_gt.bag \
  --dst /mnt/data/slam/uzh_fpv_ros2/uzhfpv_indoor/indoor_forward_7_snapdragon_with_gt \
  --dst-typestore ros2_jazzy \
  --include-topic \
    /snappy_cam/stereo_l \
    /snappy_cam/stereo_r \
    /snappy_imu \
    /groundtruth/odometry \
    /groundtruth/pose
```

转换后的 bag 可以这样播放：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play /mnt/data/slam/uzh_fpv_ros2/uzhfpv_indoor/indoor_forward_7_snapdragon_with_gt
```

## 2. 构建环境

当前验证使用 ROS2 Jazzy。由于机器上存在 conda 和 `/usr/local` 下的 OpenCV，构建前建议清理环境变量，避免 CMake 链接到不兼容的库。

### 2.1 推荐：只构建 OpenVINS 运行所需包

如果只是运行本文档中的 EuRoC Vicon Room 数据集，不需要全量构建 `third_party/`。推荐只构建 OpenVINS 运行链路需要的包：

```bash
colcon build --cmake-clean-cache \
  --packages-up-to ov_data ov_eval ov_msckf \
  --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    "-DCMAKE_IGNORE_PREFIX_PATH=/usr/local;/home/ai/anaconda3"
```

这个命令会构建 `ov_data`、`ov_eval`、`ov_msckf` 以及它们需要的 OpenVINS 依赖包，例如 `ov_core`、`ov_init`。它不会编译 `third_party/image_pipeline` 和 `third_party/isaac_ros_image_pipeline`，因此不会要求安装这些第三方处理链的额外依赖。

### 2.1.1 RK3576 板子注意事项

RK3576 板子上的路径和工作站不同：

- 仓库：`/home/sjgd/scl/work/open_vins`
- ROS2 bag：`/home/sjgd/scl/data/vicon_room1_ros2`

板子上需要额外确认 `image_transport` 已安装，否则 `ov_msckf` 会在 CMake 阶段报找不到 `image_transportConfig.cmake`：

```bash
apt-get install -y ros-jazzy-image-transport
```

RK3576 上不要使用默认 colcon 并发。默认包间并发叠加包内 `-j8 -l8`，容易让多个 `cc1plus` 同时占用数 GiB 内存并触发系统内存压力。板子上建议使用低并发：

```bash
export MAKEFLAGS='-j2 -l2'
colcon build --executor sequential --parallel-workers 2 \
  --packages-up-to ov_data ov_eval ov_msckf \
  --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    "-DCMAKE_IGNORE_PREFIX_PATH=/usr/local"
```

如果构建过程中板子异常重启，不需要全量删除工作区。只清理中断或失败的包，例如这次只清理：

```bash
rm -rf build/ov_eval build/ov_init build/ov_msckf \
       install/ov_eval install/ov_init install/ov_msckf
```

已经完整安装的 `ov_core`、`ov_data` 可以保留。

### 2.2 可选：全量构建 third_party

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
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
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

## 3. 一键运行 V1_01_easy

推荐使用一键 launch 同时启动 OpenVINS 和 ROS2 bag 回放。`euroc_vicon_room.launch.py` 默认使用 RK3576 实时 profile，不启动 RViz2，但会启动实时真值轨迹对齐、保存完整状态文件，并发布 IMU 和相机相关 TF。

```bash
# 进入 OpenVINS 工作区根目录。
cd /home/scl/work/slam/open_vins

# 加载系统 ROS2 Jazzy 环境。
source /opt/ros/jazzy/setup.bash

# 加载当前 OpenVINS 工作区安装结果。
source install/setup.bash

# dataset：选择要运行的 EuRoC Vicon Room 序列。
# verbosity：日志等级；默认 WARNING，调试时可临时设为 INFO。
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=WARNING \
  config_path:=config/euroc_mav/estimator_config.yaml

# 如果数据集路径不是默认路径，则显式设置 bag_root。
# bag_root：ROS2 bag 根目录；launch 会自动拼接 dataset 得到具体 bag 路径。
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=WARNING \
  bag_root:=/home/sjgd/scl/data/vicon_room1_ros2 \
  config_path:=config/euroc_mav/estimator_config_rk3576.yaml
```

该 launch 默认使用下面参数：

- `dataset:=V1_01_easy`：要运行的 EuRoC Vicon Room 序列名，可替换为 `V1_02_medium` 或 `V1_03_difficult`。
- `bag_root:=/mnt/data/slam/vicon_room1_ros2`：ROS2 bag 根目录；如果板端数据在 `/home/sjgd/scl/data/vicon_room1_ros2`，需要手动覆盖。
- `config:=euroc_mav`：使用 EuRoC MAV 相机、IMU 和真值配置目录。
- `config_path:=<ov_msckf share>/config/euroc_mav/estimator_config_rk3576.yaml`：默认使用 RK3576 实时 profile。
- `rviz_enable:=false`：默认不启动 RViz2，降低板端和 SSH 无 GUI 环境负载。
- `gt_align_enable:=true`：默认启动实时真值轨迹对齐节点，用于在线观察估计轨迹和真值轨迹的对齐效果。
- `save_total_state:=true`：默认保存完整状态文件，便于后续转换轨迹并做 ATE/RPE 评估。
- `verbosity:=WARNING`：默认只输出 warning 及以上日志；排查时可改为 `INFO`。
- `publish_global_to_imu_tf:=true`：默认发布 `global -> imu` TF，方便 RViz2、TF 工具和下游节点使用。
- `publish_calibration_tf:=true`：默认发布 `imu -> cam` 标定 TF，方便可视化和下游感知链路使用。
- `play_bag:=true`：默认由 launch 自动播放 bag；若要手动播放则设为 `false`。
- `bag_start_delay:=2.0`：OpenVINS 启动后等待 2 秒再播放 bag，降低启动瞬间丢首帧概率。

下面这些调参项默认不由 `euroc_vicon_room.launch.py` 覆盖，而是直接读取 `estimator_config_rk3576.yaml`；只有在命令行显式传入同名参数时才会覆盖 YAML。RK3576 profile 主要通过这些参数降低前端和 EKF 更新负载：

- `downsample_cameras:=true`：相机图像降采样到一半分辨率，显著降低 KLT 跟踪开销。
- `num_pts:=120`：每帧前端跟踪特征数量上限，低于原始 EuRoC 默认值。
- `track_frequency:=15.0`：相机更新目标频率，`/ov_msckf/poseimu` 应接近 14-15 Hz。
- `histogram_method:=NONE`：关闭直方图预处理，减少图像前处理开销。
- `num_opencv_threads:=2`：限制 OpenCV 线程数，避免板端线程过多导致调度抖动。
- `max_slam:=20`：限制状态中的 SLAM 特征数量，降低 EKF 状态维度。
- `max_msckf_in_update:=25`：限制每次 MSCKF 更新使用的特征数量，降低单帧更新时间。
- `calib_cam_extrinsics:=false`：固定 EuRoC 相机外参，不在线估计。
- `calib_cam_intrinsics:=false`：固定 EuRoC 相机内参，不在线估计。
- `calib_cam_timeoffset:=false`：固定相机-IMU 时间偏移，不在线估计。
- `executor_threads:=2`：ROS2 executor 使用 2 个线程，兼顾 IMU 回调和相机 worker。
- `max_camera_queue:=4`：相机处理队列最多保留 4 帧，避免延迟无限积累。
- `drop_stale_camera_frames:=true`：队列积压时丢弃旧相机帧，优先保持实时性。

如果只想启动 OpenVINS，不在 launch 内播放 bag，可以设置 `play_bag:=false`，再手动播放数据：

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

可另开终端订阅位姿 topic，查看实际发布频率：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 里程计发布速度
ros2 topic hz /ov_msckf/odomimu

# 纯位姿发布
ros2 topic hz /ov_msckf/poseimu
```

如果本机有桌面环境，或者已经配置好 SSH X11/Wayland 转发，可以显式打开 RViz2：

```bash
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  config_path:=config/euroc_mav/estimator_config_rk3576.yaml \
  rviz_enable:=true
```

RViz2 打开后可以观察路径、位姿、特征点等可视化内容。如果远程 SSH 没有配置图形转发，保持默认 `rviz_enable:=false`。

### 3.1 在局域网另一台机器上启动 RViz2

可以。推荐让跑 OpenVINS 的机器继续使用默认 `rviz_enable:=false`，只在局域网内有桌面环境的另一台机器上启动 RViz2。RViz2 通过 ROS2/DDS 订阅 `/ov_msckf/pathimu`、`/ov_msckf/pathgt`、`/tf`、`/ov_msckf/trackhist` 和特征点云等话题，不需要在可视化机器上重复播放 bag 或重复启动 OpenVINS。

运行 OpenVINS 的机器：

```bash
cd /home/scl/work/slam/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=WARNING \
  config_path:=config/euroc_mav/estimator_config.yaml
```

局域网内启动 RViz2 的机器也需要安装 ROS2 Jazzy，并且建议构建或同步一份当前 OpenVINS 工作区，这样可以直接使用项目内 `display_ros2.rviz` 配置。`ROS_DOMAIN_ID` 必须和运行 OpenVINS 的机器一致：

```bash
cd /path/to/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

ros2 topic list | grep ov_msckf
rviz2 -d "$(ros2 pkg prefix ov_msckf)/share/ov_msckf/launch/display_ros2.rviz"
```

也可以用项目 launch 只启动 RViz2，不启动 OpenVINS 和真值对齐节点：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  ov_enable:=false \
  rviz_enable:=true \
  gt_align_enable:=false \
  config:=euroc_mav \
  dataset:=V1_01_easy
```

如果 `ros2 topic list` 在可视化机器上看不到 `/ov_msckf/...` 话题，优先检查下面几项：

- 两台机器是否在同一局域网，且可以互相 `ping` 通。
- 两台机器是否使用相同的 `ROS_DOMAIN_ID`。
- Jazzy 环境下 `ROS_AUTOMATIC_DISCOVERY_RANGE` 是否为 `SUBNET`，并确认没有把发现限制在本机。
- 防火墙、Docker 网络、VPN 或路由器是否阻止 DDS 发现和数据通信；跨网段或不支持组播的网络通常需要额外配置 DDS 静态发现或换到同一二层网络。
- RViz2 订阅图像和点云会占用较多带宽；如果局域网带宽不足，可以在 RViz2 左侧 Displays 面板里先关闭 `Image Tracks` 和点云显示，只保留 `TF`、`Path VIO`、`Path GT`。

## 4. 运行启动数据集

本节使用 ROS2 启动 OpenVINS，并在另一个终端播放已经转换好的 ROS2 bag。启动 OpenVINS 前需要先完成第 2 章构建，并在每个终端加载 ROS2 和当前工作区环境：

```bash
cd /home/scl/work/slam/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 4.1 运行 TUM VI ROS2 bag

TUM VI 使用项目内已有 `config/tum_vi` 配置和 `ov_data/tum_vi` 真值文件。推荐先用 `dataset-room2_512_16` 验证转换和运行流程。

终端 1：启动 OpenVINS。

```bash
cd /home/scl/work/slam/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=tum_vi \
  dataset:=dataset-room2_512_16 \
  path_gt:=/home/scl/work/slam/open_vins/ov_data/tum_vi/dataset-room2_512_16.csv \
  rviz_enable:=true
```

终端 2：播放转换后的 ROS2 bag。

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /mnt/data/slam/TUM_ros2/dataset-room2_512_16 \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

如果要运行其他 TUM VI 序列，只替换 `dataset`、`path_gt` 和 bag 路径：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=tum_vi \
  dataset:=dataset-room5_512_16 \
  path_gt:=/home/scl/work/slam/open_vins/ov_data/tum_vi/dataset-room5_512_16.csv \
  rviz_enable:=true
```

```bash
ros2 bag play /mnt/data/slam/TUM_ros2/dataset-room5_512_16 \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

`dataset-room6_512_16` 可以转换和播放，但项目原始脚本中备注该序列需要更低 IMU 初始化阈值。运行 room6 前建议复制一份 `config/tum_vi/estimator_config.yaml`，把 `init_imu_thresh` 从 `0.45` 调低到 `0.25`，再通过 `config_path:=<新配置文件>` 指定；也可以先用 room2-room5 验证流程。

### 4.2 运行 EuRoC ROS2 bag

EuRoC 使用项目内已有 `config/euroc_mav` 配置和 `ov_data/euroc_mav` 真值文件。当前本地 ROS2 bag 根目录为：

```text
/mnt/data/slam/vicon_room1_ros2
```

推荐先用 `V1_01_easy` 验证流程。常规运行不需要指定 `config_path`，`subscribe.launch.py` 会根据 `config:=euroc_mav` 自动使用安装目录下的 `config/euroc_mav/estimator_config.yaml`。

终端 1：启动 OpenVINS。

```bash
cd /home/scl/work/slam/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav \
  dataset:=V1_01_easy \
  path_gt:=/home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.csv \
  rviz_enable:=true
```

终端 2：播放转换后的 ROS2 bag。

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

如果要运行其他 EuRoC Vicon Room 序列，只替换 `dataset`、`path_gt` 和 bag 路径：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav \
  dataset:=V1_02_medium \
  path_gt:=/home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_02_medium.csv \
  rviz_enable:=true
```

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_02_medium \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

也可以使用项目内已有的一键 launch，自动启动 OpenVINS 并播放本地 EuRoC ROS2 bag：

```bash
ros2 launch ov_msckf euroc_vicon_room.launch.py \
  dataset:=V1_01_easy \
  verbosity:=INFO
```

### 4.3 运行 UZH-FPV ROS2 bag

UZH-FPV 使用项目内已有 `config/uzhfpv_indoor`、`config/uzhfpv_indoor_45`、`config/uzhfpv_outdoor` 和 `config/uzhfpv_outdoor_45` 配置。ROS2 bag 根目录为：

```text
/mnt/data/slam/uzh_fpv_ros2
```

运行时 `config` 需要和 bag 所在目录匹配，`dataset` 使用 bag 目录名。推荐先用 `indoor_forward_7_snapdragon_with_gt` 验证流程。

终端 1：启动 OpenVINS。

```bash
cd /home/scl/work/slam/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=uzhfpv_indoor \
  dataset:=indoor_forward_7_snapdragon_with_gt \
  path_gt:=/home/scl/work/slam/open_vins/ov_data/uzhfpv_indoor/indoor_forward_7_snapdragon_with_gt.txt \
  rviz_enable:=true
```

终端 2：播放转换后的 ROS2 bag。

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /mnt/data/slam/uzh_fpv_ros2/uzhfpv_indoor/indoor_forward_7_snapdragon_with_gt \
  --topics /snappy_cam/stereo_l /snappy_cam/stereo_r /snappy_imu
```

如果要运行 outdoor 序列，只替换 `config`、`dataset`、`path_gt` 和 bag 路径：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=uzhfpv_outdoor \
  dataset:=outdoor_forward_3_snapdragon_with_gt \
  path_gt:=/home/scl/work/slam/open_vins/ov_data/uzhfpv_outdoor/outdoor_forward_3_snapdragon_with_gt.txt \
  rviz_enable:=true
```

```bash
ros2 bag play /mnt/data/slam/uzh_fpv_ros2/uzhfpv_outdoor/outdoor_forward_3_snapdragon_with_gt \
  --topics /snappy_cam/stereo_l /snappy_cam/stereo_r /snappy_imu
```

如果只想先看 VIO 是否正常发布结果，可以临时关闭真值对齐节点：

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=uzhfpv_indoor \
  dataset:=indoor_forward_7_snapdragon_with_gt \
  gt_align_enable:=false \
  rviz_enable:=false \
  verbosity:=INFO
```

### 4.4 查看运行状态

运行 TUM VI、EuRoC 或 UZH-FPV 时，都可以另开终端查看 OpenVINS 输出话题：

```bash
cd /home/scl/work/slam/open_vins

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic hz /ov_msckf/odomimu
ros2 topic hz /ov_msckf/poseimu
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

## 6. 和真值定量对比

`ros2 topic echo /ov_msckf/odomimu --once` 只能说明系统正在发布估计结果。要判断效果好不好，建议跑完整序列，保存估计轨迹，然后和 EuRoC 真值计算 ATE/RPE。

### 6.1 保存估计轨迹

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

如果启动时出现 `boost::filesystem::create_directories: Invalid argument`，说明当前运行的 `ov_msckf` 还是旧构建。重新执行第 2 节带 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` 的 `colcon build`，然后重新 `source install/setup.bash`。

### 6.2 使用 ov_eval 计算误差

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
  ./ov_data/euroc_mav/V1_01_easy.txt \
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

### 6.3 其他序列对应真值

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

### 6.4 如何判断结果是否正常

一般看这几项：

- 终端日志持续输出 `[TIME]`、`q_GtoI`、`p_IinG`，没有大量 `ERROR` 或 `FATAL`。
- RViz 或 `/ov_msckf/odomimu` 中轨迹连续，不突然跳变或快速发散。
- `error_singlerun` 中 `rmse_pos` 越小越好；如果达到数米甚至更大，通常说明轨迹效果较差、初始化异常、播放流程不干净或配置/依赖环境有问题。
- V1_01_easy 通常比 V1_03_difficult 更容易，建议先用 V1_01_easy 作为基准。
