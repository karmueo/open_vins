# ROS 2 EuRoC 双目 stereo_image_proc 使用说明

本文档说明如何使用本仓库新增的 ROS 2 处理链，对 EuRoC MAV Vicon Room 1 双目 bag 生成矫正图像和视差图。

## 功能概览

EuRoC 原始 bag 只包含：

- `/cam0/image_raw`
- `/cam1/image_raw`
- `/imu0`

其中不包含 `sensor_msgs/msg/CameraInfo`。本仓库新增的 `run_stereo_camera_info` 节点会从 OpenVINS 配置读取双目标定，并在收到左右图像时发布：

- `/cam0/camera_info`
- `/cam1/camera_info`

新增 launch 文件 `ov_msckf/launch/euroc_stereo_image_proc.launch.py` 会同时启动：

- `ov_msckf/run_stereo_camera_info`
- `image_proc/rectify_node`，左目
- `image_proc/rectify_node`，右目
- `stereo_image_proc/disparity_node`

输出主题包括：

- `/stereo/left/image_rect`
- `/stereo/right/image_rect`
- `/stereo/disparity`

## 前置条件

使用 ROS 2 Jazzy 环境：

```bash
source /opt/ros/jazzy/setup.bash
```

本功能依赖 `third_party/image_pipeline` 子模块，首次拉取仓库或切换分支后请确认子模块存在：

```bash
git submodule update --init --recursive
```

如果当前 shell 启用了 Conda，建议在构建时显式指定系统 Python。ROS 2 Jazzy 的 `ament_cmake` 需要系统 Python 中的 ROS Python 依赖，例如 `catkin_pkg`。

## 构建

请先进入 OpenVINS 仓库根目录。不要在 `~` 或其他上级目录直接运行 `colcon build`，否则 colcon 会递归扫描无关工程，例如 `~/work/UMI/lerobot`，并可能在包识别阶段报错。

推荐构建命令：

```bash
cd /home/scl/work/slam/open_vins
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ov_msckf stereo_image_proc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

说明：

- `--packages-up-to ov_msckf stereo_image_proc` 会构建 OpenVINS 相关包和 `image_pipeline` 中需要的运行时节点。
- `-DBUILD_TESTING=OFF` 用于跳过第三方包测试目标，避免本地缺少测试依赖时影响运行时节点构建。
- 如果你的环境没有 Conda 干扰，并且 ROS Python 依赖完整，可以去掉两个 Python 参数。

如果命令行提示符中有 `(base)`，说明当前启用了 Conda。若遇到 GTK、GLib、OpenCV 或 Python 库冲突，建议先退出 Conda 再构建：

```bash
conda deactivate
cd /home/scl/work/slam/open_vins
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ov_msckf stereo_image_proc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

构建完成后加载工作空间：

```bash
source install/setup.bash
```

## 启动处理链

终端 1：

```bash
source install/setup.bash
ros2 launch ov_msckf euroc_stereo_image_proc.launch.py config:=euroc_mav
```

默认参数：

- `config:=euroc_mav`
- `approximate_sync:=false`
- `stereo_algorithm:=1`
- `disparity_range:=128`

EuRoC 左右目时间戳同步，因此默认使用精确同步。`stereo_algorithm:=1` 表示使用 SGBM。

如需显式指定配置文件：

```bash
ros2 launch ov_msckf euroc_stereo_image_proc.launch.py \
  config_path:=/home/scl/work/slam/open_vins/config/euroc_mav/estimator_config.yaml
```

## 播放 EuRoC ROS 2 bag

终端 2：

```bash
source install/setup.bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --topics /cam0/image_raw /cam1/image_raw /imu0 \
  --loop
```

建议加 `--loop`，这样 RViz2 或 `ros2 topic echo/hz` 后启动时仍能持续收到新消息。

也可以替换为：

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_02_medium \
  --topics /cam0/image_raw /cam1/image_raw /imu0 \
  --loop
```

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_03_difficult \
  --topics /cam0/image_raw /cam1/image_raw /imu0 \
  --loop
```

## 验证输出

终端 3：

```bash
source install/setup.bash
ros2 topic list | grep -E '/cam[01]/camera_info|/stereo/.+image_rect|/stereo/disparity'
```

期望能看到：

```text
/cam0/camera_info
/cam1/camera_info
/stereo/left/image_rect
/stereo/right/image_rect
/stereo/disparity
```

检查发布频率：

```bash
ros2 topic hz /cam0/camera_info
ros2 topic hz /stereo/left/image_rect
ros2 topic hz /stereo/disparity
```

检查矫正图像 header：

```bash
ros2 topic echo --once /stereo/left/image_rect --field header
ros2 topic echo --once /stereo/right/image_rect --field header
```

注意，`header` 是消息字段，不是话题名。不要写成：

```bash
ros2 topic echo --once /stereo/left/image_rect/header
```

检查视差消息中的焦距：

```bash
ros2 topic echo --once /stereo/disparity --field f
```

正常情况下会输出正数，例如在 `V1_01_easy` 上可得到类似：

```text
2567.04296875
---
```

## RViz2 可视化

启动处理链和 bag 后，可以在新终端打开 RViz2：

```bash
cd /home/scl/work/slam/open_vins
source install/setup.bash
rviz2
```

### 查看左右矫正图像

在 RViz2 中：

1. 点击 `Add`。
2. 选择 `Image`。
3. Topic 选择 `/stereo/left/image_rect` 或 `/stereo/right/image_rect`。

如果显示 `NO Image`，先确认命令行能收到图像：

```bash
ros2 topic echo --once /stereo/left/image_rect --field header
```

如果命令行能收到但 RViz2 不显示，检查 `Image` 显示项的 QoS 设置。当前 `/stereo/left/image_rect` 由 `image_proc/rectify_node` 发布，通常是 `RELIABLE`。可在 RViz2 的 Image 显示项里将 `Reliability Policy` 设为 `Reliable`，`Transport Hint` 保持 `raw`。

### 查看视差图

`/stereo/disparity` 的类型是 `stereo_msgs/msg/DisparityImage`，不是普通 `sensor_msgs/msg/Image`。RViz2 默认 `Image` 插件不能直接显示该话题。

如需查看视差图，推荐构建并使用 `image_view/disparity_view`：

```bash
cd /home/scl/work/slam/open_vins
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-camera-calibration-parsers
colcon build --packages-up-to image_view \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
ros2 run image_view disparity_view --ros-args -r image:=/stereo/disparity
```

如果构建 `image_view` 时报错：

```text
fatal error: camera_calibration_parsers/parse.hpp: No such file or directory
```

说明缺少 `image_view` 的依赖包，请先安装：

```bash
sudo apt install ros-jazzy-camera-calibration-parsers
```

如果安装后仍然报同样错误，说明 `image_view` 的旧 CMake 缓存里还记录着依赖未找到。清掉该包的构建缓存后重建：

```bash
cd /home/scl/work/slam/open_vins
rm -rf build/image_view install/image_view
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to image_view \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 查看点云

EuRoC 是灰度双目数据，不是真 RGBD 相机数据。若希望在 RViz2 中看三维结果，可以用 `stereo_image_proc/point_cloud_node` 将视差转成 `sensor_msgs/msg/PointCloud2`：

```bash
cd /home/scl/work/slam/open_vins
source install/setup.bash
ros2 run stereo_image_proc point_cloud_node --ros-args \
  -p approximate_sync:=true \
  -p approximate_sync_tolerance_seconds:=0.05 \
  -p use_color:=false \
  -r left/image_rect_color:=/stereo/left/image_rect \
  -r /stereo/left/camera_info:=/cam0/camera_info \
  -r right/camera_info:=/cam1/camera_info \
  -r disparity:=/stereo/disparity \
  -r points2:=/stereo/points2
```

然后在 RViz2 中：

1. 将 `Global Options` 的 `Fixed Frame` 设置为 `cam0`。
2. 点击 `Add`。
3. 选择 `PointCloud2`。
4. Topic 选择 `/stereo/points2`。
5. 如果点比较小，可以将 `Style` 设置为 `Points`，并将 `Size (Pixels)` 调到 `2` 或 `3`。

验证点云是否持续发布：

```bash
ros2 topic echo --once /stereo/points2 --field header
ros2 topic hz /stereo/points2
```

注意，`point_cloud_node` 是懒发布逻辑：只有 `/stereo/points2` 存在订阅者时，它才会订阅输入并生成点云。因此启动 `ros2 topic hz /stereo/points2` 或 RViz2 的 `PointCloud2` 显示后，点云才会开始发布。

RViz2 中看到点云“固定不动”是正常现象。当前点云的 `frame_id` 是 `cam0`，如果 RViz2 的 `Fixed Frame` 也设为 `cam0`，显示的是相机坐标系下的当前帧点云，不是全局地图。该链路不会自动累计点云，也不会把点云变换到 `map` 或 `odom`。

## 单独测试 CameraInfo 生成

构建后可以运行新增测试程序：

```bash
source /opt/ros/jazzy/setup.bash
./build/ov_msckf/test_stereo_camera_info config/euroc_mav/estimator_config.yaml
```

该测试会验证：

- 左右相机分辨率为 `752x480`
- `distortion_model` 为 `plumb_bob`
- `D/K/R/P` 字段被正确填充
- 右目投影矩阵中包含有效 baseline 项

## 常见问题

### rosdep 未初始化

如果运行：

```bash
rosdep install --from-paths . --ignore-src -r -y
```

出现：

```text
ERROR: your rosdep installation has not been initialized yet
```

说明当前机器没有初始化 rosdep。可以先由系统管理员或有 sudo 权限的用户执行：

```bash
sudo rosdep init
rosdep update
```

如果只是运行本功能，并且依赖已经安装完成，可以跳过该步骤，直接使用上面的 `colcon build` 命令。

### Conda Python 缺少 catkin_pkg

如果构建时报错：

```text
ModuleNotFoundError: No module named 'catkin_pkg'
```

并且日志中显示正在使用 Conda Python，例如：

```text
/home/ai/anaconda3/bin/python3
```

请在构建时显式指定系统 Python：

```bash
cd /home/scl/work/slam/open_vins
colcon build --packages-up-to ov_msckf stereo_image_proc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 在 home 目录运行导致扫描无关工程

如果在 `~` 下运行 `colcon build`，可能出现类似：

```text
Exception in package identification extension 'python_setup_py' in 'work/UMI/lerobot'
```

这不是 OpenVINS 的错误，而是 colcon 扫描到了 home 目录下的其他 Python 工程。请进入本仓库根目录后重新构建：

```bash
cd /home/scl/work/slam/open_vins
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ov_msckf stereo_image_proc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

如果误在 `~` 下构建过，当前工作区的 `install/setup.bash` 也可能记录旧的 underlay 路径，source 时出现类似：

```text
not found: "/home/scl/install/ov_msckf/share/ov_msckf/local_setup.bash"
```

这表示 `install/setup.bash` 里链式 source 了旧工作区 `/home/scl/install`。用干净环境重新生成当前工作区 setup 脚本：

```bash
cd /home/scl/work/slam/open_vins
env -u COLCON_PREFIX_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH bash -lc '
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-up-to ov_msckf stereo_image_proc image_view \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
'
```

验证不再引用旧路径：

```bash
grep -R "/home/scl/install" install/setup.* || true
source /home/scl/work/slam/open_vins/install/setup.bash
ros2 pkg prefix ov_msckf
ros2 pkg prefix image_view
```

### Conda 库路径导致 GTK/GLib 链接错误

如果日志中出现：

```text
undefined reference to `g_once_init_enter_pointer'
undefined reference to `g_once_init_leave_pointer'
```

通常是 Conda 的库路径干扰了系统 GTK/GLib。请先退出 Conda，确认提示符前没有 `(base)`，再重新构建：

```bash
conda deactivate
cd /home/scl/work/slam/open_vins
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ov_msckf stereo_image_proc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 长时间没有 /stereo/disparity 输出

优先检查以下项：

1. `/cam0/camera_info` 和 `/cam1/camera_info` 是否随图像发布。
2. `/stereo/left/image_rect` 和 `/stereo/right/image_rect` 是否有输出。
3. bag 是否正在发布 `/cam0/image_raw` 和 `/cam1/image_raw`。
4. launch 日志中是否有 `uncalibrated camera`、同步相关警告，或 QoS 不兼容警告。

如果看到类似：

```text
requesting incompatible QoS
Last incompatible policy: RELIABILITY_QOS_POLICY
```

说明发布端和订阅端 QoS 不兼容。当前 `run_stereo_camera_info` 已使用 `RELIABLE` 发布 CameraInfo，以匹配 `image_proc/rectify_node`。请确认你已经重新构建并重新 source：

```bash
cd /home/scl/work/slam/open_vins
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ov_msckf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

EuRoC 数据默认左右目同步，通常不需要开启近似同步。如果使用其他数据集且左右目时间戳不完全一致，可以尝试：

```bash
ros2 launch ov_msckf euroc_stereo_image_proc.launch.py \
  config:=euroc_mav approximate_sync:=true
```

### /stereo/points2 没有输出

先确认点云节点已经启动，且 `/stereo/points2` 有 publisher：

```bash
ros2 topic info -v /stereo/points2
ros2 node info /point_cloud_node
```

如果 `/stereo/points2` 有 publisher，但 `ros2 topic hz /stereo/points2` 没有输出，请检查点云节点启动命令是否使用了近似同步：

```bash
ros2 run stereo_image_proc point_cloud_node --ros-args \
  -p approximate_sync:=true \
  -p approximate_sync_tolerance_seconds:=0.05 \
  -p use_color:=false \
  -r left/image_rect_color:=/stereo/left/image_rect \
  -r /stereo/left/camera_info:=/cam0/camera_info \
  -r right/camera_info:=/cam1/camera_info \
  -r disparity:=/stereo/disparity \
  -r points2:=/stereo/points2
```

这里必须使用：

```bash
-r /stereo/left/camera_info:=/cam0/camera_info
```

`point_cloud_node` 会根据 `left/image_rect_color` 自动推导左 CameraInfo 话题为 `/stereo/left/camera_info`，所以仅写 `-r left/camera_info:=/cam0/camera_info` 通常不会生效。

还需要确认上游输入都在发布：

```bash
ros2 topic hz /stereo/left/image_rect
ros2 topic hz /stereo/disparity
ros2 topic echo --once /cam0/camera_info --field header
ros2 topic echo --once /cam1/camera_info --field header
```

## 与 OpenVINS 估计器的关系

该处理链只用于运行 ROS 2 `image_proc` 和 `stereo_image_proc`，生成矫正图像和视差图；不会改变 OpenVINS 估计器原本的订阅链路。`/imu0` 可以随 bag 一起播放，但 `stereo_image_proc` 本身不消费 IMU 数据。
