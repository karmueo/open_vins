# Repository Guidelines

## 项目结构与模块组织

OpenVINS 在仓库根目录下按多个 CMake/ROS 包组织。`ov_core/` 提供数学、相机模型、特征跟踪、仿真和通用工具代码。`ov_msckf/` 包含主要 VIO 估计器、ROS 可视化、launch 文件和辅助脚本。`ov_init/` 实现惯性初始化，`ov_eval/` 提供评估、轨迹对齐和绘图工具。`ov_data/` 存放小型数据集与标定资源，`config/` 按数据集或传感器保存 YAML 配置。文档与静态资源在 `docs/`，CI 与 Docker 配置在 `.github/workflows/` 和 `Dockerfile_*`。

## 构建、测试与开发命令

- `mkdir -p build && cd build && cmake ../ov_msckf && make`：构建 CI 使用的非 ROS 估计器路径。
- `catkin build`：在 ROS 1 catkin 工作空间中构建全部包。
- `colcon build`：在 ROS 2 工作空间中构建全部包。
- `roslaunch ov_msckf simulation.launch verbosity:=WARNING`：在 source `devel/setup.bash` 后运行 ROS 1 仿真冒烟测试。
- `ros2 run ov_msckf run_simulation src/open_vins/config/rpng_sim/estimator_config.yaml`：在 source `install/setup.bash` 后运行 ROS 2 仿真路径。

构建前确认已安装 Eigen、OpenCV、Boost、Ceres 以及目标 ROS 发行版。

## 本地数据集

`/mnt/data/slam/vicon_room1` 中的数据集是 EuRoC MAV 的 Vicon Room 1 子集，当前项目已支持。该目录包含 `V1_01_easy`、`V1_02_medium`、`V1_03_difficult`，每个序列均有 ROS 1 `.bag` 文件以及 EuRoC 标准 `mav0/` 结构；项目内对应配置为 `config/euroc_mav/`，真值文件在 `ov_data/euroc_mav/`。

ROS 1 下可在 source `devel/setup.bash` 后运行，例如：

- `roslaunch ov_msckf serial.launch config:=euroc_mav dataset:=V1_01_easy bag:=/mnt/data/slam/vicon_room1/V1_01_easy/V1_01_easy.bag`
- `roslaunch ov_msckf subscribe.launch config:=euroc_mav dataset:=V1_02_medium bag:=/mnt/data/slam/vicon_room1/V1_02_medium/V1_02_medium.bag dobag:=true`

已使用 conda 环境 `rosbags`（路径 `/home/scl/.conda/envs/rosbags`，安装 `rosbags` 工具）将上述三个 ROS 1 bag 转换为 ROS 2 bag，输出目录为 `/mnt/data/slam/vicon_room1_ros2`：

- `/mnt/data/slam/vicon_room1_ros2/V1_01_easy`
- `/mnt/data/slam/vicon_room1_ros2/V1_02_medium`
- `/mnt/data/slam/vicon_room1_ros2/V1_03_difficult`

ROS 2 下可在 source `install/setup.bash` 后启动 OpenVINS，并在另一个终端播放数据，例如：

- `ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav`
- `ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy`

## 编码风格与命名约定

C++ 代码使用 `.clang-format` 格式化：2 空格缩进、同行大括号、140 列限制、include 排序、指针右对齐。尽量保持头文件与源文件成对维护，例如 `State.h` 与 `State.cpp`。类型使用 `PascalCase`；函数和变量通常使用小写或 `snake_case`；测试和演示入口文件使用 `test_*.cpp` 或 `run_*.cpp`。新增逻辑优先复用当前包内已有工具，避免不必要的跨包依赖。

## 测试指南

测试主要是各包 CMake 文件中声明的独立 C++ 可执行程序，而不是单独的单元测试目录。新增测试应放在被测包附近，并沿用现有命名，例如 `ov_msckf/src/test_sim_meas.cpp` 或 `ov_init/src/test_simulation.cpp`。提交前至少构建相关的非 ROS、ROS 1 或 ROS 2 目标，并运行对应仿真或可执行测试。

## 提交与 Pull Request 规范

近期提交信息多为简洁的祈使句或描述句，例如 `support android logging`、`fix erase of old imu messages`、`enable building without ROS even if you have ROS`。主题应具体；关联问题时使用类似 `fix #358 calibration file documentation` 的格式。PR 应说明受影响的包、行为变化、已运行的构建或测试命令、使用的数据集或配置，以及相关 ROS 版本假设。
