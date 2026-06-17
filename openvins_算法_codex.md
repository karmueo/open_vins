<!--
本文档用于说明当前 OpenVINS 项目的核心视觉惯性里程计算法流程、状态模型、传播模型、视觉更新模型和关键工程参数。
-->

# OpenVINS 核心算法说明

## 1. 当前项目中的算法主线

当前仓库实现的是基于误差状态 EKF 的 OpenVINS / MSCKF 视觉惯性里程计。核心数据流是：IMU 高频传播当前惯性状态；相机到来时执行状态传播和克隆；特征跟踪提供跨帧观测；MSCKF 特征通过零空间投影消除临时三维点；SLAM 特征通过延迟初始化进入状态；可选 ZUPT 在静止时用惯性约束收缩不确定性；FEJ 用第一估计点计算雅可比以保持一致性。

### 步骤 1：系统处理时序

相机第 \(k\) 帧到来时，系统从上一帧时间 \(t_{k-1}\) 传播到当前时间 \(t_k\)，随后把当前 IMU 位姿克隆进滑动窗口，再用视觉特征执行更新。

$$
t_{\mathrm{imu}} = t_{\mathrm{cam}} + t_{\mathrm{off}}
$$

公式说明：该公式表示当前项目使用的相机时间与 IMU 时间之间的外参时间偏移关系。传播模块按照 IMU 时间选择积分区间，视觉更新按照相机时间管理 clone。

参数说明：\(t_{\mathrm{imu}}\) 表示 IMU 时钟下的时间；\(t_{\mathrm{cam}}\) 表示相机时钟下的时间；\(t_{\mathrm{off}}\) 表示相机到 IMU 的时间偏移，项目中对应 `calib_camimu_dt` / `_calib_dt_CAMtoIMU`。

数值参数化例子：若相机图像时间 \(t_{\mathrm{cam}}=12.300\,\mathrm{s}\)，标定时间偏移 \(t_{\mathrm{off}}=0.004\,\mathrm{s}\)，则 IMU 传播目标时间为 \(t_{\mathrm{imu}}=12.304\,\mathrm{s}\)。系统会选取覆盖上一状态 IMU 时间到 \(12.304\,\mathrm{s}\) 的 IMU 数据。

### 步骤 2：核心状态组成

OpenVINS 的状态包含当前 IMU 导航状态、滑动窗口 clone、相机标定参数、可选 IMU 内参、可选 SLAM 特征和协方差矩阵。默认配置里 FEJ 开启、传播方式为 RK4、最大 clone 数为 11、最大 SLAM 特征数为 25。

$$
\mathbf{x} =
\begin{bmatrix}
\mathbf{x}_I \\
\mathbf{x}_{C_1} \\
\cdots \\
\mathbf{x}_{C_c} \\
\mathbf{x}_{\mathrm{calib}} \\
\mathbf{x}_{\mathrm{slam}}
\end{bmatrix}
$$

公式说明：该公式表示滤波器整体状态向量由当前 IMU 状态、\(c\) 个历史 clone、标定状态和 SLAM 特征状态拼接形成。MSCKF 临时特征通常不会长期保留在状态中。

参数说明：\(\mathbf{x}\) 表示总状态；\(\mathbf{x}_I\) 表示当前 IMU 状态；\(\mathbf{x}_{C_i}\) 表示第 \(i\) 个 clone 位姿；\(c\) 表示当前 clone 数；\(\mathbf{x}_{\mathrm{calib}}\) 表示相机外参、内参、时间偏移和可选 IMU 内参；\(\mathbf{x}_{\mathrm{slam}}\) 表示长期保留的 SLAM / ArUco 特征。

数值参数化例子：若 IMU 误差状态为 15 维，当前有 11 个 clone 且每个 clone 是 6 维误差，相机时间偏移 1 维、双目外参共 \(2\times6=12\) 维、双目内参共 \(2\times8=16\) 维、SLAM 特征 25 个且每个 3 维，则总误差维度约为 \(15+11\times6+1+12+16+25\times3=185\)。

## 2. IMU 测量模型

### 步骤 3：陀螺仪与加速度计测量

IMU 提供角速度和比力测量。OpenVINS 的基本模型把测量值分解为真实物理量、传感器 bias 和白噪声。

$$
{}^I\boldsymbol{\omega}_m(t)
=
{}^I\boldsymbol{\omega}(t)
+ \mathbf{b}_g(t)
+ \mathbf{n}_g(t)
$$

公式说明：该公式表示陀螺仪测量角速度等于 IMU 坐标系中的真实角速度、陀螺仪零偏和陀螺仪白噪声之和。

参数说明：\({}^I\boldsymbol{\omega}_m(t)\) 表示时刻 \(t\) 在 IMU 坐标系 \(\{I\}\) 下的陀螺仪测量；\({}^I\boldsymbol{\omega}(t)\) 表示真实角速度；\(\mathbf{b}_g(t)\) 表示陀螺仪 bias；\(\mathbf{n}_g(t)\) 表示陀螺仪白噪声。

$$
{}^I\mathbf{a}_m(t)
=
{}^I\mathbf{a}(t)
+ {}^I_G\mathbf{R}(t){}^G\mathbf{g}
+ \mathbf{b}_a(t)
+ \mathbf{n}_a(t)
$$

公式说明：该公式表示加速度计测量等于 IMU 坐标系中的真实平动加速度、旋转到 IMU 坐标系的重力项、加速度计 bias 和加速度计白噪声之和。

参数说明：\({}^I\mathbf{a}_m(t)\) 表示时刻 \(t\) 在 IMU 坐标系 \(\{I\}\) 下的加速度计测量；\({}^I\mathbf{a}(t)\) 表示真实平动加速度；\({}^I_G\mathbf{R}(t)\) 表示从全局坐标系 \(\{G\}\) 到 IMU 坐标系 \(\{I\}\) 的旋转矩阵；\({}^G\mathbf{g}\) 表示全局坐标系中的重力向量；\(\mathbf{b}_a(t)\) 表示加速度计 bias；\(\mathbf{n}_a(t)\) 表示加速度计白噪声。

数值参数化例子：若静止传感器姿态接近水平，\({}^I_G\mathbf{R}=\mathbf{I}_3\)，\({}^G\mathbf{g}=[0,0,9.81]^\top\,\mathrm{m/s^2}\)，\(\mathbf{b}_a=[0.02,-0.01,0.03]^\top\,\mathrm{m/s^2}\)，噪声近似为零，则加速度计读数约为 \([0.02,-0.01,9.84]^\top\,\mathrm{m/s^2}\)。若陀螺仪真实角速度为 \([0,0,0.1]^\top\,\mathrm{rad/s}\)，\(\mathbf{b}_g=[0.001,-0.002,0.003]^\top\,\mathrm{rad/s}\)，则角速度测量约为 \([0.001,-0.002,0.103]^\top\,\mathrm{rad/s}\)。

### 步骤 4：可选 IMU 内参模型

当前项目支持 KALIBR 和 RPNG 形式的 IMU 内参。启用 IMU 内参标定时，传播会把尺度误差、轴不正交和陀螺仪重力敏感性纳入测量校正。

$$
{}^I\hat{\mathbf{a}}
=
{}^I_a\mathbf{R}\,\mathbf{D}_a
\left({}^a\mathbf{a}_m-\mathbf{b}_a\right)
$$

公式说明：该公式表示把加速度计原始测量通过加速度计内参矩阵、加速度计到 IMU 的旋转和 bias 校正后，得到 IMU 坐标系下的估计加速度。

参数说明：\({}^I\hat{\mathbf{a}}\) 表示校正后的加速度估计；\({}^I_a\mathbf{R}\) 表示从加速度计传感器坐标系到 IMU 坐标系的旋转；\(\mathbf{D}_a\) 表示加速度计尺度和轴不正交内参矩阵；\({}^a\mathbf{a}_m\) 表示加速度计坐标系下的原始测量；\(\mathbf{b}_a\) 表示加速度计 bias。

$$
{}^I\hat{\boldsymbol{\omega}}
=
{}^I_w\mathbf{R}\,\mathbf{D}_w
\left({}^w\boldsymbol{\omega}_m-\mathbf{b}_g-\mathbf{T}_g{}^I\hat{\mathbf{a}}\right)
$$

公式说明：该公式表示把陀螺仪原始测量减去 bias 和重力敏感项，再通过陀螺仪内参矩阵和传感器到 IMU 的旋转得到校正角速度。

参数说明：\({}^I\hat{\boldsymbol{\omega}}\) 表示校正后的角速度估计；\({}^I_w\mathbf{R}\) 表示从陀螺仪传感器坐标系到 IMU 坐标系的旋转；\(\mathbf{D}_w\) 表示陀螺仪尺度和轴不正交内参矩阵；\({}^w\boldsymbol{\omega}_m\) 表示陀螺仪坐标系下的原始测量；\(\mathbf{b}_g\) 表示陀螺仪 bias；\(\mathbf{T}_g\) 表示陀螺仪重力敏感矩阵；\({}^I\hat{\mathbf{a}}\) 表示校正后的加速度估计。

数值参数化例子：若不开启 IMU 内参标定，可近似取 \(\mathbf{D}_a=\mathbf{I}_3\)、\(\mathbf{D}_w=\mathbf{I}_3\)、\(\mathbf{T}_g=\mathbf{0}_{3\times3}\)、\({}^I_a\mathbf{R}=\mathbf{I}_3\)、\({}^I_w\mathbf{R}=\mathbf{I}_3\)。若 \({}^a\mathbf{a}_m=[0.10,0.00,9.90]^\top\)、\(\mathbf{b}_a=[0.02,0.00,0.04]^\top\)，则 \({}^I\hat{\mathbf{a}}=[0.08,0.00,9.86]^\top\)。若 \({}^w\boldsymbol{\omega}_m=[0.01,0.02,0.03]^\top\)、\(\mathbf{b}_g=[0.001,0.001,0.002]^\top\)，则 \({}^I\hat{\boldsymbol{\omega}}=[0.009,0.019,0.028]^\top\)。

## 3. 惯性状态与误差状态

### 步骤 5：IMU 名义状态

当前 IMU 状态保存全局到 IMU 的 JPL 四元数、IMU 在全局中的位置、速度、陀螺仪 bias 和加速度计 bias。

$$
\mathbf{x}_I(t)=
\begin{bmatrix}
{}^I_G\bar{\mathbf{q}}(t)\\
{}^G\mathbf{p}_I(t)\\
{}^G\mathbf{v}_I(t)\\
\mathbf{b}_g(t)\\
\mathbf{b}_a(t)
\end{bmatrix}
$$

公式说明：该公式表示 OpenVINS 的当前 IMU 名义状态。四元数用于表示姿态，位置和速度在全局坐标系表达，两个 bias 用随机游走建模。

参数说明：\(\mathbf{x}_I(t)\) 表示时刻 \(t\) 的 IMU 名义状态；\({}^I_G\bar{\mathbf{q}}(t)\) 表示从全局坐标系到 IMU 坐标系的单位四元数；\({}^G\mathbf{p}_I(t)\) 表示 IMU 原点在全局坐标系中的位置；\({}^G\mathbf{v}_I(t)\) 表示 IMU 原点在全局坐标系中的速度；\(\mathbf{b}_g(t)\) 表示陀螺仪 bias；\(\mathbf{b}_a(t)\) 表示加速度计 bias。

数值参数化例子：初始化后可有 \({}^I_G\bar{\mathbf{q}}=[0,0,0,1]^\top\)，\({}^G\mathbf{p}_I=[0,0,0]^\top\,\mathrm{m}\)，\({}^G\mathbf{v}_I=[0,0,0]^\top\,\mathrm{m/s}\)，\(\mathbf{b}_g=[0.001,-0.002,0.0005]^\top\,\mathrm{rad/s}\)，\(\mathbf{b}_a=[0.02,-0.01,0.03]^\top\,\mathrm{m/s^2}\)。

### 步骤 6：误差状态

滤波器协方差不直接使用 4 维四元数误差，而使用 3 维小角度误差，因而 IMU 误差状态是 15 维。

$$
{}^I_G\bar{\mathbf{q}}
=
\delta\bar{\mathbf{q}}\otimes{}^I_G\hat{\bar{\mathbf{q}}}
$$

公式说明：该公式表示真实四元数由小误差四元数左乘估计四元数得到。OpenVINS 使用左乘姿态误差定义。

参数说明：\({}^I_G\bar{\mathbf{q}}\) 表示真实全局到 IMU 姿态四元数；\(\delta\bar{\mathbf{q}}\) 表示小姿态误差四元数；\(\otimes\) 表示四元数乘法；\({}^I_G\hat{\bar{\mathbf{q}}}\) 表示估计的全局到 IMU 姿态四元数。

$$
\delta\bar{\mathbf{q}}
\simeq
\begin{bmatrix}
\frac{1}{2}\tilde{\boldsymbol{\theta}}\\
1
\end{bmatrix}
$$

公式说明：该公式表示小角度近似下的姿态误差四元数。误差协方差中使用 \(\tilde{\boldsymbol{\theta}}\) 的三个分量表示姿态误差。

参数说明：\(\delta\bar{\mathbf{q}}\) 表示小姿态误差四元数；\(\tilde{\boldsymbol{\theta}}\) 表示三维姿态小角度误差向量；\(\frac{1}{2}\tilde{\boldsymbol{\theta}}\) 表示四元数向量部的一阶近似；\(1\) 表示四元数标量部的一阶近似。

$$
\tilde{\mathbf{x}}_I(t)=
\begin{bmatrix}
{}^I_G\tilde{\boldsymbol{\theta}}(t)\\
{}^G\tilde{\mathbf{p}}_I(t)\\
{}^G\tilde{\mathbf{v}}_I(t)\\
\tilde{\mathbf{b}}_g(t)\\
\tilde{\mathbf{b}}_a(t)
\end{bmatrix}
$$

公式说明：该公式表示 IMU 的 15 维误差状态。误差状态用于传播协方差和视觉更新，更新后再注入名义状态。

参数说明：\(\tilde{\mathbf{x}}_I(t)\) 表示时刻 \(t\) 的 IMU 误差状态；\({}^I_G\tilde{\boldsymbol{\theta}}(t)\) 表示姿态小角度误差；\({}^G\tilde{\mathbf{p}}_I(t)\) 表示位置误差；\({}^G\tilde{\mathbf{v}}_I(t)\) 表示速度误差；\(\tilde{\mathbf{b}}_g(t)\) 表示陀螺仪 bias 误差；\(\tilde{\mathbf{b}}_a(t)\) 表示加速度计 bias 误差。

数值参数化例子：若一次 EKF 更新得到姿态误差 \(\tilde{\boldsymbol{\theta}}=[0.002,-0.001,0.0005]^\top\,\mathrm{rad}\)，则误差四元数近似为 \([0.001,-0.0005,0.00025,1]^\top\)。若位置误差为 \([0.01,-0.02,0.005]^\top\,\mathrm{m}\)，速度误差为 \([0.03,0.00,-0.01]^\top\,\mathrm{m/s}\)，这些误差会按状态类型注入到名义状态。

## 4. IMU 连续运动模型

### 步骤 7：姿态、位置、速度和 bias 连续时间动力学

OpenVINS 以 IMU 运动学作为传播模型。姿态由角速度积分，位置由速度积分，速度由去重力后的加速度积分，两个 bias 按随机游走演化。

$$
{}^I_G\dot{\bar{\mathbf{q}}}(t)
=
\frac{1}{2}\boldsymbol{\Omega}\left({}^I\boldsymbol{\omega}(t)\right)
{}^I_G\bar{\mathbf{q}}(t)
$$

公式说明：该公式表示四元数姿态的连续时间运动学方程。角速度通过矩阵 \(\boldsymbol{\Omega}(\cdot)\) 作用在当前四元数上。

参数说明：\({}^I_G\dot{\bar{\mathbf{q}}}(t)\) 表示四元数对时间的导数；\(\boldsymbol{\Omega}({}^I\boldsymbol{\omega}(t))\) 表示由 IMU 坐标系角速度构造的四元数运动矩阵；\({}^I\boldsymbol{\omega}(t)\) 表示真实角速度；\({}^I_G\bar{\mathbf{q}}(t)\) 表示全局到 IMU 的单位四元数。

$$
{}^G\dot{\mathbf{p}}_I(t)
=
{}^G\mathbf{v}_I(t)
$$

公式说明：该公式表示位置导数等于速度。

参数说明：\({}^G\dot{\mathbf{p}}_I(t)\) 表示 IMU 全局位置对时间的导数；\({}^G\mathbf{v}_I(t)\) 表示 IMU 在全局坐标系中的速度。

$$
{}^G\dot{\mathbf{v}}_I(t)
=
{}^I_G\mathbf{R}(t)^\top{}^I\mathbf{a}(t)
-{}^G\mathbf{g}
$$

公式说明：该公式表示全局速度导数由 IMU 坐标系加速度旋转到全局坐标系后减去重力得到。当前项目的离散传播公式中也采用该符号约定。

参数说明：\({}^G\dot{\mathbf{v}}_I(t)\) 表示 IMU 全局速度对时间的导数；\({}^I_G\mathbf{R}(t)^\top\) 表示从 IMU 坐标系到全局坐标系的旋转；\({}^I\mathbf{a}(t)\) 表示 IMU 坐标系下去 bias 后的真实加速度项；\({}^G\mathbf{g}\) 表示全局坐标系中的重力向量。

$$
\dot{\mathbf{b}}_g(t)=\mathbf{n}_{wg}(t),
\qquad
\dot{\mathbf{b}}_a(t)=\mathbf{n}_{wa}(t)
$$

公式说明：该公式表示陀螺仪 bias 和加速度计 bias 都按照随机游走模型变化，变化率由白噪声驱动。

参数说明：\(\dot{\mathbf{b}}_g(t)\) 表示陀螺仪 bias 对时间的导数；\(\mathbf{n}_{wg}(t)\) 表示陀螺仪 bias 随机游走噪声；\(\dot{\mathbf{b}}_a(t)\) 表示加速度计 bias 对时间的导数；\(\mathbf{n}_{wa}(t)\) 表示加速度计 bias 随机游走噪声。

数值参数化例子：若当前 \({}^G\mathbf{v}_I=[1,0,0]^\top\,\mathrm{m/s}\)，\({}^I_G\mathbf{R}=\mathbf{I}_3\)，去 bias 后加速度 \({}^I\mathbf{a}=[0.1,0,9.81]^\top\,\mathrm{m/s^2}\)，\({}^G\mathbf{g}=[0,0,9.81]^\top\,\mathrm{m/s^2}\)，则速度导数是 \([0.1,0,0]^\top\,\mathrm{m/s^2}\)，位置导数是 \([1,0,0]^\top\,\mathrm{m/s}\)。

## 5. 离散 IMU 传播

### 步骤 8：离散姿态传播

项目支持 `discrete`、`rk4` 和 `analytical` 三种传播形式，默认配置为 `rk4`。为了理解核心结构，可先看常值角速度假设下的离散形式。

$$
{}^{I_{k+1}}_G\hat{\bar{\mathbf{q}}}
=
\exp\left(
\frac{1}{2}\boldsymbol{\Omega}\left(\hat{\boldsymbol{\omega}}_k\right)\Delta t_k
\right)
{}^{I_k}_G\hat{\bar{\mathbf{q}}}
$$

公式说明：该公式表示在时间区间 \([t_k,t_{k+1}]\) 内用校正后的角速度传播四元数姿态。

参数说明：\({}^{I_{k+1}}_G\hat{\bar{\mathbf{q}}}\) 表示传播后的全局到 IMU 姿态估计；\(\exp(\cdot)\) 表示矩阵指数；\(\boldsymbol{\Omega}(\hat{\boldsymbol{\omega}}_k)\) 表示由校正角速度构造的四元数运动矩阵；\(\hat{\boldsymbol{\omega}}_k\) 表示第 \(k\) 个 IMU 区间的校正角速度；\(\Delta t_k=t_{k+1}-t_k\) 表示 IMU 积分时间间隔；\({}^{I_k}_G\hat{\bar{\mathbf{q}}}\) 表示传播前的全局到 IMU 姿态估计。

数值参数化例子：若 \(\hat{\boldsymbol{\omega}}_k=[0,0,0.1]^\top\,\mathrm{rad/s}\)，\(\Delta t_k=0.01\,\mathrm{s}\)，则该区间绕 \(z\) 轴旋转约 \(0.001\,\mathrm{rad}\)。若传播前四元数接近单位姿态，传播后姿态会产生一个很小的 yaw 变化。

### 步骤 9：离散位置和速度传播

对一个 IMU 小区间，常值加速度近似下的位置和速度传播形式如下。

$$
{}^G\hat{\mathbf{p}}_{I_{k+1}}
=
{}^G\hat{\mathbf{p}}_{I_k}
+{}^G\hat{\mathbf{v}}_{I_k}\Delta t_k
+\frac{1}{2}{}^{I_k}_G\hat{\mathbf{R}}^\top\hat{\mathbf{a}}_k\Delta t_k^2
-\frac{1}{2}{}^G\mathbf{g}\Delta t_k^2
$$

公式说明：该公式表示位置由旧位置、旧速度积分、IMU 加速度积分和重力项共同传播得到。

参数说明：\({}^G\hat{\mathbf{p}}_{I_{k+1}}\) 表示传播后的 IMU 全局位置估计；\({}^G\hat{\mathbf{p}}_{I_k}\) 表示传播前的 IMU 全局位置估计；\({}^G\hat{\mathbf{v}}_{I_k}\) 表示传播前的 IMU 全局速度估计；\(\Delta t_k\) 表示 IMU 积分时间间隔；\({}^{I_k}_G\hat{\mathbf{R}}^\top\) 表示从 IMU 坐标系到全局坐标系的估计旋转；\(\hat{\mathbf{a}}_k\) 表示校正后的加速度；\({}^G\mathbf{g}\) 表示全局重力向量。

$$
{}^G\hat{\mathbf{v}}_{I_{k+1}}
=
{}^G\hat{\mathbf{v}}_{I_k}
+{}^{I_k}_G\hat{\mathbf{R}}^\top\hat{\mathbf{a}}_k\Delta t_k
-{}^G\mathbf{g}\Delta t_k
$$

公式说明：该公式表示速度由旧速度、IMU 加速度积分和重力积分共同传播得到。

参数说明：\({}^G\hat{\mathbf{v}}_{I_{k+1}}\) 表示传播后的 IMU 全局速度估计；\({}^G\hat{\mathbf{v}}_{I_k}\) 表示传播前的 IMU 全局速度估计；\({}^{I_k}_G\hat{\mathbf{R}}^\top\) 表示从 IMU 坐标系到全局坐标系的估计旋转；\(\hat{\mathbf{a}}_k\) 表示校正后的加速度；\(\Delta t_k\) 表示 IMU 积分时间间隔；\({}^G\mathbf{g}\) 表示全局重力向量。

数值参数化例子：若 \({}^G\hat{\mathbf{p}}_{I_k}=[0,0,0]^\top\,\mathrm{m}\)，\({}^G\hat{\mathbf{v}}_{I_k}=[1,0,0]^\top\,\mathrm{m/s}\)，\({}^{I_k}_G\hat{\mathbf{R}}=\mathbf{I}_3\)，\(\hat{\mathbf{a}}_k=[0.2,0,9.81]^\top\,\mathrm{m/s^2}\)，\({}^G\mathbf{g}=[0,0,9.81]^\top\,\mathrm{m/s^2}\)，\(\Delta t_k=0.01\,\mathrm{s}\)，则速度更新约为 \([1.002,0,0]^\top\,\mathrm{m/s}\)，位置更新约为 \([0.01001,0,0]^\top\,\mathrm{m}\)。

### 步骤 10：误差状态传播

误差状态的传播用状态转移矩阵和噪声雅可比描述。项目中会对每个 IMU 小区间累乘状态转移，并累加过程噪声。

$$
\tilde{\mathbf{x}}_{I,k+1}
=
\boldsymbol{\Phi}_{k+1,k}\tilde{\mathbf{x}}_{I,k}
+\mathbf{G}_k\mathbf{n}_k
$$

公式说明：该公式表示一个 IMU 区间内的线性化误差状态传播。协方差传播依赖该线性模型。

参数说明：\(\tilde{\mathbf{x}}_{I,k+1}\) 表示传播后的 IMU 误差状态；\(\boldsymbol{\Phi}_{k+1,k}\) 表示从 \(t_k\) 到 \(t_{k+1}\) 的误差状态转移矩阵；\(\tilde{\mathbf{x}}_{I,k}\) 表示传播前的 IMU 误差状态；\(\mathbf{G}_k\) 表示过程噪声雅可比；\(\mathbf{n}_k\) 表示离散过程噪声向量。

$$
\mathbf{P}_{k+1|k}
=
\boldsymbol{\Phi}_{k+1,k}
\,\mathbf{P}_{k|k}\,
\boldsymbol{\Phi}_{k+1,k}^{\top}
+\mathbf{G}_k\mathbf{Q}_{d,k}\mathbf{G}_k^{\top}
$$

公式说明：该公式表示 IMU 误差协方差从当前后验传播到下一时刻先验。当前项目在多个 IMU 小区间内累积 \(\boldsymbol{\Phi}\) 和过程噪声，再一次性更新相关协方差块。

参数说明：\(\mathbf{P}_{k+1|k}\) 表示 \(t_{k+1}\) 时刻的先验协方差；\(\boldsymbol{\Phi}_{k+1,k}\) 表示误差状态转移矩阵；\(\mathbf{P}_{k|k}\) 表示 \(t_k\) 时刻的后验协方差；\(\mathbf{G}_k\) 表示过程噪声雅可比；\(\mathbf{Q}_{d,k}\) 表示离散过程噪声协方差。

数值参数化例子：若一个简化一维状态的方差为 \(P_{k|k}=0.04\)，状态转移为 \(\Phi=1.0\)，过程噪声方差为 \(Q_d=0.0001\)，则传播后方差为 \(P_{k+1|k}=0.0401\)。在真实系统中同样思想扩展到完整矩阵，位置、速度、姿态和 bias 会互相关联。

### 步骤 11：连续噪声转离散噪声

OpenVINS 配置中 IMU 噪声通常来自连续时间噪声密度，传播时需要按 IMU 时间间隔转换为离散噪声。

$$
\mathbf{Q}_{d,k}=
\begin{bmatrix}
\frac{\sigma_g^2}{\Delta t_k}\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
\mathbf{0}_3 & \frac{\sigma_a^2}{\Delta t_k}\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3\\
\mathbf{0}_3 & \mathbf{0}_3 & \sigma_{wg}^2\Delta t_k\mathbf{I}_3 & \mathbf{0}_3\\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \sigma_{wa}^2\Delta t_k\mathbf{I}_3
\end{bmatrix}
$$

公式说明：该公式表示离散 IMU 噪声协方差的常见构造方式。测量白噪声项按 \(1/\Delta t_k\) 放缩，bias 随机游走项按 \(\Delta t_k\) 放缩。

参数说明：\(\mathbf{Q}_{d,k}\) 表示第 \(k\) 个 IMU 区间的离散噪声协方差；\(\sigma_g\) 表示陀螺仪连续时间噪声密度；\(\sigma_a\) 表示加速度计连续时间噪声密度；\(\sigma_{wg}\) 表示陀螺仪 bias 随机游走噪声密度；\(\sigma_{wa}\) 表示加速度计 bias 随机游走噪声密度；\(\Delta t_k\) 表示 IMU 积分时间间隔；\(\mathbf{I}_3\) 表示 \(3\times3\) 单位矩阵；\(\mathbf{0}_3\) 表示 \(3\times3\) 零矩阵。

数值参数化例子：若 \(\Delta t_k=0.005\,\mathrm{s}\)，\(\sigma_g=1.7\times10^{-4}\,\mathrm{rad/s/\sqrt{Hz}}\)，则角速度测量噪声离散方差约为 \((1.7\times10^{-4})^2/0.005=5.78\times10^{-6}\)。若 \(\sigma_{wg}=1.9\times10^{-5}\,\mathrm{rad/s^2/\sqrt{Hz}}\)，则陀螺仪 bias 随机游走离散方差约为 \((1.9\times10^{-5})^2\times0.005=1.805\times10^{-12}\)。

## 6. 相机投影与测量模型

### 步骤 12：世界点到相机坐标

视觉更新的几何核心是把全局特征点投影到当前相机坐标系，再投影到归一化平面和像素平面。

$$
{}^{C_k}\mathbf{p}_f
=
{}^C_I\mathbf{R}\,{}^{I_k}_G\mathbf{R}
\left({}^G\mathbf{p}_f-{}^G\mathbf{p}_{I_k}\right)
+{}^C\mathbf{p}_I
$$

公式说明：该公式表示把全局三维特征点转换到第 \(k\) 帧相机坐标系。项目状态中保存的是 IMU 位姿和 IMU 到相机外参，因此需要先从全局到 IMU，再从 IMU 到相机。

参数说明：\({}^{C_k}\mathbf{p}_f\) 表示特征点在第 \(k\) 帧相机坐标系中的坐标；\({}^C_I\mathbf{R}\) 表示从 IMU 坐标系到相机坐标系的旋转外参；\({}^{I_k}_G\mathbf{R}\) 表示从全局坐标系到第 \(k\) 帧 IMU 坐标系的旋转；\({}^G\mathbf{p}_f\) 表示特征点全局坐标；\({}^G\mathbf{p}_{I_k}\) 表示第 \(k\) 帧 IMU 全局位置；\({}^C\mathbf{p}_I\) 表示 IMU 原点在相机坐标系中的位置。

数值参数化例子：若 \({}^C_I\mathbf{R}=\mathbf{I}_3\)，\({}^{I_k}_G\mathbf{R}=\mathbf{I}_3\)，\({}^G\mathbf{p}_f=[2,0,5]^\top\,\mathrm{m}\)，\({}^G\mathbf{p}_{I_k}=[1,0,0]^\top\,\mathrm{m}\)，\({}^C\mathbf{p}_I=[0.05,0,0]^\top\,\mathrm{m}\)，则 \({}^{C_k}\mathbf{p}_f=[1.05,0,5]^\top\,\mathrm{m}\)。

### 步骤 13：归一化针孔投影

特征点在相机坐标系中被投影到归一化图像平面。

$$
\mathbf{z}_{n,k}
=
\begin{bmatrix}
{}^Cx/{}^Cz\\
{}^Cy/{}^Cz
\end{bmatrix}
$$

公式说明：该公式表示针孔相机的归一化投影。深度 \({}^Cz\) 必须为正，特征在相机前方才能形成有效观测。

参数说明：\(\mathbf{z}_{n,k}\) 表示第 \(k\) 帧归一化无畸变图像坐标；\({}^Cx\) 表示特征点在相机坐标系中的 \(x\) 坐标；\({}^Cy\) 表示特征点在相机坐标系中的 \(y\) 坐标；\({}^Cz\) 表示特征点在相机坐标系中的 \(z\) 深度。

$$
\frac{\partial \mathbf{z}_{n,k}}{\partial{}^{C_k}\mathbf{p}_f}
=
\begin{bmatrix}
\frac{1}{{}^Cz} & 0 & -\frac{{}^Cx}{({}^Cz)^2}\\
0 & \frac{1}{{}^Cz} & -\frac{{}^Cy}{({}^Cz)^2}
\end{bmatrix}
$$

公式说明：该公式表示归一化投影对相机坐标系三维点的雅可比，是视觉残差线性化的基础。

参数说明：\(\frac{\partial \mathbf{z}_{n,k}}{\partial{}^{C_k}\mathbf{p}_f}\) 表示归一化坐标对相机系三维点坐标的导数矩阵；\(\mathbf{z}_{n,k}\) 表示归一化无畸变图像坐标；\({}^{C_k}\mathbf{p}_f\) 表示第 \(k\) 帧相机坐标系中的特征点坐标；\({}^Cx\)、\({}^Cy\)、\({}^Cz\) 分别表示相机坐标系中的三维坐标分量。

数值参数化例子：若 \({}^{C_k}\mathbf{p}_f=[1.05,0,5]^\top\,\mathrm{m}\)，则归一化坐标为 \([0.21,0]^\top\)。对应投影雅可比第一行为 \([0.2,0,-0.042]\)，第二行为 \([0,0.2,0]\)。

### 步骤 14：像素测量与畸变

当前项目支持 `radtan` 和 `equidistant` 相机模型。以 radtan 为例，归一化坐标经径向和切向畸变后得到像素坐标。

$$
\begin{bmatrix}
u\\
v
\end{bmatrix}
=
\begin{bmatrix}
f_x x+c_x\\
f_y y+c_y
\end{bmatrix}
$$

公式说明：该公式表示把畸变后的归一化坐标映射到像素坐标。

参数说明：\(u\) 表示像素横坐标；\(v\) 表示像素纵坐标；\(f_x\) 表示相机 \(x\) 方向焦距；\(f_y\) 表示相机 \(y\) 方向焦距；\(x\) 表示畸变后的归一化横坐标；\(y\) 表示畸变后的归一化纵坐标；\(c_x\) 表示主点横坐标；\(c_y\) 表示主点纵坐标。

$$
x=x_n(1+k_1r^2+k_2r^4)+2p_1x_ny_n+p_2(r^2+2x_n^2)
$$

公式说明：该公式表示 radtan 模型下畸变后横向归一化坐标的计算方式。

参数说明：\(x\) 表示畸变后的归一化横坐标；\(x_n\) 表示无畸变归一化横坐标；\(y_n\) 表示无畸变归一化纵坐标；\(k_1\) 表示一阶径向畸变系数；\(k_2\) 表示二阶径向畸变系数；\(p_1\) 表示第一切向畸变系数；\(p_2\) 表示第二切向畸变系数；\(r^2=x_n^2+y_n^2\) 表示归一化半径平方。

$$
y=y_n(1+k_1r^2+k_2r^4)+p_1(r^2+2y_n^2)+2p_2x_ny_n
$$

公式说明：该公式表示 radtan 模型下畸变后纵向归一化坐标的计算方式。

参数说明：\(y\) 表示畸变后的归一化纵坐标；\(x_n\) 表示无畸变归一化横坐标；\(y_n\) 表示无畸变归一化纵坐标；\(k_1\) 表示一阶径向畸变系数；\(k_2\) 表示二阶径向畸变系数；\(p_1\) 表示第一切向畸变系数；\(p_2\) 表示第二切向畸变系数；\(r^2=x_n^2+y_n^2\) 表示归一化半径平方。

数值参数化例子：若 \(x_n=0.21\)、\(y_n=0.00\)、\(k_1=-0.10\)、\(k_2=0.01\)、\(p_1=0.001\)、\(p_2=0.001\)，则 \(r^2=0.0441\)，\(x\approx0.2091\)，\(y\approx0.000044\)。若 \(f_x=f_y=460\)、\(c_x=320\)、\(c_y=240\)，则像素坐标约为 \(u=416.19\)、\(v=240.02\)。

## 7. 特征初始化

### 步骤 15：线性三角化

MSCKF 和 SLAM 特征进入更新前，会先根据 clone 相机位姿三角化三维点。当前项目选择观测最多的相机作为 anchor，并在线性系统中求解 anchor 坐标系下的特征点。

$$
\mathbf{A}\,{}^A\mathbf{p}_f=\mathbf{b}
$$

公式说明：该公式表示线性三角化得到的三维点求解问题。项目中 \(\mathbf{A}\) 和 \(\mathbf{b}\) 由多帧归一化 bearing 和相机相对位姿累计得到。

参数说明：\(\mathbf{A}\) 表示由多帧 bearing 约束累计形成的 \(3\times3\) 系数矩阵；\({}^A\mathbf{p}_f\) 表示特征点在 anchor 相机坐标系中的三维坐标；\(\mathbf{b}\) 表示由相机相对平移和 bearing 约束累计形成的三维向量。

$$
{}^G\mathbf{p}_f
=
{}^A_G\mathbf{R}^{\top}{}^A\mathbf{p}_f
+{}^G\mathbf{p}_A
$$

公式说明：该公式表示把 anchor 坐标系中的三维特征点转换到全局坐标系。

参数说明：\({}^G\mathbf{p}_f\) 表示特征点全局坐标；\({}^A_G\mathbf{R}^{\top}\) 表示从 anchor 坐标系到全局坐标系的旋转；\({}^A\mathbf{p}_f\) 表示特征点在 anchor 坐标系中的坐标；\({}^G\mathbf{p}_A\) 表示 anchor 相机原点在全局坐标系中的位置。

数值参数化例子：若线性系统求得 \({}^A\mathbf{p}_f=[0.5,0.1,4.0]^\top\,\mathrm{m}\)，anchor 相机全局位置 \({}^G\mathbf{p}_A=[1,0,0]^\top\,\mathrm{m}\)，anchor 姿态近似全局对齐，则 \({}^G\mathbf{p}_f=[1.5,0.1,4.0]^\top\,\mathrm{m}\)。若项目默认 `min_dist=0.10`、`max_dist=60`，该点深度 \(4.0\,\mathrm{m}\) 在合法范围内。

### 步骤 16：逆深度参数优化

项目默认会对三角化结果做少量迭代优化。优化内部常使用 anchor 相机坐标下的 \(\alpha,\beta,\rho\) 表示。

$$
{}^A\mathbf{p}_f
=
\frac{1}{\rho}
\begin{bmatrix}
\alpha\\
\beta\\
1
\end{bmatrix}
$$

公式说明：该公式表示 MSCKF 风格锚定逆深度参数化。三维点由归一化横向 bearing、纵向 bearing 和逆深度共同确定。

参数说明：\({}^A\mathbf{p}_f\) 表示特征点在 anchor 坐标系中的三维坐标；\(\rho\) 表示逆深度；\(\alpha\) 表示归一化横向 bearing 参数；\(\beta\) 表示归一化纵向 bearing 参数；\([\,\alpha,\beta,1\,]^\top\) 表示 anchor 相机坐标系下的非单位 bearing 向量。

$$
\min_{\alpha,\beta,\rho}
\sum_i
\left\|
\mathbf{z}_{n,i}
-
\pi\left({}^{C_i}_A\mathbf{R}{}^A\mathbf{p}_f+{}^{C_i}\mathbf{p}_A\right)
\right\|^2
$$

公式说明：该公式表示特征初始化的重投影误差优化目标。项目默认最多迭代 5 次，初始阻尼为 \(10^{-3}\)，并用步长和 cost 降低量判断收敛。

参数说明：\(\alpha\) 表示归一化横向 bearing 参数；\(\beta\) 表示归一化纵向 bearing 参数；\(\rho\) 表示逆深度；\(\sum_i\) 表示对所有观测求和；\(\mathbf{z}_{n,i}\) 表示第 \(i\) 个归一化观测；\(\pi(\cdot)\) 表示针孔归一化投影函数；\({}^{C_i}_A\mathbf{R}\) 表示从 anchor 坐标系到第 \(i\) 个相机坐标系的旋转；\({}^A\mathbf{p}_f\) 表示特征点在 anchor 坐标系中的三维坐标；\({}^{C_i}\mathbf{p}_A\) 表示 anchor 原点在第 \(i\) 个相机坐标系中的坐标。

数值参数化例子：若三角化得到 \({}^A\mathbf{p}_f=[0.5,0.1,4.0]^\top\)，则 \(\alpha=0.5/4.0=0.125\)，\(\beta=0.1/4.0=0.025\)，\(\rho=1/4.0=0.25\)。若一次迭代后参数增量 \(\Delta=[0.0002,-0.0001,0.0003]^\top\)，其范数约为 \(3.74\times10^{-4}\)，大于默认 `min_dx=1e-6`，则还没有达到步长收敛条件。

## 8. 视觉残差线性化

### 步骤 17：单个观测残差

视觉测量更新使用观测像素或归一化坐标与预测投影之间的差作为残差。

$$
\mathbf{r}_{i}
=
\mathbf{z}_{i}
-\mathbf{h}\left(\hat{\mathbf{x}},\hat{\boldsymbol{\lambda}}\right)
$$

公式说明：该公式表示第 \(i\) 个视觉观测残差。OpenVINS 内部会根据相机模型、外参、clone 位姿和特征表示计算预测观测。

参数说明：\(\mathbf{r}_i\) 表示第 \(i\) 个二维视觉残差；\(\mathbf{z}_i\) 表示第 \(i\) 个实际视觉观测；\(\mathbf{h}(\hat{\mathbf{x}},\hat{\boldsymbol{\lambda}})\) 表示在当前状态估计和特征估计处计算的预测观测；\(\hat{\mathbf{x}}\) 表示当前状态估计；\(\hat{\boldsymbol{\lambda}}\) 表示当前特征参数估计。

$$
\mathbf{r}
\simeq
\mathbf{H}_x\tilde{\mathbf{x}}
+\mathbf{H}_f\tilde{\boldsymbol{\lambda}}
+\mathbf{n}
$$

公式说明：该公式表示多帧视觉残差的一阶线性化。MSCKF 更新会消除 \(\tilde{\boldsymbol{\lambda}}\)，SLAM 更新会把 \(\tilde{\boldsymbol{\lambda}}\) 作为状态的一部分。

参数说明：\(\mathbf{r}\) 表示堆叠后的视觉残差；\(\mathbf{H}_x\) 表示残差对滤波器状态误差的雅可比；\(\tilde{\mathbf{x}}\) 表示滤波器状态误差；\(\mathbf{H}_f\) 表示残差对特征参数误差的雅可比；\(\tilde{\boldsymbol{\lambda}}\) 表示特征参数误差；\(\mathbf{n}\) 表示视觉测量噪声。

数值参数化例子：若某特征被 5 帧观测到，每帧提供二维观测，则 \(\mathbf{r}\) 是 \(10\times1\) 向量。若当前状态相关维度为 81，特征参数为三维点，则 \(\mathbf{H}_x\) 是 \(10\times81\)，\(\mathbf{H}_f\) 是 \(10\times3\)。

## 9. MSCKF 零空间更新

### 步骤 18：消除临时特征状态

MSCKF 的核心思想是用多帧观测约束 clone 和当前状态，同时通过左零空间投影消除临时特征点，避免把每个普通特征长期加入状态。

$$
\mathbf{H}_f
=
\begin{bmatrix}
\mathbf{Q}_1 & \mathbf{Q}_2
\end{bmatrix}
\begin{bmatrix}
\mathbf{R}_1\\
\mathbf{0}
\end{bmatrix}
$$

公式说明：该公式表示对特征雅可比 \(\mathbf{H}_f\) 做 QR 分解。矩阵 \(\mathbf{Q}_2\) 张成 \(\mathbf{H}_f^\top\) 的左零空间。

参数说明：\(\mathbf{H}_f\) 表示视觉残差对临时特征误差的雅可比；\(\mathbf{Q}_1\) 表示 QR 分解中对应 \(\mathbf{H}_f\) 列空间的正交基；\(\mathbf{Q}_2\) 表示 QR 分解中对应 \(\mathbf{H}_f\) 左零空间的正交基；\(\mathbf{R}_1\) 表示上三角矩阵；\(\mathbf{0}\) 表示零矩阵块。

$$
\mathbf{Q}_2^\top\mathbf{r}
\simeq
\mathbf{Q}_2^\top\mathbf{H}_x\tilde{\mathbf{x}}
+\mathbf{Q}_2^\top\mathbf{n}
$$

公式说明：该公式表示左乘零空间后，临时特征误差项被消除，剩余残差只约束滤波器状态。

参数说明：\(\mathbf{Q}_2^\top\mathbf{r}\) 表示零空间投影后的残差；\(\mathbf{Q}_2^\top\mathbf{H}_x\) 表示零空间投影后的状态雅可比；\(\tilde{\mathbf{x}}\) 表示滤波器状态误差；\(\mathbf{Q}_2^\top\mathbf{n}\) 表示零空间投影后的视觉噪声；\(\mathbf{Q}_2\) 表示特征雅可比左零空间基。

数值参数化例子：若一个三维特征有 6 次二维观测，则原始残差维度为 \(12\)，\(\mathbf{H}_f\) 维度为 \(12\times3\)。若 \(\mathbf{H}_f\) 满秩，零空间投影后的残差维度为 \(12-3=9\)。这 9 个约束会参与 MSCKF EKF 更新。

### 步骤 19：卡方门限剔除异常特征

OpenVINS 对每个特征线性系统做卡方检验。MSCKF 和 SLAM 更新器都预先构造 0.95 置信度的卡方表，并乘以配置中的 `chi2_multipler`。

$$
\chi^2
=
\mathbf{r}^{\top}
\left(\mathbf{H}\mathbf{P}\mathbf{H}^{\top}+\mathbf{R}\right)^{-1}
\mathbf{r}
$$

公式说明：该公式表示归一化创新平方，用于判断该特征观测是否与当前状态和协方差一致。

参数说明：\(\chi^2\) 表示卡方统计量；\(\mathbf{r}\) 表示当前特征的残差；\(\mathbf{H}\) 表示当前特征的状态雅可比；\(\mathbf{P}\) 表示参与该特征更新的状态边缘协方差；\(\mathbf{R}\) 表示视觉测量噪声协方差。

$$
\chi^2
\le
\gamma\,\chi^2_{\nu,0.95}
$$

公式说明：该公式表示特征通过卡方门限的条件。若统计量超过门限，当前项目会剔除该特征，避免异常匹配或错误三角化污染状态。

参数说明：\(\chi^2\) 表示卡方统计量；\(\gamma\) 表示配置中的卡方门限倍率，对应 `chi2_multipler`；\(\chi^2_{\nu,0.95}\) 表示自由度为 \(\nu\)、置信度为 0.95 的卡方分布分位数；\(\nu\) 表示残差维度。

数值参数化例子：若零空间投影后残差维度 \(\nu=9\)，0.95 卡方分位数约为 16.92，默认倍率 \(\gamma=5\)，则门限约为 84.6。若计算得到 \(\chi^2=30\)，该特征通过；若 \(\chi^2=120\)，该特征会被剔除。

### 步骤 20：测量压缩

大量 MSCKF 特征堆叠后会产生很高维的残差。项目对投影后的总雅可比做 QR 压缩，降低 EKF 更新矩阵规模。

$$
\mathbf{H}_o
=
\mathbf{Q}_1\mathbf{R}_1
$$

公式说明：该公式表示对零空间投影后的总状态雅可比做薄 QR 分解，保留与状态维度相关的有效约束。

参数说明：\(\mathbf{H}_o\) 表示零空间投影后的总状态雅可比；\(\mathbf{Q}_1\) 表示正交矩阵的有效列空间部分；\(\mathbf{R}_1\) 表示压缩后的上三角雅可比。

$$
\mathbf{Q}_1^\top\mathbf{r}_o
\simeq
\mathbf{R}_1\tilde{\mathbf{x}}
+\mathbf{Q}_1^\top\mathbf{n}_o
$$

公式说明：该公式表示压缩后的等价线性测量。压缩不会改变信息内容，主要降低后续 EKF 更新成本。

参数说明：\(\mathbf{Q}_1^\top\mathbf{r}_o\) 表示压缩后的残差；\(\mathbf{r}_o\) 表示零空间投影后的原始残差；\(\mathbf{R}_1\) 表示压缩后的状态雅可比；\(\tilde{\mathbf{x}}\) 表示滤波器状态误差；\(\mathbf{Q}_1^\top\mathbf{n}_o\) 表示压缩后的测量噪声；\(\mathbf{n}_o\) 表示零空间投影后的测量噪声。

数值参数化例子：若一次 MSCKF 更新中通过筛选的特征形成 300 维残差，而参与状态维度为 90，则压缩后残差行数最多约为 90。EKF 更新中的矩阵求逆规模由 \(300\times300\) 下降到约 \(90\times90\)。

## 10. EKF 更新

### 步骤 21：误差状态更新

无论 MSCKF、SLAM 还是 ZUPT，最终都形成线性残差并调用 EKF 更新。

$$
\mathbf{S}
=
\mathbf{H}\mathbf{P}\mathbf{H}^{\top}
+\mathbf{R}
$$

公式说明：该公式表示创新协方差。它描述预测观测不确定性与测量噪声的总和。

参数说明：\(\mathbf{S}\) 表示创新协方差；\(\mathbf{H}\) 表示线性测量雅可比；\(\mathbf{P}\) 表示当前先验状态协方差；\(\mathbf{R}\) 表示测量噪声协方差。

$$
\mathbf{K}
=
\mathbf{P}\mathbf{H}^{\top}\mathbf{S}^{-1}
$$

公式说明：该公式表示 Kalman 增益。增益决定测量残差注入状态误差的权重。

参数说明：\(\mathbf{K}\) 表示 Kalman 增益；\(\mathbf{P}\) 表示当前先验状态协方差；\(\mathbf{H}^{\top}\) 表示线性测量雅可比的转置；\(\mathbf{S}^{-1}\) 表示创新协方差的逆。

$$
\delta\hat{\mathbf{x}}
=
\mathbf{K}\mathbf{r}
$$

公式说明：该公式表示由 Kalman 增益和残差计算误差状态修正量。该修正量随后按不同状态类型注入四元数、位置、速度、bias、标定量和特征。

参数说明：\(\delta\hat{\mathbf{x}}\) 表示估计得到的误差状态修正量；\(\mathbf{K}\) 表示 Kalman 增益；\(\mathbf{r}\) 表示线性测量残差。

$$
\mathbf{P}^{+}
=
\mathbf{P}
-\mathbf{K}\mathbf{H}\mathbf{P}
$$

公式说明：该公式表示 EKF 更新后的协方差。当前项目会在状态辅助工具中维护变量顺序、边缘协方差和注入后的协方差一致性。

参数说明：\(\mathbf{P}^{+}\) 表示更新后的后验协方差；\(\mathbf{P}\) 表示更新前的先验协方差；\(\mathbf{K}\) 表示 Kalman 增益；\(\mathbf{H}\) 表示线性测量雅可比。

数值参数化例子：若一维情况下先验方差 \(P=0.04\)，测量雅可比 \(H=1\)，测量噪声方差 \(R=0.01\)，残差 \(r=0.10\)，则 \(S=0.05\)，\(K=0.8\)，修正量 \(\delta\hat{x}=0.08\)，更新后方差 \(P^+=0.008\)。

## 11. SLAM 特征更新

### 步骤 22：延迟初始化 SLAM 特征

SLAM 特征和 ArUco 特征会在通过三角化、重投影优化和一致性检验后加入状态，之后作为状态变量继续更新。

$$
\mathbf{r}
\simeq
\mathbf{H}_x\tilde{\mathbf{x}}
+\mathbf{H}_f\tilde{\boldsymbol{\lambda}}
+\mathbf{n}
$$

公式说明：该公式表示 SLAM 特征延迟初始化时的线性系统。与 MSCKF 临时特征不同，\(\tilde{\boldsymbol{\lambda}}\) 会进入状态协方差。

参数说明：\(\mathbf{r}\) 表示 SLAM 特征的堆叠视觉残差；\(\mathbf{H}_x\) 表示残差对已有状态误差的雅可比；\(\tilde{\mathbf{x}}\) 表示已有状态误差；\(\mathbf{H}_f\) 表示残差对新特征参数误差的雅可比；\(\tilde{\boldsymbol{\lambda}}\) 表示新特征参数误差；\(\mathbf{n}\) 表示视觉测量噪声。

$$
\mathbf{x}^{+}
=
\begin{bmatrix}
\mathbf{x}\\
\boldsymbol{\lambda}
\end{bmatrix}
$$

公式说明：该公式表示成功初始化 SLAM 特征后，滤波器状态会追加该特征参数。

参数说明：\(\mathbf{x}^{+}\) 表示扩展后的状态；\(\mathbf{x}\) 表示扩展前的状态；\(\boldsymbol{\lambda}\) 表示新加入的 SLAM 特征参数，可以是全局三维点、锚定三维点或锚定逆深度参数。

数值参数化例子：若当前状态维度为 110，初始化一个全局三维 SLAM 特征，则状态维度变为 113。若初始化的是单深度锚定逆深度特征，则只追加 1 维深度参数，但仍需要保留锚定 clone 的语义信息。

### 步骤 23：锚点变换

锚定表示的 SLAM 特征如果锚点 clone 即将被边缘化，项目会把特征变换到新的 anchor，以避免引用被删除的 clone。

$$
{}^{A_{\mathrm{new}}}\mathbf{p}_f
=
{}^{A_{\mathrm{new}}}_{A_{\mathrm{old}}}\mathbf{R}
{}^{A_{\mathrm{old}}}\mathbf{p}_f
+{}^{A_{\mathrm{new}}}\mathbf{p}_{A_{\mathrm{old}}}
$$

公式说明：该公式表示把旧 anchor 坐标系下的特征坐标转换到新 anchor 坐标系。

参数说明：\({}^{A_{\mathrm{new}}}\mathbf{p}_f\) 表示特征点在新 anchor 坐标系中的坐标；\({}^{A_{\mathrm{new}}}_{A_{\mathrm{old}}}\mathbf{R}\) 表示从旧 anchor 坐标系到新 anchor 坐标系的旋转；\({}^{A_{\mathrm{old}}}\mathbf{p}_f\) 表示特征点在旧 anchor 坐标系中的坐标；\({}^{A_{\mathrm{new}}}\mathbf{p}_{A_{\mathrm{old}}}\) 表示旧 anchor 原点在新 anchor 坐标系中的位置。

数值参数化例子：若旧 anchor 下特征为 \([0.5,0.1,4.0]^\top\,\mathrm{m}\)，新旧 anchor 姿态相同，新 anchor 相对旧 anchor 向前移动 \(0.2\,\mathrm{m}\)，则旧 anchor 原点在新 anchor 坐标中可近似为 \([-0.2,0,0]^\top\,\mathrm{m}\)，新 anchor 下特征约为 \([0.3,0.1,4.0]^\top\,\mathrm{m}\)。

## 12. clone 管理与边缘化

### 步骤 24：随机克隆

相机帧到来后，当前 IMU 位姿会被克隆到滑动窗口中，用于承载后续多帧视觉约束。

$$
\mathbf{x}_{C_k}
=
\begin{bmatrix}
{}^{I_k}_G\bar{\mathbf{q}}\\
{}^G\mathbf{p}_{I_k}
\end{bmatrix}
$$

公式说明：该公式表示第 \(k\) 个 clone 存储当前 IMU 姿态和位置，不包含速度和 bias。

参数说明：\(\mathbf{x}_{C_k}\) 表示第 \(k\) 个 clone 状态；\({}^{I_k}_G\bar{\mathbf{q}}\) 表示第 \(k\) 个图像时刻全局到 IMU 的姿态四元数；\({}^G\mathbf{p}_{I_k}\) 表示第 \(k\) 个图像时刻 IMU 在全局坐标系中的位置。

$$
\mathbf{P}^{+}
=
\mathbf{J}_{\mathrm{aug}}\mathbf{P}\mathbf{J}_{\mathrm{aug}}^{\top}
$$

公式说明：该公式表示克隆扩维时的协方差增广。新 clone 与当前 IMU 位姿保持交叉相关。

参数说明：\(\mathbf{P}^{+}\) 表示增广后的协方差；\(\mathbf{J}_{\mathrm{aug}}\) 表示从原状态到增广状态的雅可比；\(\mathbf{P}\) 表示增广前的协方差。

数值参数化例子：若当前协方差维度为 95，新增一个 6 维 clone 后维度变为 101。若 `max_clones=11`，第 12 个 clone 加入后系统会选择最旧 clone 作为边缘化对象。

## 13. ZUPT 零速更新

### 步骤 25：静止惯性残差

启用 ZUPT 时，系统会用 IMU 测量构造“角速度为零、加速度仅为重力”的静止约束，同时结合速度阈值和图像视差辅助判断。

$$
\mathbf{r}_{\omega}
=
-\left(\boldsymbol{\omega}_m-\mathbf{b}_g\right)
$$

公式说明：该公式表示零速检测中的角速度残差。若系统静止，去 bias 后角速度应接近零。

参数说明：\(\mathbf{r}_{\omega}\) 表示角速度残差；\(\boldsymbol{\omega}_m\) 表示陀螺仪测量；\(\mathbf{b}_g\) 表示陀螺仪 bias。

$$
\mathbf{r}_{a}
=
-\left(\mathbf{a}_m-\mathbf{b}_a-{}^I_G\mathbf{R}{}^G\mathbf{g}\right)
$$

公式说明：该公式表示零速检测中的加速度残差。若系统静止，去 bias 后的加速度计读数应接近重力在 IMU 坐标系中的表达。

参数说明：\(\mathbf{r}_{a}\) 表示加速度残差；\(\mathbf{a}_m\) 表示加速度计测量；\(\mathbf{b}_a\) 表示加速度计 bias；\({}^I_G\mathbf{R}\) 表示从全局坐标系到 IMU 坐标系的旋转；\({}^G\mathbf{g}\) 表示全局重力向量。

$$
\mathbf{r}_{\mathrm{zupt}}^{\top}
\left(\mathbf{H}_{\mathrm{zupt}}\mathbf{P}\mathbf{H}_{\mathrm{zupt}}^{\top}
+\alpha\mathbf{R}_{\mathrm{zupt}}\right)^{-1}
\mathbf{r}_{\mathrm{zupt}}
<
\gamma\chi^2_{\nu,0.95}
$$

公式说明：该公式表示 ZUPT 的卡方接受条件。当前项目还会检查速度范数是否低于 `zupt_max_velocity`，并可用图像平均视差覆盖 IMU 判断。

参数说明：\(\mathbf{r}_{\mathrm{zupt}}\) 表示堆叠后的 ZUPT 残差；\(\mathbf{H}_{\mathrm{zupt}}\) 表示 ZUPT 残差雅可比；\(\mathbf{P}\) 表示相关状态边缘协方差；\(\alpha\) 表示 ZUPT 噪声放大倍率，对应 `zupt_noise_multiplier`；\(\mathbf{R}_{\mathrm{zupt}}\) 表示 ZUPT 测量噪声协方差；\(\gamma\) 表示 ZUPT 卡方门限倍率；\(\chi^2_{\nu,0.95}\) 表示自由度为 \(\nu\)、0.95 置信度的卡方分位数。

数值参数化例子：若静止时去 bias 后角速度约为 \([0.002,-0.001,0.001]^\top\,\mathrm{rad/s}\)，加速度残差约为 \([0.03,-0.02,0.04]^\top\,\mathrm{m/s^2}\)，速度范数 \(0.05\,\mathrm{m/s}\) 小于默认 `zupt_max_velocity=1.0`，平均视差 \(0.4\) 小于默认 `zupt_max_disparity=1.0`，则 ZUPT 很可能被接受。若速度范数 \(2.0\,\mathrm{m/s}\)，即使加速度变化小，也会被速度阈值拒绝。

### 步骤 26：视差辅助静止判定

ZUPT 还可检查连续图像中的平均特征位移，低视差常表示平台静止或近似静止。

$$
\frac{1}{N}
\sum_{i=1}^{N}
\left\|
\mathbf{u}_{k,i}-\mathbf{u}_{k-1,i}
\right\|
<
\Delta d
$$

公式说明：该公式表示图像视差静止判定。当前项目要求平均视差低于阈值并且有足够特征数，才认为视觉证据支持静止。

参数说明：\(N\) 表示连续两帧共有的特征数量；\(\mathbf{u}_{k,i}\) 表示第 \(i\) 个特征在当前帧的像素坐标；\(\mathbf{u}_{k-1,i}\) 表示第 \(i\) 个特征在上一帧的像素坐标；\(\|\cdot\|\) 表示欧氏距离；\(\Delta d\) 表示平均视差阈值，对应 `zupt_max_disparity`。

数值参数化例子：若 50 个跟踪特征的平均像素位移为 \(0.35\) 像素，\(\Delta d=1.0\) 像素，则视差判定通过。若平均像素位移为 \(3.2\) 像素，则视差判定不支持静止。

## 14. FEJ 一致性

### 步骤 27：第一估计雅可比

VINS 存在全局平移和绕重力方向 yaw 的不可观方向。FEJ 的作用是使用变量第一次被线性化时的估计值计算雅可比，减少虚假信息增益。

$$
\mathbf{H}_{k}\mathbf{\Phi}_{k,0}\mathcal{N}
=
\mathbf{0}
$$

公式说明：该公式表示不可观子空间 \(\mathcal{N}\) 应处在整体线性化系统的零空间中。若线性化点随更新随意改变，滤波器可能错误地把不可观方向当成可观方向。

参数说明：\(\mathbf{H}_{k}\) 表示第 \(k\) 次测量雅可比；\(\mathbf{\Phi}_{k,0}\) 表示从初始时刻到第 \(k\) 时刻的状态转移矩阵；\(\mathcal{N}\) 表示 VINS 不可观子空间；\(\mathbf{0}\) 表示零矩阵或零向量。

$$
\mathbf{J}_{\mathrm{meas}}
=
\left.
\frac{\partial \mathbf{h}(\mathbf{x})}{\partial \mathbf{x}}
\right|_{\mathbf{x}=\mathbf{x}_{\mathrm{FEJ}}}
$$

公式说明：该公式表示测量雅可比在第一估计点处求导。当前项目默认 `use_fej=true`，clone、特征和部分标定量会保存 FEJ 值用于雅可比计算。

参数说明：\(\mathbf{J}_{\mathrm{meas}}\) 表示测量雅可比；\(\mathbf{h}(\mathbf{x})\) 表示非线性测量函数；\(\mathbf{x}\) 表示状态；\(\mathbf{x}_{\mathrm{FEJ}}\) 表示变量第一次进入滤波器或第一次线性化时保存的估计值。

数值参数化例子：若某 clone 初次加入时 yaw 为 \(5.0^\circ\)，后续 EKF 更新把当前估计修正到 \(5.3^\circ\)。开启 FEJ 后，涉及该 clone 的历史视觉雅可比仍可使用 \(5.0^\circ\) 处的线性化值，从而保持历史约束的一致性。

## 15. 当前项目默认参数与算法含义

### 步骤 28：主要估计参数

当前仓库的参数解析显示，核心估计相关默认值如下。实际运行时以具体 `config/*/estimator_config.yaml` 和外部 IMU / 相机标定 YAML 为准。

$$
\sigma_{\mathrm{pix}}^2
=
\sigma_{\mathrm{pix}}\sigma_{\mathrm{pix}}
$$

公式说明：该公式表示视觉像素噪声方差由像素噪声标准差平方得到。MSCKF、SLAM 和 ArUco 更新各自有独立的像素噪声参数。

参数说明：\(\sigma_{\mathrm{pix}}^2\) 表示像素测量噪声方差；\(\sigma_{\mathrm{pix}}\) 表示像素测量噪声标准差。

数值参数化例子：若 `up_msckf_sigma_px=1.0`，则 MSCKF 视觉更新的单个像素坐标噪声方差为 \(1.0^2=1.0\)。若某相机跟踪更不稳定，把该值调为 \(2.0\)，噪声方差变为 \(4.0\)，视觉更新权重会降低。

当前项目中值得关注的参数含义：

- `use_fej=true`：默认使用第一估计雅可比。
- `integration=rk4`：默认使用 RK4 形式传播 IMU 名义状态。
- `max_clones=11`：滑动窗口最多保留 11 个 clone。
- `max_slam=25`：默认最多保留 25 个 SLAM 特征。
- `max_msckf_in_update=1000`：单次图像时刻最多使用 1000 个 MSCKF 特征。
- `calib_cam_extrinsics`、`calib_cam_intrinsics`、`calib_cam_timeoffset`：分别控制相机外参、内参和时间偏移是否在线标定。
- `calib_imu_intrinsics`、`calib_imu_g_sensitivity`：分别控制 IMU 内参和陀螺仪重力敏感性是否在线标定。
- `try_zupt`、`zupt_max_velocity`、`zupt_noise_multiplier`、`zupt_max_disparity`：控制零速更新启用和判定阈值。

数值参数化例子：若运行 EuRoC 双目数据，`max_clones=11` 表示大约保留最近 11 个图像时刻的 IMU 位姿；若相机频率为 \(20\,\mathrm{Hz}\)，滑窗时间跨度约为 \(10/20=0.5\,\mathrm{s}\)。若 `max_slam=25`，最多有 25 个长期地图点参与状态估计，普通短生命周期角点主要通过 MSCKF 更新提供约束。

## 16. 一次完整图像更新的数值化流程

### 步骤 29：从图像到状态更新

假设当前相机时间为 \(20.000\,\mathrm{s}\)，上一状态时间为 \(19.950\,\mathrm{s}\)，IMU 频率为 \(200\,\mathrm{Hz}\)，系统大致执行以下流程。

$$
N_{\mathrm{imu}}
=
\frac{t_k-t_{k-1}}{\Delta t_{\mathrm{imu}}}
$$

公式说明：该公式估算一次相机间隔内需要处理的 IMU 小区间数量。

参数说明：\(N_{\mathrm{imu}}\) 表示 IMU 积分小区间数量；\(t_k\) 表示当前相机帧时间；\(t_{k-1}\) 表示上一相机帧时间；\(\Delta t_{\mathrm{imu}}\) 表示 IMU 采样周期。

数值参数化例子：若 \(t_k=20.000\,\mathrm{s}\)、\(t_{k-1}=19.950\,\mathrm{s}\)、\(\Delta t_{\mathrm{imu}}=0.005\,\mathrm{s}\)，则 \(N_{\mathrm{imu}}=10\)。系统会在这 10 个 IMU 区间内传播状态和协方差。

一次完整图像更新可以按以下数值化步骤理解：

1. 传播：处理约 10 个 IMU 小区间，把状态传播到 \(20.000\,\mathrm{s}\)。
2. 克隆：把当前 IMU 姿态和位置作为新 clone 加入窗口。
3. 跟踪：假设当前图像跟踪到 180 个普通角点和 0 个 ArUco tag。
4. MSCKF 筛选：假设 120 个角点拥有足够跨帧观测，其中 90 个通过三角化和卡方检验。
5. 零空间投影：若平均每个通过特征有 5 次观测，每个特征从 10 维残差降到 7 维残差，理论累计约 \(90\times7=630\) 维残差。
6. 测量压缩：若参与状态维度为 90，压缩后残差行数最多约 90。
7. EKF 更新：用压缩后的残差和雅可比更新当前 IMU 状态、clone、标定量和已有 SLAM 特征。
8. SLAM 延迟初始化：若已过 `dt_slam_delay=2.0\,\mathrm{s}`，且状态中 SLAM 特征少于 25，则从稳定特征中初始化新的 SLAM 特征。
9. 边缘化：若 clone 数超过 11，最旧 clone 被边缘化；若锚定 SLAM 特征依赖最旧 clone，会先执行锚点变换。


## 17. 从初始化到最终位姿输出的完整算例

本节给出一个端到端数值例子，展示 OpenVINS 从静止初始化、第一帧 clone、IMU 传播、视觉更新到最终输出位姿的计算链路。例子采用简化二维运动趋势和三维 VIO 状态表达，数值用于理解计算顺序，实际项目会在完整三维矩阵和多特征系统中执行同样流程。

### 步骤 30：输入数据与标定假设

假设系统使用一个双目或单目相机与一个 IMU，IMU 频率为 200 Hz，相机频率为 20 Hz，重力大小为 9.81。为了便于计算，令相机和 IMU 的时间偏移为 0，相机外参接近单位姿态，并给出一个小平移。

$$
{}^C\bar{\boldsymbol{q}}_I=
\begin{bmatrix}
0\\0\\0\\1
\end{bmatrix},
\qquad
{}^C\mathbf{p}_I=
\begin{bmatrix}
0.05\\0\\0
\end{bmatrix}\,\mathrm{m}
$$

公式说明：该公式表示例子中 IMU 到相机的外参。姿态外参为单位四元数，平移外参表示 IMU 原点在相机坐标系中向相机 x 轴正方向偏移 0.05 m。

参数说明：\({}^C\bar{\boldsymbol{q}}_I\) 表示从 IMU 坐标系到相机坐标系的旋转四元数；\({}^C\mathbf{p}_I\) 表示 IMU 原点在相机坐标系中的位置；\(\mathrm{m}\) 表示米；\([0,0,0,1]^\top\) 表示单位四元数。

$$
{}^G\mathbf{g}=\begin{bmatrix}0\\0\\9.81\end{bmatrix}\,\mathrm{m/s^2},
\qquad
\Delta t_{\mathrm{imu}}=0.005\,\mathrm{s},
\qquad
\Delta t_{\mathrm{cam}}=0.05\,\mathrm{s}
$$

公式说明：该公式给出例子中的重力、IMU 采样周期和相机采样周期。

参数说明：\({}^G\mathbf{g}\) 表示全局坐标系中的重力向量；\(\Delta t_{\mathrm{imu}}\) 表示 IMU 采样周期；\(\Delta t_{\mathrm{cam}}\) 表示相机采样周期；\(\mathrm{m/s^2}\) 表示米每二次方秒；\(\mathrm{s}\) 表示秒。

数值参数化例子：IMU 频率为 \(1/0.005=200\,\mathrm{Hz}\)，相机频率为 \(1/0.05=20\,\mathrm{Hz}\)。相邻两帧图像之间有 \(0.05/0.005=10\) 个 IMU 积分小区间。

### 步骤 31：静止初始化估计姿态、bias、速度和位置

初始化阶段通常收集一小段 IMU 和图像数据。静止或弱运动初始化时，加速度计平均值用于确定 roll / pitch，陀螺仪平均值用于估计初始陀螺仪 bias，初始速度和位置可设为零或由初始化器估计。

$$
\bar{\boldsymbol{\omega}}_m
=
\frac{1}{N}\sum_{i=1}^{N}\boldsymbol{\omega}_{m,i}
$$

公式说明：该公式表示初始化窗口内陀螺仪测量的平均值。静止初始化时，该平均值可作为陀螺仪 bias 的初始估计。

参数说明：\(\bar{\boldsymbol{\omega}}_m\) 表示陀螺仪测量均值；\(N\) 表示初始化窗口中的 IMU 样本数；\(\boldsymbol{\omega}_{m,i}\) 表示第 \(i\) 个陀螺仪测量。

$$
\bar{\mathbf{a}}_m
=
\frac{1}{N}\sum_{i=1}^{N}\mathbf{a}_{m,i}
$$

公式说明：该公式表示初始化窗口内加速度计测量的平均值。静止初始化时，该平均值应接近重力在 IMU 坐标系中的表达加上加速度计 bias。

参数说明：\(\bar{\mathbf{a}}_m\) 表示加速度计测量均值；\(N\) 表示初始化窗口中的 IMU 样本数；\(\mathbf{a}_{m,i}\) 表示第 \(i\) 个加速度计测量。

$$
\hat{\mathbf{b}}_{g,0}=\bar{\boldsymbol{\omega}}_m,
\qquad
\hat{\mathbf{b}}_{a,0}=\bar{\mathbf{a}}_m-{}^{I_0}_G\hat{\mathbf{R}}{}^G\mathbf{g}
$$

公式说明：该公式表示一个简化静止初始化中的 bias 估计。陀螺仪 bias 由角速度均值给出，加速度计 bias 由加速度均值减去重力在 IMU 坐标系下的表达得到。

参数说明：\(\hat{\mathbf{b}}_{g,0}\) 表示初始陀螺仪 bias 估计；\(\bar{\boldsymbol{\omega}}_m\) 表示陀螺仪测量均值；\(\hat{\mathbf{b}}_{a,0}\) 表示初始加速度计 bias 估计；\(\bar{\mathbf{a}}_m\) 表示加速度计测量均值；\({}^{I_0}_G\hat{\mathbf{R}}\) 表示初始化时全局到 IMU 的旋转估计；\({}^G\mathbf{g}\) 表示全局重力向量。

数值参数化例子：若初始化窗口 \(N=200\)，陀螺仪均值为 \([0.001,-0.002,0.0005]^\top\,\mathrm{rad/s}\)，加速度计均值为 \([0.02,-0.01,9.84]^\top\,\mathrm{m/s^2}\)，初始姿态近似水平，则 \(\hat{\mathbf{b}}_{g,0}=[0.001,-0.002,0.0005]^\top\,\mathrm{rad/s}\)，\(\hat{\mathbf{b}}_{a,0}=[0.02,-0.01,0.03]^\top\,\mathrm{m/s^2}\)。初始状态可设为 \({}^G\hat{\mathbf{p}}_{I_0}=[0,0,0]^\top\,\mathrm{m}\)，\({}^G\hat{\mathbf{v}}_{I_0}=[0,0,0]^\top\,\mathrm{m/s}\)。

### 步骤 32：第一帧图像到来并创建第一个 clone

当第一帧可用于滤波状态后，系统把当前 IMU 姿态和位置克隆到滑动窗口。该 clone 后续会作为视觉多帧约束的状态变量。

$$
\mathbf{x}_{C_0}=
\begin{bmatrix}
{}^{I_0}_G\hat{\bar{\mathbf{q}}}\\
{}^G\hat{\mathbf{p}}_{I_0}
\end{bmatrix}
$$

公式说明：该公式表示第一帧图像时间对应的 clone 状态。clone 只保存姿态和位置，不保存速度与 bias。

参数说明：\(\mathbf{x}_{C_0}\) 表示第一帧对应的 clone 状态；\({}^{I_0}_G\hat{\bar{\mathbf{q}}}\) 表示第一帧时刻全局到 IMU 的姿态四元数估计；\({}^G\hat{\mathbf{p}}_{I_0}\) 表示第一帧时刻 IMU 在全局坐标系中的位置估计。

$$
\mathbf{P}_{C_0,I_0}=\mathbf{J}_{C_0}\mathbf{P}_{I_0,I_0}
$$

公式说明：该公式表示 clone 与当前 IMU 状态之间的交叉协方差来自当前 IMU 状态协方差和增广雅可比。

参数说明：\(\mathbf{P}_{C_0,I_0}\) 表示 clone \(C_0\) 与当前 IMU 状态 \(I_0\) 的交叉协方差；\(\mathbf{J}_{C_0}\) 表示 clone 增广对当前 IMU 状态的雅可比；\(\mathbf{P}_{I_0,I_0}\) 表示当前 IMU 状态自身协方差。

数值参数化例子：若初始化后姿态标准差约为 \(1^\circ\)，位置标准差约为 \(0.10\,\mathrm{m}\)，则第一个 clone 会继承这部分不确定性。此时窗口内 clone 数为 1，低于默认最大值 11，不触发边缘化。

### 步骤 33：第二帧前的 IMU 传播

第二帧相机时间为 \(t_1=0.05\,\mathrm{s}\)。在 \([0,0.05]\) 内有 10 个 IMU 小区间。假设去 bias 后平均角速度为 \([0,0,0.02]^\top\,\mathrm{rad/s}\)，去 bias 后平均加速度为 \([0.40,0,9.81]^\top\,\mathrm{m/s^2}\)，姿态仍接近水平。

$$
\Delta\boldsymbol{\theta}
=
\hat{\boldsymbol{\omega}}\Delta t_{\mathrm{cam}}
$$

公式说明：该公式表示相邻两帧之间由平均角速度近似得到的旋转向量增量。

参数说明：\(\Delta\boldsymbol{\theta}\) 表示相机帧间姿态旋转向量增量；\(\hat{\boldsymbol{\omega}}\) 表示去 bias 后的平均角速度；\(\Delta t_{\mathrm{cam}}\) 表示相邻相机帧时间间隔。

$$
{}^G\hat{\mathbf{v}}_{I_1}
=
{}^G\hat{\mathbf{v}}_{I_0}
+
\left({}^{I_0}_G\hat{\mathbf{R}}^\top\hat{\mathbf{a}}-{}^G\mathbf{g}\right)\Delta t_{\mathrm{cam}}
$$

公式说明：该公式表示从第一帧到第二帧的速度传播。去 bias 加速度旋转到全局后减去重力，再对时间积分。

参数说明：\({}^G\hat{\mathbf{v}}_{I_1}\) 表示第二帧时刻 IMU 全局速度估计；\({}^G\hat{\mathbf{v}}_{I_0}\) 表示第一帧时刻 IMU 全局速度估计；\({}^{I_0}_G\hat{\mathbf{R}}^\top\) 表示第一帧时刻从 IMU 到全局的旋转；\(\hat{\mathbf{a}}\) 表示去 bias 后的平均加速度；\({}^G\mathbf{g}\) 表示全局重力向量；\(\Delta t_{\mathrm{cam}}\) 表示相邻相机帧时间间隔。

$$
{}^G\hat{\mathbf{p}}_{I_1}
=
{}^G\hat{\mathbf{p}}_{I_0}
+{}^G\hat{\mathbf{v}}_{I_0}\Delta t_{\mathrm{cam}}
+
\frac{1}{2}
\left({}^{I_0}_G\hat{\mathbf{R}}^\top\hat{\mathbf{a}}-{}^G\mathbf{g}\right)
\Delta t_{\mathrm{cam}}^2
$$

公式说明：该公式表示从第一帧到第二帧的位置传播。该式使用平均加速度近似，实际项目默认可用 RK4 在多个 IMU 小区间内积分。

参数说明：\({}^G\hat{\mathbf{p}}_{I_1}\) 表示第二帧时刻 IMU 全局位置估计；\({}^G\hat{\mathbf{p}}_{I_0}\) 表示第一帧时刻 IMU 全局位置估计；\({}^G\hat{\mathbf{v}}_{I_0}\) 表示第一帧时刻 IMU 全局速度估计；\(\Delta t_{\mathrm{cam}}\) 表示相邻相机帧时间间隔；\({}^{I_0}_G\hat{\mathbf{R}}^\top\) 表示第一帧时刻从 IMU 到全局的旋转；\(\hat{\mathbf{a}}\) 表示去 bias 后的平均加速度；\({}^G\mathbf{g}\) 表示全局重力向量。

数值参数化例子：\(\Delta\boldsymbol{\theta}=[0,0,0.02]^\top\times0.05=[0,0,0.001]^\top\,\mathrm{rad}\)。若 \({}^G\hat{\mathbf{v}}_{I_0}=0\)、\({}^G\hat{\mathbf{p}}_{I_0}=0\)、\({}^{I_0}_G\hat{\mathbf{R}}=\mathbf{I}_3\)，并按简化平均加速度模型把 10 个 IMU 小区间合并为总时间 \(\Delta t_{\mathrm{cam}}=0.05\,\mathrm{s}\)，则先计算净全局加速度：

$$
\mathbf{a}_{\mathrm{net}}
=
{}^{I_0}_G\hat{\mathbf{R}}^\top\hat{\mathbf{a}}-{}^G\mathbf{g}
=
\begin{bmatrix}0.40\\0\\9.81\end{bmatrix}
-
\begin{bmatrix}0\\0\\9.81\end{bmatrix}
=
\begin{bmatrix}0.40\\0\\0\end{bmatrix}
\,\mathrm{m/s^2}
$$

速度由加速度一阶积分得到：

$$
{}^G\hat{\mathbf{v}}_{I_1}
=
{}^G\hat{\mathbf{v}}_{I_0}
+\mathbf{a}_{\mathrm{net}}\Delta t_{\mathrm{cam}}
=
\begin{bmatrix}0\\0\\0\end{bmatrix}
+
\begin{bmatrix}0.40\\0\\0\end{bmatrix}\times0.05
=
\begin{bmatrix}0.02\\0\\0\end{bmatrix}
\,\mathrm{m/s}
$$

位置由初始速度项和加速度二阶积分得到；由于本例初始速度为零，只剩 \(\frac{1}{2}\mathbf{a}_{\mathrm{net}}\Delta t_{\mathrm{cam}}^2\)：

$$
{}^G\hat{\mathbf{p}}_{I_1}
=
{}^G\hat{\mathbf{p}}_{I_0}
+{}^G\hat{\mathbf{v}}_{I_0}\Delta t_{\mathrm{cam}}
+\frac{1}{2}\mathbf{a}_{\mathrm{net}}\Delta t_{\mathrm{cam}}^2
=
\frac{1}{2}
\begin{bmatrix}0.40\\0\\0\end{bmatrix}
\times0.05^2
=
\begin{bmatrix}0.0005\\0\\0\end{bmatrix}
\,\mathrm{m}
$$

### 步骤 34：第二帧创建 clone 并形成特征观测

传播到 \(t_1\) 后，系统创建第二个 clone。假设一个特征在第一帧和第二帧都被观测到，并且已知归一化观测坐标。

$$
\mathbf{x}_{C_1}=
\begin{bmatrix}
{}^{I_1}_G\hat{\bar{\mathbf{q}}}\\
{}^G\hat{\mathbf{p}}_{I_1}
\end{bmatrix}
$$

公式说明：该公式表示第二帧图像时间对应的 clone 状态。该 clone 和第一帧 clone 共同参与跨帧特征约束。

参数说明：\(\mathbf{x}_{C_1}\) 表示第二帧对应的 clone 状态；\({}^{I_1}_G\hat{\bar{\mathbf{q}}}\) 表示第二帧时刻全局到 IMU 的姿态四元数估计；\({}^G\hat{\mathbf{p}}_{I_1}\) 表示第二帧时刻 IMU 在全局坐标系中的位置估计。

$$
\mathbf{z}_{n,0}=\begin{bmatrix}0.125\\0.025\end{bmatrix},
\qquad
\mathbf{z}_{n,1}=\begin{bmatrix}0.123\\0.025\end{bmatrix}
$$

公式说明：该公式表示同一个特征在两帧中的归一化图像观测。第二帧横向坐标略小，表示相机向前或侧向运动后产生了少量视差。

参数说明：\(\mathbf{z}_{n,0}\) 表示第一帧归一化观测；\(\mathbf{z}_{n,1}\) 表示第二帧归一化观测；第一个分量表示归一化横坐标；第二个分量表示归一化纵坐标。

数值参数化例子：若焦距 \(f_x=f_y=460\)，第一帧归一化观测 \([0.125,0.025]^\top\) 对应像素偏移约为 \([57.5,11.5]^\top\) 像素；第二帧横向归一化坐标减少 0.002，对应约 \(0.92\) 像素的横向视差。

### 步骤 35：三角化得到临时特征点

用两个或多个 clone 的相机位姿和归一化观测，可以三角化特征点。为了让例子可读，假设三角化和一次重投影优化后得到 anchor 相机坐标系中的特征点。

$$
{}^{C_0}\hat{\mathbf{p}}_f
=
\begin{bmatrix}
0.50\\0.10\\4.00
\end{bmatrix}\,\mathrm{m}
$$

公式说明：该公式表示特征点在第一帧 anchor 相机坐标系中的三维坐标估计。该坐标来自线性三角化和后续重投影优化。

参数说明：\({}^{C_0}\hat{\mathbf{p}}_f\) 表示特征点在第一帧相机坐标系中的坐标估计；第一个分量表示相机 x 方向位置；第二个分量表示相机 y 方向位置；第三个分量表示深度；\(\mathrm{m}\) 表示米。

$$
\hat{\alpha}=\frac{{}^{C_0}\hat{x}_f}{{}^{C_0}\hat{z}_f},
\qquad
\hat{\beta}=\frac{{}^{C_0}\hat{y}_f}{{}^{C_0}\hat{z}_f},
\qquad
\hat{\rho}=\frac{1}{{}^{C_0}\hat{z}_f}
$$

公式说明：该公式表示把 anchor 相机坐标中的三维点转换为 MSCKF 风格的锚定逆深度参数。

参数说明：\(\hat{\alpha}\) 表示估计的归一化横向 bearing 参数；\({}^{C_0}\hat{x}_f\) 表示特征点在第一帧相机坐标系中的 x 坐标；\({}^{C_0}\hat{z}_f\) 表示特征点在第一帧相机坐标系中的 z 深度；\(\hat{\beta}\) 表示估计的归一化纵向 bearing 参数；\({}^{C_0}\hat{y}_f\) 表示特征点在第一帧相机坐标系中的 y 坐标；\(\hat{\rho}\) 表示估计的逆深度。

数值参数化例子：由 \({}^{C_0}\hat{\mathbf{p}}_f=[0.50,0.10,4.00]^\top\) 得到 \(\hat{\alpha}=0.125\)、\(\hat{\beta}=0.025\)、\(\hat{\rho}=0.25\)。该深度 \(4.00\,\mathrm{m}\) 位于默认 `min_dist=0.10` 和 `max_dist=60` 之间，可通过距离检查。

### 步骤 36：构造 MSCKF 残差并做零空间投影

假设该特征在 4 个 clone 中被观测到，因此原始残差维度为 \(2\times4=8\)。三维特征被零空间投影消除后，剩余 \(8-3=5\) 维约束进入 EKF。

$$
\mathbf{r}_{\mathrm{raw}}
=
\begin{bmatrix}
0.20\\-0.10\\0.10\\0.05\\-0.05\\0.02\\0.08\\-0.04
\end{bmatrix}\,\mathrm{px}
$$

公式说明：该公式表示一个特征在 4 次观测上的原始像素残差堆叠。每两维对应一帧图像的横向和纵向残差。

参数说明：\(\mathbf{r}_{\mathrm{raw}}\) 表示原始视觉残差向量；\(\mathrm{px}\) 表示像素；向量中的每两个连续分量对应一个二维像素观测残差。

$$
\mathbf{r}_{o}=\mathbf{Q}_2^\top\mathbf{r}_{\mathrm{raw}},
\qquad
\mathbf{H}_{o}=\mathbf{Q}_2^\top\mathbf{H}_{x}
$$

公式说明：该公式表示 MSCKF 左零空间投影后的残差和状态雅可比。投影后特征参数误差被消除。

参数说明：\(\mathbf{r}_{o}\) 表示零空间投影后的残差；\(\mathbf{Q}_2\) 表示特征雅可比左零空间基；\(\mathbf{r}_{\mathrm{raw}}\) 表示原始视觉残差；\(\mathbf{H}_{o}\) 表示零空间投影后的状态雅可比；\(\mathbf{H}_{x}\) 表示原始状态雅可比。

数值参数化例子：若 \(\mathbf{r}_{\mathrm{raw}}\) 是 8 维，\(\mathbf{H}_f\) 是 \(8\times3\) 且满秩，则 \(\mathbf{Q}_2^\top\) 是 \(5\times8\)，\(\mathbf{r}_o\) 是 5 维。若投影后卡方统计量为 \(\chi^2=2.4\)，自由度为 5 的 0.95 分位数约为 11.07，默认倍率 5 给出门限 55.35，因此该特征通过检验。

### 步骤 37：EKF 更新并把误差注入状态

多个通过检验的 MSCKF 特征会合并、压缩并进入 EKF 更新。这里用一个简化的更新量说明状态如何被修正。

$$
\delta\hat{\mathbf{x}}
=\mathbf{K}\mathbf{r}
$$

公式说明：该公式表示 EKF 根据 Kalman 增益和残差得到误差状态修正量。真实项目中 \(\delta\hat{\mathbf{x}}\) 会包含姿态、位置、速度、bias、clone、标定和特征等相关变量的误差。

参数说明：\(\delta\hat{\mathbf{x}}\) 表示误差状态修正量；\(\mathbf{K}\) 表示 Kalman 增益；\(\mathbf{r}\) 表示压缩后的总残差。

$$
{}^G\hat{\mathbf{p}}_{I_1}^{+}
=
{}^G\hat{\mathbf{p}}_{I_1}^{-}
+
{}^G\delta\hat{\mathbf{p}}_{I_1}
$$

公式说明：该公式表示位置误差修正注入当前 IMU 名义位置。上标 \(-\) 表示更新前先验，上标 \(+\) 表示更新后后验。

参数说明：\({}^G\hat{\mathbf{p}}_{I_1}^{+}\) 表示更新后的第二帧 IMU 全局位置估计；\({}^G\hat{\mathbf{p}}_{I_1}^{-}\) 表示更新前的第二帧 IMU 全局位置估计；\({}^G\delta\hat{\mathbf{p}}_{I_1}\) 表示 EKF 求得的 IMU 位置修正量。

$$
{}^{I_1}_G\hat{\bar{\mathbf{q}}}^{+}
=
\delta\hat{\bar{\mathbf{q}}}\otimes{}^{I_1}_G\hat{\bar{\mathbf{q}}}^{-}
$$

公式说明：该公式表示姿态误差修正通过左乘四元数注入当前 IMU 名义姿态。

参数说明：\({}^{I_1}_G\hat{\bar{\mathbf{q}}}^{+}\) 表示更新后的全局到 IMU 姿态四元数；\(\delta\hat{\bar{\mathbf{q}}}\) 表示由小角度误差构造的姿态误差四元数；\(\otimes\) 表示四元数乘法；\({}^{I_1}_G\hat{\bar{\mathbf{q}}}^{-}\) 表示更新前的全局到 IMU 姿态四元数。

数值参数化例子：传播先验位置为 \({}^G\hat{\mathbf{p}}_{I_1}^{-}=[0.0005,0,0]^\top\,\mathrm{m}\)，EKF 位置修正为 \({}^G\delta\hat{\mathbf{p}}_{I_1}=[-0.0001,0.0002,0]^\top\,\mathrm{m}\)，则更新后位置为 \({}^G\hat{\mathbf{p}}_{I_1}^{+}=[0.0004,0.0002,0]^\top\,\mathrm{m}\)。若传播姿态 yaw 约为 \(0.001\,\mathrm{rad}\)，EKF 姿态小角修正为 \([0,0,-0.0002]^\top\,\mathrm{rad}\)，则更新后 yaw 约为 \(0.0008\,\mathrm{rad}\)。

### 步骤 38：输出最终 IMU 位姿和相机位姿

OpenVINS 输出通常以当前 IMU 状态为核心，也可以通过相机外参换算当前相机位姿。若可视化或 ROS 输出需要相机位姿，可由 IMU 位姿和外参组合得到。

$$
{}^G\mathbf{p}_{C_1}
=
{}^G\hat{\mathbf{p}}_{I_1}^{+}
-
\left({}^{C}_{I}\hat{\mathbf{R}}{}^{I_1}_{G}\hat{\mathbf{R}}^{+}\right)^\top
{}^C\mathbf{p}_I
$$

公式说明：该公式表示由更新后的 IMU 位姿和 IMU 到相机外参计算当前相机在全局坐标系中的位置。该式与项目中构造 clone 相机位姿的形式一致。

参数说明：\({}^G\mathbf{p}_{C_1}\) 表示第二帧相机原点在全局坐标系中的位置；\({}^G\hat{\mathbf{p}}_{I_1}^{+}\) 表示更新后的第二帧 IMU 全局位置；\({}^{C}_{I}\hat{\mathbf{R}}\) 表示 IMU 到相机的旋转外参；\({}^{I_1}_{G}\hat{\mathbf{R}}^{+}\) 表示更新后的全局到 IMU 旋转；\({}^C\mathbf{p}_I\) 表示 IMU 原点在相机坐标系中的位置。

$$
{}^{C_1}_{G}\hat{\mathbf{R}}
=
{}^{C}_{I}\hat{\mathbf{R}}{}^{I_1}_{G}\hat{\mathbf{R}}^{+}
$$

公式说明：该公式表示由更新后的 IMU 姿态和 IMU 到相机旋转外参计算当前相机姿态。

参数说明：\({}^{C_1}_{G}\hat{\mathbf{R}}\) 表示第二帧相机坐标系相对全局坐标系的旋转；\({}^{C}_{I}\hat{\mathbf{R}}\) 表示 IMU 到相机的旋转外参；\({}^{I_1}_{G}\hat{\mathbf{R}}^{+}\) 表示更新后的全局到 IMU 旋转。

$$
\mathbf{T}^{G}_{I_1}
=
\begin{bmatrix}
{}^G_{I_1}\hat{\mathbf{R}}^{+} & {}^G\hat{\mathbf{p}}_{I_1}^{+}\\
\mathbf{0}_{1\times3} & 1
\end{bmatrix}
$$

公式说明：该公式表示最终可输出的 IMU 位姿齐次变换矩阵。该矩阵把 IMU 坐标系下的点变换到全局坐标系。

参数说明：\(\mathbf{T}^{G}_{I_1}\) 表示从 IMU 坐标系到全局坐标系的齐次变换；\({}^G_{I_1}\hat{\mathbf{R}}^{+}\) 表示更新后的 IMU 到全局旋转；\({}^G\hat{\mathbf{p}}_{I_1}^{+}\) 表示更新后的 IMU 全局位置；\(\mathbf{0}_{1\times3}\) 表示一行三列零向量；\(1\) 表示齐次矩阵右下角元素。

数值参数化例子：若更新后 \({}^G\hat{\mathbf{p}}_{I_1}^{+}=[0.0004,0.0002,0]^\top\,\mathrm{m}\)，更新后 yaw 约为 \(0.0008\,\mathrm{rad}\)，外参 \({}^C\mathbf{p}_I=[0.05,0,0]^\top\,\mathrm{m}\)，且 \({}^{C}_{I}\mathbf{R}=\mathbf{I}_3\)，则相机全局位置约为 \({}^G\mathbf{p}_{C_1}=[-0.0496,0.00016,0]^\top\,\mathrm{m}\)。最终对外发布的 IMU 位姿可理解为位置 \([0.0004,0.0002,0]^\top\,\mathrm{m}\)、姿态 yaw \(0.0008\,\mathrm{rad}\)、速度 \([0.02,0,0]^\top\,\mathrm{m/s}\) 附近的后验状态。

### 步骤 39：继续运行到最终轨迹

每来一帧图像，系统重复传播、clone、视觉更新、SLAM 延迟初始化和边缘化。最终轨迹就是每个输出时刻的后验 IMU 位姿序列。

$$
\mathcal{T}
=
\left\{
\mathbf{T}^{G}_{I_0},
\mathbf{T}^{G}_{I_1},
\ldots,
\mathbf{T}^{G}_{I_K}
\right\}
$$

公式说明：该公式表示完整 VIO 输出轨迹由多个时刻的 IMU 位姿组成。实际 ROS 可视化、轨迹保存和评估会使用这些位姿及其时间戳。

参数说明：\(\mathcal{T}\) 表示输出轨迹集合；\(\mathbf{T}^{G}_{I_0}\) 表示初始时刻 IMU 到全局的位姿；\(\mathbf{T}^{G}_{I_1}\) 表示第二个输出时刻 IMU 到全局的位姿；\(\mathbf{T}^{G}_{I_K}\) 表示第 \(K\) 个输出时刻 IMU 到全局的位姿；\(K\) 表示最终输出时刻索引。

数值参数化例子：若系统运行 5 秒，相机频率 20 Hz，则最多可输出约 100 个图像时刻位姿。若每帧沿全局 x 方向平均前进 0.01 m，则 5 秒后轨迹末端位置约为 \([1.0,0,0]^\top\,\mathrm{m}\)，真实 OpenVINS 输出会同时包含由 IMU 和视觉共同校正后的姿态、速度、bias 和协方差信息。

## 18. 关键理解总结

OpenVINS 的核心是“IMU 提供高频先验，视觉提供低频几何校正”。IMU 传播给出连续运动预测和协方差增长；视觉投影残差把图像特征约束转化为状态校正；MSCKF 通过零空间投影避免把普通点全部放入状态；SLAM 特征保留少量长期点以增强稳定性；ZUPT 用静止时的 IMU 和视差证据降低漂移；FEJ 维护不可观方向的一致性。

若要定位算法实现入口，可按以下文件阅读：

- `ov_msckf/src/core/VioManager.cpp`：整体数据流、初始化、传播、更新调度。
- `ov_msckf/src/state/Propagator.cpp`：IMU 传播、协方差传播、时间偏移处理和 clone 增广。
- `ov_msckf/src/update/UpdaterMSCKF.cpp`：MSCKF 特征筛选、三角化、零空间投影、卡方检验、测量压缩和 EKF 更新。
- `ov_msckf/src/update/UpdaterSLAM.cpp`：SLAM / ArUco 特征延迟初始化、常规更新和锚点变换。
- `ov_msckf/src/update/UpdaterZeroVelocity.cpp`：ZUPT 静止检测和零速更新。
- `ov_core/src/feat/FeatureInitializer.cpp`：三角化、1D 三角化、重投影优化和特征质量检查。
- `docs/propagation*.dox`、`docs/update*.dox`、`docs/fej.dox`：项目自带推导文档。
