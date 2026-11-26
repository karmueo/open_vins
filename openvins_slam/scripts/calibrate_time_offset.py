#!/usr/bin/env python3
"""用 RAW 双目旋转和陀螺积分标定时差，并按需更新 calib.yaml。"""

import argparse
import csv
import fcntl
import math
import os
from dataclasses import dataclass
from pathlib import Path
import shutil
import sys
import tempfile

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml


OFFSET_KEY = "ROS.ImuTimeOffsetSec"  # OpenVINS 读取的顶层修正量字段。
SEARCH_LIMIT_SEC = .100  # 候选偏移对称区间的半宽。
SEARCH_STEP_SEC = .00025  # 候选偏移之间的时间间隔。
WINDOW_SEC = .050  # 用于估计视觉旋转的目标帧间隔。
ROTATION_ERROR_LIMIT_RAD = .020  # 拟合损失的截断点，也是判定窗口吻合的误差上限。


@dataclass(frozen=True)
class OffsetEstimate:
    """记录总修正量、有效窗口数、旋转误差和按秒分块重采样区间。"""

    seconds: float  # 直接加到原始 IMU 时间戳上的修正量。
    windows: int  # 共同用于比较所有候选偏移的有效窗口数。
    median_error_deg: float  # 最优修正下完整旋转误差的中位数。
    rms_error_deg: float  # 最优修正下截断旋转误差的均方根。
    interval_sec: tuple[float, float]  # 分块重采样的分位区间。


def fit_time_offset(imu_time, gyro, windows, camera_rotations):
    """估计加到 IMU 时间戳上的修正量，单位为秒。

    输入共用秒级时间轴。gyro 为相机坐标系中的角速度（rad/s），
    windows 为图像起止时刻，camera_rotations 为对应的相对旋转向量（rad）。
    返回 OffsetEstimate。数据覆盖、运动或拟合置信度不足时抛出 ValueError。
    """
    if len(imu_time) < 2 or not np.all(np.isfinite(imu_time)) or np.any(np.diff(imu_time) <= 0):
        raise ValueError("IMU 时间戳须有限且严格递增")
    if not np.all(np.isfinite(gyro)):
        raise ValueError("陀螺角速度含非有限值")

    # 各候选偏移使用同一批窗口，积分也避开明显缺样处。
    valid = (np.isfinite(camera_rotations).all(axis=1)
             & (np.linalg.norm(camera_rotations, axis=1) >= np.deg2rad(.25))
             & (windows[:, 1] > windows[:, 0])
             & (windows[:, 0] - SEARCH_LIMIT_SEC >= imu_time[0])
             & (windows[:, 1] + SEARCH_LIMIT_SEC <= imu_time[-1]))
    imu_step = np.diff(imu_time)
    for gap in np.flatnonzero(imu_step > 3 * np.median(imu_step)):
        valid &= ((windows[:, 1] + SEARCH_LIMIT_SEC < imu_time[gap])
                  | (windows[:, 0] - SEARCH_LIMIT_SEC > imu_time[gap + 1]))
    windows = windows[valid]
    blocks = np.floor(windows[:, 0])
    seconds = np.unique(blocks)
    if len(windows) < 30 or len(seconds) < 3:
        raise ValueError(f"有效窗口不足（{len(windows)} 个，覆盖 {len(seconds)} 个一秒时间段）")

    visual = Rotation.from_rotvec(camera_rotations[valid])
    steps = Rotation.from_rotvec((gyro[:-1] + gyro[1:]) * (imu_step[:, None] / 2))
    orientation = Rotation.identity()
    quaternions = np.empty((len(imu_time), 4))
    quaternions[0] = orientation.as_quat()
    for index, step in enumerate(steps, 1):
        orientation = orientation * step
        quaternions[index] = orientation.as_quat()
    interpolate = Slerp(imu_time, Rotation.from_quat(quaternions))

    # 图像时刻 t 对应原始 IMU 时间轴上的 t - s。
    half_count = round(SEARCH_LIMIT_SEC / SEARCH_STEP_SEC)
    candidates = np.arange(-half_count, half_count + 1) * SEARCH_STEP_SEC
    errors = []
    for shift in candidates:
        poses = interpolate((windows - shift).ravel())
        predicted = poses[::2].inv() * poses[1::2]
        errors.append((predicted.inv() * visual).magnitude())
    errors = np.asarray(errors)
    squared = np.minimum(errors, ROTATION_ERROR_LIMIT_RAD) ** 2
    rms = np.sqrt(squared.mean(axis=1))
    best = int(np.argmin(rms))
    if best in (0, len(candidates) - 1):
        raise ValueError("最优时差落在 ±100 ms 搜索边界")
    distant = np.abs(np.arange(len(candidates)) - best) >= round(.005 / SEARCH_STEP_SEC)
    if rms[distant].min() - rms[best] < max(.05 * rms[best], 1e-8):
        raise ValueError("距最优值 5 ms 及以上的候选与最优值无法可靠区分")

    matched = np.count_nonzero(errors[best] <= ROTATION_ERROR_LIMIT_RAD)
    if matched * 2 <= len(windows):
        raise ValueError(f"旋转拟合质量不足（{matched}/{len(windows)} 个窗口的旋转误差不超过 "
                         f"{ROTATION_ERROR_LIMIT_RAD:.3f} rad，须超过半数）")

    block_loss = np.array([squared[:, blocks == second].sum(axis=1) for second in seconds])
    # 固定随机种子保证同一份数据的区间和写回判定可复现。
    random = np.random.default_rng(1939)
    selections = random.multinomial(len(seconds), np.full(len(seconds), 1 / len(seconds)), size=1000)
    estimates = candidates[np.argmin(selections @ block_loss, axis=1)]
    lower, upper = np.quantile(estimates, [.025, .975])
    if upper - lower > .005 + 1e-12:
        raise ValueError(f"按秒分块估计的时差区间过宽（{(upper - lower) * 1e3:.3f} ms）")
    return OffsetEstimate(float(candidates[best]), len(windows),
                          float(np.rad2deg(np.median(errors[best]))),
                          float(np.rad2deg(rms[best])), (float(lower), float(upper)))


def _csv_rows(path, columns):
    """按列名读取 RAW 索引或 IMU CSV，跳过注释和空行。"""
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = csv.DictReader(line for line in stream if line.strip() and not line.lstrip().startswith("#"))
        if rows.fieldnames:
            rows.fieldnames = [name.strip() for name in rows.fieldnames]
        if not rows.fieldnames or not set(columns).issubset(rows.fieldnames):
            raise ValueError(f"{path} 缺少必需列 {', '.join(columns)}")
        if len(set(rows.fieldnames)) != len(rows.fieldnames):
            raise ValueError(f"{path} 含重复列名")
        for row in rows:
            if None in row or None in row.values():
                raise ValueError(f"{path} 的 CSV 记录列数不正确")
            yield tuple(row[column].strip() for column in columns)


def _raw_input(raw):
    """按 RAW 目录规则返回图像路径、相机时刻、IMU 时刻和三轴陀螺数据。"""
    directory = next((path for path in (raw / "aligned", raw)
                      if (path / "image_timestamps.txt").is_file() and (path / "imu.csv").is_file()), None)
    if directory is None:
        raise ValueError(f"{raw} 或其 aligned/ 下须成对提供 image_timestamps.txt 和 imu.csv")
    images = raw / "images"
    paths = []
    stamps = []
    for name, stamp in _csv_rows(directory / "image_timestamps.txt", ("image_file", "cam_time_ns")):
        relative = Path(name)
        path = images / relative
        if not name or relative.is_absolute() or ".." in relative.parts or not path.is_file():
            raise ValueError(f"无效的 RAW 图像路径 {name!r}")
        paths.append(path)
        stamps.append(int(stamp))
    imu_stamps = []
    gyro = []
    columns = ("time_ns", "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s")
    for stamp, *rates in _csv_rows(directory / "imu.csv", columns):
        imu_stamps.append(int(stamp))
        gyro.append([float(rate) for rate in rates])

    axes = []
    for name, values in (("相机", stamps), ("IMU", imu_stamps)):
        axis = np.asarray(values, dtype=np.int64)
        if len(axis) < 2 or axis[0] < 0 or np.any(axis[1:] <= axis[:-1]):
            raise ValueError(f"{name}须有至少两个严格递增的非负纳秒时间戳")
        axes.append(axis)
    # 在整数域减去公共起点，保留纪元纳秒时间戳的精度。
    camera_time, imu_time = [(axis - axes[0][0]) * 1e-9 for axis in axes]
    return paths, camera_time, imu_time, np.asarray(gyro)


def _array(values, shape, name):
    """将标定参数转换成指定形状的有限浮点数组。"""
    result = np.asarray(values, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 应为形状 {shape} 的有限数值")
    return result


def _rectification(calibration):
    """由两目内外参创建约四分之一分辨率的矫正映射和投影矩阵。"""
    matrices, distortions, transforms, resolutions = [], [], [], []
    for name in ("cam0", "cam1"):
        camera = calibration[name]
        if camera["distortion_model"] != "fisheye":
            raise ValueError(f"{name} 须使用 fisheye 畸变模型")
        resolution = _array(camera["resolution"], (2,), f"{name}.resolution")
        if np.any(resolution <= 0) or np.any(resolution % 2):
            raise ValueError(f"{name} 的宽高须为正偶数")
        width, height = map(int, resolution)
        resolutions.append((width, height))
        fx, fy, cx, cy = _array(camera["intrinsics"], (4,), f"{name}.intrinsics")
        if min(fx, fy) <= 0:
            raise ValueError(f"{name} 的焦距须为正")
        sx, sy = (width // 4) / width, (height // 4) / height
        matrices.append(np.array([[fx * sx, 0, (cx + .5) * sx - .5],
                                  [0, fy * sy, (cy + .5) * sy - .5], [0, 0, 1]]))
        distortions.append(_array(camera["distortion_coeffs"], (4,), f"{name}.distortion_coeffs"))
        transform = _array(camera["T_cam_imu"], (4, 4), f"{name}.T_cam_imu").copy()
        rotation = transform[:3, :3]
        if (np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 1e-5
                or abs(np.linalg.det(rotation) - 1) > 1e-5
                or np.linalg.norm(transform[3] - [0, 0, 0, 1]) > 1e-10):
            raise ValueError(f"{name}.T_cam_imu 须为刚体变换")
        transform[:3, 3] *= .001  # 标定平移用毫米，本文计算用米。
        transforms.append(transform)

    if resolutions[0] != resolutions[1]:
        raise ValueError("两目分辨率须相同")
    width, height = resolutions[0]
    size = (width // 4, height // 4)
    if size[0] < 32 or size[1] < 16:
        raise ValueError("图像分辨率过小，无法估计双目视差")
    stereo = transforms[1] @ np.linalg.inv(transforms[0])
    r0, r1, p0, p1, _ = cv2.fisheye.stereoRectify(
        matrices[0], distortions[0], matrices[1], distortions[1], size,
        stereo[:3, :3], stereo[:3, 3], flags=cv2.CALIB_ZERO_DISPARITY, balance=0.)
    baseline_fx = -p1[0, 3]
    if not math.isfinite(baseline_fx) or baseline_fx <= 0:
        raise ValueError("双目标定未给出有效的正视差基线")
    maps = [cv2.fisheye.initUndistortRectifyMap(k, d, r, p, size, cv2.CV_16SC2)
            for k, d, r, p in zip(matrices, distortions, (r0, r1), (p0, p1))]
    return (width, height), size, maps, p0[:, :3], baseline_fx, r0 @ transforms[0][:3, :3]


def _rectified_frame(path, full_size, size, maps):
    """解码、拆分横向拼接的双目图，再缩放和矫正。"""
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
    width, height = full_size
    if image is None or image.shape != (height, 2 * width):
        raise ValueError(f"{path} 无法解码为标定分辨率的横向双目图")
    return [cv2.remap(cv2.resize(eye, size, interpolation=cv2.INTER_AREA), *mapping, cv2.INTER_LINEAR)
            for eye, mapping in zip(np.hsplit(image, 2), maps)]


def _visual_rotation(left, right, next_left, intrinsics, baseline_fx, stereo_matcher):
    """用双目深度和帧间跟踪估计相机旋转向量，约束不足时返回 None。"""
    source = cv2.goodFeaturesToTrack(left, 450, .008, 6)
    if source is None or len(source) < 25:
        return None
    flow = dict(winSize=(21, 21), maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, .01))
    target, forward, _ = cv2.calcOpticalFlowPyrLK(left, next_left, source, None, **flow)
    returned, backward, _ = cv2.calcOpticalFlowPyrLK(next_left, left, target, None, **flow)
    source, target = source.reshape(-1, 2), target.reshape(-1, 2)
    tracked = ((forward[:, 0] != 0) & (backward[:, 0] != 0)
               & (np.linalg.norm(returned.reshape(-1, 2) - source, axis=1) < .8))
    source, target = source[tracked], target[tracked]
    if len(source) < 25:
        return None

    disparity = stereo_matcher.compute(left, right)
    pixels = np.rint(source).astype(int)
    depth_pixels = disparity[pixels[:, 1], pixels[:, 0]] / 16.
    valid = depth_pixels > 2
    source, target = source[valid], target[valid]
    if len(source) < 25:
        return None
    z = baseline_fx / depth_pixels[valid]
    xyz = np.column_stack(((source[:, 0] - intrinsics[0, 2]) * z / intrinsics[0, 0],
                           (source[:, 1] - intrinsics[1, 2]) * z / intrinsics[1, 1], z))
    cv2.setRNGSeed(991)
    found, rotation, translation, inliers = cv2.solvePnPRansac(
        xyz, target, intrinsics, None, iterationsCount=160, reprojectionError=1.,
        confidence=.99, flags=cv2.SOLVEPNP_EPNP)
    if not found or inliers is None or len(inliers) < max(25, .40 * len(source)):
        return None
    indices = inliers[:, 0]
    rotation, translation = cv2.solvePnPRefineLM(
        xyz[indices], target[indices], intrinsics, None, rotation, translation)
    projected, _ = cv2.projectPoints(xyz[indices], rotation, translation, intrinsics, None)
    error = projected.reshape(-1, 2) - target[indices]
    if np.sqrt(np.mean(np.sum(error ** 2, axis=1))) >= .7:
        return None
    # PnP 把点从旧相机坐标系变换到新坐标系，取逆得到相机自身转动。
    return -rotation.ravel()


def estimate_raw(raw, calibration):
    """读取 RAW 目录和标定字典，通过双目旋转估计总 IMU 时间修正。"""
    paths, camera_time, imu_time, gyro = _raw_input(raw)
    full_size, size, maps, intrinsics, baseline_fx, imu_rotation = _rectification(calibration)
    bias = _array(calibration["imu"]["gyroscope_bias"], (3,), "imu.gyroscope_bias")
    gyro = (gyro - np.deg2rad(bias)) @ imu_rotation.T
    cv2.setNumThreads(1)
    disparities = min(112, ((size[0] - 8) // 16) * 16)
    stereo_matcher = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=disparities, blockSize=5, P1=200, P2=800,
        disp12MaxDiff=1, uniquenessRatio=12, speckleWindowSize=40, speckleRange=2)
    frame_step = np.median(np.diff(camera_time))
    stride = max(1, round(WINDOW_SEC / frame_step))

    windows, rotations = [], []
    previous = _rectified_frame(paths[0], full_size, size, maps)
    for index in range(stride, len(paths), stride):
        current = _rectified_frame(paths[index], full_size, size, maps)
        start, end = camera_time[[index - stride, index]]
        if end - start < 1.5 * stride * frame_step:
            motion = _visual_rotation(*previous, current[0], intrinsics, baseline_fx, stereo_matcher)
            if motion is not None:
                windows.append((start, end))
                rotations.append(motion)
        previous = current
    return fit_time_offset(imu_time, gyro, np.asarray(windows).reshape(-1, 2),
                           np.asarray(rotations).reshape(-1, 3))


def _offset_nodes(root):
    """检查顶层 YAML 映射和时差键的唯一性，返回匹配的键值节点。"""
    if not isinstance(root, yaml.MappingNode):
        raise ValueError("标定文件须为单个 YAML 映射")
    matches = [(key, value) for key, value in root.value
               if isinstance(key, yaml.ScalarNode) and key.value == OFFSET_KEY]
    if len(matches) > 1:
        raise ValueError(f"标定文件含重复的 {OFFSET_KEY}")
    return matches[0] if matches else None


def _calibration_fields(original):
    """读取标定所需的顶层字段，遵循 yaml-cpp 对重复 imu 键取首项的行为。"""
    loader = yaml.SafeLoader(original)
    try:
        root = loader.get_single_node()
        _offset_nodes(root)
        fields = {}
        for key, value in root.value:
            if (isinstance(key, yaml.ScalarNode) and key.value in ("cam0", "cam1", "imu", OFFSET_KEY)
                    and key.value not in fields):
                fields[key.value] = loader.construct_object(value, deep=True)
        previous = fields.get(OFFSET_KEY)
        if previous is not None and (not math.isfinite(float(previous)) or abs(float(previous)) > 10):
            raise ValueError(f"{OFFSET_KEY} 须为 ±10 秒内的有限数值")
        return fields
    finally:
        loader.dispose()


def _updated_yaml(original, offset):
    """用 YAML 节点位置替换时差标量，保留文件中的其余文本。"""
    text = original.decode("utf-8")
    root = yaml.compose(text, Loader=yaml.SafeLoader)
    match = _offset_nodes(root)
    literal = f"{offset:.9f}"
    if match is not None:
        key, value = match
        start, end = value.start_mark.index, value.end_mark.index
        if (not isinstance(value, yaml.ScalarNode) or start < key.end_mark.index
                or value.start_mark.line != value.end_mark.line or text[start:end].startswith(("&", "!"))):
            raise ValueError(f"{OFFSET_KEY} 须为独立的单行标量")
        replacement = (" " if start == end else "") + literal
    elif root.flow_style:
        start = end = root.end_mark.index - 1  # 在顶层的右花括号之前插入。
        previous_token = None
        for token in yaml.scan(text):
            if token.start_mark.index >= start:
                break
            previous_token = token
        separator = " " if isinstance(previous_token, yaml.tokens.FlowEntryToken) else ", " if root.value else ""
        replacement = f"{separator}{OFFSET_KEY}: {literal}"
    else:
        start = end = root.end_mark.index
        newline = "\r\n" if "\r\n" in text else "\n"
        prepend = newline if start and text[start - 1] not in "\r\n" else ""
        indent = " " * root.start_mark.column
        replacement = f"{prepend}{indent}{OFFSET_KEY}: {literal}{newline}"
    updated = text[:start] + replacement + text[end:]
    updated_offset = _offset_nodes(yaml.compose(updated, Loader=yaml.SafeLoader))
    if updated_offset is None or updated_offset[1].value != literal:
        raise ValueError(f"无法生成有效的 {OFFSET_KEY}")
    return updated.encode("utf-8")


def _write_copy(path, contents, prefix):
    """在目标同目录写入独占创建的副本，复制文件权限后同步到磁盘。"""
    staged = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=prefix, delete=False) as stream:
            staged = Path(stream.name)
            stream.write(contents)
            stream.flush()
            source_owner = path.stat()
            staged_owner = os.fstat(stream.fileno())
            if (staged_owner.st_uid, staged_owner.st_gid) != (source_owner.st_uid, source_owner.st_gid):
                os.fchown(stream.fileno(), source_owner.st_uid, source_owner.st_gid)
            shutil.copystat(path, staged)
            os.fsync(stream.fileno())
        return staged
    except BaseException:
        if staged is not None:
            staged.unlink(missing_ok=True)
        raise


def update_calibration(path, original, offset):
    """更新时差并返回备份路径。原值字节相同时返回 None。

    original 是估计启动时读取的标定内容，offset 单位为秒。写入前核对
    文件内容，发现并发变更时抛出 ValueError。标定替换使用同目录原子重命名。
    """
    if not math.isfinite(offset) or abs(offset) > SEARCH_LIMIT_SEC:
        raise ValueError("待写回的时差超出有效搜索区间")
    updated = _updated_yaml(original, offset)
    path = path.resolve(strict=True)
    directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        # 目录锁串行化本工具的提交，避免重命名文件后锁留在旧 inode 上。
        fcntl.flock(directory_fd, fcntl.LOCK_EX)
        if path.read_bytes() != original:
            raise ValueError(f"{path} 在估计期间已变更，请重新估计")
        if updated == original:
            return None
        backup = _write_copy(path, original, f"{path.name}.bak.")
        staged = _write_copy(path, updated, f".{path.name}.")
        try:
            os.utime(staged, None)
            if path.read_bytes() != original:
                raise ValueError(f"{path} 在提交期间已变更，请重新估计")
            os.replace(staged, path)
            os.fsync(directory_fd)
        finally:
            staged.unlink(missing_ok=True)
        return backup
    finally:
        os.close(directory_fd)


def main(argv=None):
    """解析命令行，预览估计结果或按 --write 更新标定，返回退出状态。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--in-raw", type=Path, required=True, help="RAW 采集目录")
    parser.add_argument("--cameras", type=Path, help="读入及回写的 calib.yaml，默认查找 RAW 目录及其父目录")
    parser.add_argument("--write", action="store_true", help=f"备份并回写 {OFFSET_KEY}")
    args = parser.parse_args(argv)
    try:
        # 与 RAW 启动入口一致，按传入路径的父目录查找标定，保留目录符号链接。
        raw = args.in_raw.expanduser().absolute()
        if not raw.is_dir():
            raise ValueError(f"RAW 目录不存在 {raw}")
        candidates = (args.cameras.expanduser(),) if args.cameras else (raw / "calib.yaml", raw.parent / "calib.yaml")
        path = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
        if path is None:
            raise ValueError(f"未找到标定文件 {' / '.join(map(str, candidates))}")
        original = path.read_bytes()
        calibration = _calibration_fields(original)
        previous = calibration.get(OFFSET_KEY)
        print(f"CALIBRATION path={path} previous_sec={previous if previous is not None else 'unset'}", flush=True)
        estimate = estimate_raw(raw, calibration)
        lower, upper = estimate.interval_sec
        print(f"TIME_OFFSET estimated_sec={estimate.seconds:.9f} estimated_ms={estimate.seconds * 1e3:.3f} "
              f"valid_windows={estimate.windows} median_error_deg={estimate.median_error_deg:.6f} "
              f"clipped_rms_deg={estimate.rms_error_deg:.6f} interval_ms=[{lower * 1e3:.3f},{upper * 1e3:.3f}]")
        if args.write:
            backup = update_calibration(path, original, estimate.seconds)
            print(f"UPDATED path={path} backup={backup}" if backup else f"UNCHANGED path={path}")
        return 0
    except (OSError, ValueError, OverflowError, TypeError, KeyError, csv.Error, yaml.YAMLError, cv2.error) as error:
        print(f"calibrate_time_offset.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
