"""验证 RAW 时间偏移估计、命令行预览和标定文件回写。"""

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "calibrate_time_offset.py"
SPEC = importlib.util.spec_from_file_location("calibrate_time_offset", SCRIPT)
calibrator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = calibrator
SPEC.loader.exec_module(calibrator)


def _orientation(times):
    """返回时刻数组对应的合成三轴姿态。"""
    return Rotation.from_euler("xyz", np.column_stack((
        .28 * np.sin(4.2 * times) + .09 * np.sin(13.3 * times),
        .22 * np.sin(6.1 * times) + .13 * np.cos(11.1 * times),
        .19 * np.cos(5.3 * times) + .08 * np.sin(16.7 * times),
    )))


def _motion(offset, speed=1.):
    """生成带指定 IMU 修正量（秒）和运动速率倍率的采样。"""
    imu_time = np.arange(0, 8, .002)
    physical_time = imu_time + offset
    epsilon = .00001  # 用中心差分近似瞬时角速度的半步长。
    gyro = (_orientation((physical_time - epsilon) * speed).inv() *
            _orientation((physical_time + epsilon) * speed)).as_rotvec() / (2 * epsilon)
    starts = np.arange(.3, 7.6, .05)
    windows = np.column_stack((starts, starts + .05))
    camera_rotations = (_orientation(windows[:, 0] * speed).inv() *
                        _orientation(windows[:, 1] * speed)).as_rotvec()
    return imu_time, gyro, windows, camera_rotations


@pytest.mark.parametrize("offset", [-.020, 0., .0175])
def test_estimates_total_offset_with_correct_sign(offset):
    """恢复正、负和零偏移，校验给原始 IMU 时间戳加修正的约定。"""
    result = calibrator.fit_time_offset(*_motion(offset))
    assert result.seconds == pytest.approx(offset, abs=.00025)
    assert result.median_error_deg < .01


def test_rejects_stationary_motion():
    """静止图像无法确定时差，估计应报告可用窗口不足。"""
    imu_time, gyro, windows, rotations = _motion(0.)
    with pytest.raises(ValueError, match="有效窗口"):
        calibrator.fit_time_offset(imu_time, np.zeros_like(gyro), windows, np.zeros_like(rotations))


def test_rejects_search_boundary():
    """真实偏移超出搜索区间时拒绝边界解。"""
    with pytest.raises(ValueError, match="搜索边界"):
        calibrator.fit_time_offset(*_motion(.105))


def test_rejects_constant_speed_rotation():
    """恒定角速度下各候选时差等价，估计应拒绝写回用的结果。"""
    imu_time, _, windows, _ = _motion(0.)
    gyro = np.tile([0., 0., 1.], (len(imu_time), 1))
    camera_rotations = np.column_stack((np.zeros((len(windows), 2)), np.diff(windows, axis=1)))
    with pytest.raises(ValueError):
        calibrator.fit_time_offset(imu_time, gyro, windows, camera_rotations)


def test_estimates_offset_with_minority_of_bad_windows():
    """保留对少量视觉跟踪异常窗口的容忍度。"""
    imu_time, gyro, windows, rotations = _motion(.020, speed=1.5)
    rotations[::5, 0] += .12

    result = calibrator.fit_time_offset(imu_time, gyro, windows, rotations)

    assert result.seconds == pytest.approx(.020, abs=.00025)


def test_updates_only_offset_and_keeps_original_backup(tmp_path):
    """保留 CRLF、注释、重复的 imu 和权限，并备份原始字节。"""
    original = (
        b"# calibration\r\nimu: {gyroscope_bias: [1, 2, 3]}\r\n"
        b"imu: {gyroscope_bias: [4, 5, 6]}\r\n"
        b"ROS.ImuTimeOffsetSec:  -0.005  # seconds\r\n"
        b"unrelated: {gain: 3}\r\n"
    )
    path = tmp_path / "calib.yaml"
    path.write_bytes(original)
    path.chmod(0o664)

    backup = calibrator.update_calibration(path, original, -.020)

    assert path.read_bytes() == original.replace(b"-0.005 ", b"-0.020000000 ")
    assert path.stat().st_mode & 0o777 == 0o664
    assert backup.read_bytes() == original
    assert backup.parent == path.parent


@pytest.mark.parametrize(("original", "expected"), [
    (b"cam0: {}\n...\n", b"cam0: {}\nROS.ImuTimeOffsetSec: 0.000000000\n...\n"),
    (b"{cam0: {}}\n", b"{cam0: {}, ROS.ImuTimeOffsetSec: 0.000000000}\n"),
    (b"ROS.ImuTimeOffsetSec: # empty\nimu: {}\n", b"ROS.ImuTimeOffsetSec: 0.000000000 # empty\nimu: {}\n"),
])
def test_inserts_missing_or_empty_offset(tmp_path, original, expected):
    """在合法的 YAML 位置添加缺失值或填写空值。"""
    path = tmp_path / "calib.yaml"
    path.write_bytes(original)
    calibrator.update_calibration(path, original, 0.)
    assert path.read_bytes() == expected


def test_duplicate_offset_key_prevents_write(tmp_path):
    """拒绝有多个生效候选的时差字段。"""
    path = tmp_path / "calib.yaml"
    original = b"ROS.ImuTimeOffsetSec: 0.0\nROS.ImuTimeOffsetSec: 0.1\n"
    path.write_bytes(original)
    with pytest.raises(ValueError, match="重复"):
        calibrator.update_calibration(path, original, -.020)
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]


def test_concurrent_edit_prevents_write(tmp_path):
    """标定期间文件被编辑后保留新内容。"""
    path = tmp_path / "calib.yaml"
    original = b"ROS.ImuTimeOffsetSec: 0.0\n"
    changed = b"ROS.ImuTimeOffsetSec: 0.01\n"
    path.write_bytes(changed)
    with pytest.raises(ValueError, match="已变更"):
        calibrator.update_calibration(path, original, -.020)
    assert path.read_bytes() == changed
    assert list(tmp_path.iterdir()) == [path]


def test_raw_rejects_duplicate_camera_timestamp_column(tmp_path):
    """重复时间戳列会让实际使用的时间轴歧义，应拒绝估计。"""
    images = tmp_path / "images"
    images.mkdir()
    for image in ("a.jpg", "b.jpg"):
        (images / image).touch()
    (tmp_path / "image_timestamps.txt").write_text(
        "image_file,cam_time_ns,cam_time_ns\na.jpg,0,1\nb.jpg,2,3\n")
    (tmp_path / "imu.csv").write_text(
        "time_ns,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s\n0,0,0,0\n3,0,0,0\n")
    with pytest.raises(ValueError, match="重复列名"):
        calibrator._raw_input(tmp_path)


def _estimate_stub(*_):
    """提供确定性估计结果以验证命令行文件操作。"""
    return calibrator.OffsetEstimate(-.020, 120, .09, .15, (-.0205, -.0195))


def test_cli_refuses_write_when_most_windows_disagree(tmp_path, monkeypatch, capsys):
    """少数窗口支持错误偏移时，保留原标定且不创建备份。"""
    path = tmp_path / "calib.yaml"
    original = b"ROS.ImuTimeOffsetSec: 0.020\n"
    path.write_bytes(original)

    true_offset, false_offset, speed = .020, -.040, 1.5
    imu_time, gyro, windows, rotations = _motion(true_offset, speed)
    rotations[:, 0] += .12
    # 每三个窗口中有一个吻合错误时差，且这些窗口分布在整个采集段。
    false_starts, false_ends = ((windows[::3] + true_offset - false_offset) * speed).T
    rotations[::3] = (_orientation(false_starts).inv() * _orientation(false_ends)).as_rotvec()

    def estimate_from_motion(*_):
        """用合成旋转代替图像解码和跟踪，执行实际的时差拟合。"""
        return calibrator.fit_time_offset(imu_time, gyro, windows, rotations)

    monkeypatch.setattr(calibrator, "estimate_raw", estimate_from_motion)

    assert calibrator.main(["--in-raw", str(tmp_path), "--write"]) == 1
    assert "旋转拟合质量不足" in capsys.readouterr().err
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]


def test_cli_previews_then_writes_parent_calibration(tmp_path, monkeypatch, capsys):
    """默认预览父目录标定，明确请求后才回写。"""
    raw = tmp_path / "002"
    raw.mkdir()
    path = tmp_path / "calib.yaml"
    original = b"ROS.ImuTimeOffsetSec: -0.005\n"
    path.write_bytes(original)
    monkeypatch.setattr(calibrator, "estimate_raw", _estimate_stub)
    args = ["--in-raw", str(raw)]

    assert calibrator.main(args) == 0
    assert "estimated_ms=-20.000" in capsys.readouterr().out
    assert path.read_bytes() == original
    assert not list(tmp_path.glob("calib.yaml.bak.*"))

    assert calibrator.main([*args, "--write"]) == 0
    assert path.read_bytes() == b"ROS.ImuTimeOffsetSec: -0.020000000\n"
    backup, = tmp_path.glob("calib.yaml.bak.*")
    assert backup.read_bytes() == original


def test_cli_writes_parent_calibration_next_to_raw_symlink(tmp_path, monkeypatch):
    """RAW 目录为符号链接时，父目录与 RAW 启动入口的解析一致。"""
    recording = tmp_path / "recordings" / "002"
    recording.mkdir(parents=True)
    raw = tmp_path / "002"
    raw.symlink_to(recording, target_is_directory=True)
    selected = raw.parent / "calib.yaml"
    target_parent = recording.parent / "calib.yaml"
    original = b"ROS.ImuTimeOffsetSec: 0.010\n"
    other = b"ROS.ImuTimeOffsetSec: 0.015\n"
    selected.write_bytes(original)
    target_parent.write_bytes(other)
    monkeypatch.setattr(calibrator, "estimate_raw", _estimate_stub)

    assert calibrator.main(["--in-raw", str(raw), "--write"]) == 0
    assert selected.read_bytes() == b"ROS.ImuTimeOffsetSec: -0.020000000\n"
    assert target_parent.read_bytes() == other
    backup, = tmp_path.glob("calib.yaml.bak.*")
    assert backup.read_bytes() == original
