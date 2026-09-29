# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright 2026 Marc Duclusaud

import math
import threading
import time
from dataclasses import dataclass
import numpy as np

from constants import IMU_I2C_ADDRESS, IMU_MOUNT_QUAT


@dataclass(frozen=True)
class IMUSnapshot:
    timestamp_s: float
    quat: tuple[float, float, float, float]
    gyro: tuple[float, float, float]
    acc: tuple[float, float, float]
    valid: bool
    error_count: int


def imu_quat_to_body(
    q: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Convert a quaternion measured in IMU frame to the trunk (body) frame.

    Applies q_body = q_imu * conjugate(IMU_MOUNT_QUAT).
    """
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = IMU_MOUNT_QUAT[0], -IMU_MOUNT_QUAT[1], -IMU_MOUNT_QUAT[2], -IMU_MOUNT_QUAT[3]
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )

def quat_apply_inverse(quat: list[float], vec: list[float]) -> list[float]:
    """Apply an inverse quaternion rotation to a vector.

    Args:
        quat: The quaternion in (w, x, y, z) format.
        vec: The vector in (x, y, z) format.

    Returns:
        The rotated vector in (x, y, z) format.
    """
    xyz = quat[1:]
    w = quat[0]
    t = 2 * np.cross(xyz, vec)
    return vec - w * t + np.cross(xyz, t)

class ThreadedIMUReader:
    """Read BNO085/BNO080 fusion: wxyz quaternion, rad/s gyro and acceleration in g."""

    def __init__(self, i2c_bus: int, frequency_hz: float = 200.0, warn_interval_s: float = 1.0,
                 address: int = IMU_I2C_ADDRESS) -> None:
        if not math.isfinite(frequency_hz) or frequency_hz <= 0:
            raise ValueError("frequency_hz must be finite and > 0")

        self._period_s = 1.0 / frequency_hz
        report_interval_us = round(self._period_s * 1_000_000)
        if not 1 <= report_interval_us <= 0xFFFFFFFF:
            raise ValueError("frequency_hz is outside the BNO08x report interval range")

        # Keep hardware-only imports out of simulation and orientation helpers.
        from adafruit_extended_bus import ExtendedI2C
        from adafruit_bno08x import (
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
            BNO_REPORT_GAME_ROTATION_VECTOR,
        )
        from adafruit_bno08x.i2c import BNO08X_I2C

        self._i2c = ExtendedI2C(i2c_bus)
        try:
            self._imu = BNO08X_I2C(self._i2c, address=address)
            # Game fusion uses gravity and gyro, avoiding motor-induced magnetic
            # heading corrections. Yaw may drift, as with the previous 6-axis IMU.
            for feature in (BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE,
                            BNO_REPORT_GAME_ROTATION_VECTOR):
                self._imu.enable_feature(feature, report_interval=report_interval_us)
        except Exception:
            self._i2c.deinit()
            raise
        self._closed = False
        self._warn_interval_s = warn_interval_s

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run_loop, name="imu-reader", daemon=True)

        now = time.perf_counter()
        self._snapshot = IMUSnapshot(
            timestamp_s=now,
            quat=(1.0, 0.0, 0.0, 0.0),
            gyro=(0.0, 0.0, 0.0),
            acc=(0.0, 0.0, 0.0),
            valid=False,
            error_count=0,
        )

        self._error_count = 0
        self._last_warn_s = 0.0

    def start(self) -> None:
        if self._stop_event.is_set():
            raise RuntimeError("A stopped IMU reader cannot be restarted")
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self, timeout_s: float = 1.0) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=timeout_s)
        if not self._thread.is_alive() and not self._closed:
            self._i2c.deinit()
            self._closed = True

    def get_latest(self) -> IMUSnapshot:
        with self._lock:
            return self._snapshot

    def get_status(self) -> dict[str, float | int | bool]:
        snap = self.get_latest()
        now = time.perf_counter()
        return {
            "valid": snap.valid,
            "age_s": max(0.0, now - snap.timestamp_s),
            "error_count": snap.error_count,
            "target_frequency_hz": 1.0 / self._period_s,
        }

    def _run_loop(self) -> None:
        next_tick = time.perf_counter()

        while not self._stop_event.is_set():
            now = time.perf_counter()
            try:
                # Adafruit reports xyzw and m/s^2; callers expect wxyz and g.
                x, y, z, w = self._imu.game_quaternion
                gx, gy, gz = self._imu.gyro
                ax, ay, az = self._imu.acceleration
                if not all(math.isfinite(v) for v in (w, x, y, z, gx, gy, gz, ax, ay, az)):
                    raise ValueError("Non-finite IMU sample")
                norm = math.sqrt(w*w + x*x + y*y + z*z)
                if norm < 1e-6:
                    raise ValueError("Invalid IMU quaternion")
                with self._lock:
                    self._snapshot = IMUSnapshot(
                        timestamp_s=now,
                        quat=(w / norm, x / norm, y / norm, z / norm),
                        gyro=(float(gx), float(gy), float(gz)),
                        acc=(ax / 9.80665, ay / 9.80665, az / 9.80665),
                        valid=True,
                        error_count=self._error_count,
                    )
            except Exception as exc:
                self._error_count += 1
                if (now - self._last_warn_s) >= self._warn_interval_s:
                    print(f"Warning: IMU read failed ({self._error_count}): {exc}", end="\r\n", flush=True)
                    self._last_warn_s = now
                with self._lock:
                    prev = self._snapshot
                    self._snapshot = IMUSnapshot(
                        timestamp_s=prev.timestamp_s,
                        quat=prev.quat,
                        gyro=prev.gyro,
                        acc=prev.acc,
                        valid=False,
                        error_count=self._error_count,
                    )

            next_tick += self._period_s
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                self._stop_event.wait(sleep_s)
            else:
                # Reset cadence anchor when late to avoid accumulating drift.
                next_tick = time.perf_counter()
