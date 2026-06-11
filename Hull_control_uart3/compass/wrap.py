"""航向角连续化（0/360 边界）与组合读取。"""

from __future__ import annotations

import time
from typing import NamedTuple, Optional

from .config import CompassConfig, OutputMode
from .device import DDM350B
from .filter import AdaptiveCompassKalmanFilter, CompassKalmanFilter, FilteredCompassData
from .heading import normalize_deg360


class WrappedHeadingData(NamedTuple):
    continuous_heading: float
    wrapped_heading: float
    raw_heading: float
    roll: float
    pitch: float
    heading_std: float


class HeadingWrap:
    def __init__(self, initial_heading: float = 0.0):
        self._continuous_heading = initial_heading
        self._last_wrapped_heading = initial_heading
        self._initialized = False

    def reset(self, heading: float = 0.0):
        self._continuous_heading = heading
        self._last_wrapped_heading = heading
        self._initialized = False

    def update(self, wrapped_heading: float) -> float:
        if not self._initialized:
            self._continuous_heading = wrapped_heading
            self._last_wrapped_heading = wrapped_heading
            self._initialized = True
            return self._continuous_heading
        diff = wrapped_heading - self._last_wrapped_heading
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        self._continuous_heading += diff
        self._last_wrapped_heading = wrapped_heading
        return self._continuous_heading

    def get_continuous(self) -> float:
        return self._continuous_heading

    def get_wrapped(self) -> float:
        return normalize_deg360(self._continuous_heading)


class HeadingWrapReader:
    def __init__(
        self,
        port: str = "/dev/ttyS0",
        baudrate: int = 115200,
        process_noise: float = 0.001,
        measurement_noise: float = 0.1,
        compass_mode: OutputMode = OutputMode.AUTO_50HZ,
        use_adaptive: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.compass_mode = compass_mode
        self.use_adaptive = use_adaptive
        self._compass: Optional[DDM350B] = None
        self._kalman: Optional[CompassKalmanFilter] = None
        self._heading_wrap: Optional[HeadingWrap] = None
        self._last_filtered: Optional[FilteredCompassData] = None
        self._is_running = False
        self._update_count = 0
        self._last_update_time = 0.0

    def start(self) -> bool:
        try:
            from .config import KalmanConfig

            cfg = CompassConfig(
                port=self.port,
                baudrate=self.baudrate,
                mode=self.compass_mode,
                kalman=KalmanConfig(enabled=False),
            )
            self._compass = DDM350B(cfg)
            if not self._compass.connect():
                print(f"[错误] 无法连接到罗盘 {self.port}")
                return False
            if not self._compass.set_mode(self.compass_mode):
                print(f"[警告] 设置模式 {self.compass_mode.name} 失败: {self._compass.last_error}")
            cls = AdaptiveCompassKalmanFilter if self.use_adaptive else CompassKalmanFilter
            self._kalman = cls(
                process_noise=self.process_noise,
                measurement_noise=self.measurement_noise,
                wrap_angles=[False, False, False],
            )
            self._heading_wrap = HeadingWrap()
            self._is_running = True
            return True
        except Exception as e:
            print(f"[错误] 启动失败: {e}")
            return False

    def update(self) -> Optional[WrappedHeadingData]:
        if not self._is_running or self._compass is None or self._kalman is None:
            return None
        raw = self._compass.read_raw()
        if raw is None:
            return None
        from types import SimpleNamespace

        meas = SimpleNamespace(
            roll=raw.roll, pitch=raw.pitch, heading=raw.heading_mag_deg360
        )
        self._last_filtered = self._kalman.update(meas)
        continuous = self._heading_wrap.update(self._last_filtered.heading)
        wrapped = self._heading_wrap.get_wrapped()
        self._update_count += 1
        self._last_update_time = time.time()
        return WrappedHeadingData(
            continuous_heading=continuous,
            wrapped_heading=wrapped,
            raw_heading=raw.heading_mag_deg360,
            roll=raw.roll,
            pitch=raw.pitch,
            heading_std=self._last_filtered.heading_std,
        )

    def get_heading(self) -> Optional[float]:
        if self._heading_wrap is None:
            return None
        return self._heading_wrap.get_continuous()

    def get_wrapped_heading(self) -> Optional[float]:
        if self._heading_wrap is None:
            return None
        return self._heading_wrap.get_wrapped()

    def get_filtered_data(self) -> Optional[FilteredCompassData]:
        return self._last_filtered

    def stop(self):
        self._is_running = False
        if self._compass:
            self._compass.disconnect()
            self._compass = None

    def reset(self, heading: float = 0.0):
        if self._heading_wrap:
            self._heading_wrap.reset(heading)
        if self._kalman:
            self._kalman.reset()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
