"""
DDM350B 电子罗盘统一接口。

推荐用法:
    from compass import DDM350B, CompassConfig, OutputMode, read_heading, calibrate

    heading = read_heading(port="/dev/ttyS0", kalman=False)
"""

from __future__ import annotations

from typing import Optional, Union

from .config import (
    Axis,
    CompassConfig,
    HeadingFormat,
    KalmanConfig,
    NorthReference,
    OutputMode,
)
from .device import CompassSample, DDM350B
from .filter import (
    AdaptiveCompassKalmanFilter,
    CompassKalmanFilter,
    FilteredCompassData,
    KalmanFilter1D,
)
from .heading import angle_difference, normalize_deg360, to_heading_format
from .wrap import HeadingWrap, HeadingWrapReader, WrappedHeadingData

# 兼容旧名
CompassData = CompassSample


def read_heading(
    port: str = "/dev/ttyS0",
    baudrate: int = 115200,
    mode: OutputMode = OutputMode.POLLING,
    kalman: bool = False,
    process_noise: float = 0.001,
    measurement_noise: float = 0.1,
    heading_format: HeadingFormat = HeadingFormat.DEG360,
    north_reference: NorthReference = NorthReference.MAGNETIC,
    magnetic_declination_deg: float = 0.0,
    **kwargs,
) -> Optional[float]:
    """读取一次航向角（度）。"""
    cfg = CompassConfig(
        port=port,
        baudrate=baudrate,
        mode=mode,
        axis=Axis.HEADING,
        kalman=KalmanConfig(
            enabled=kalman,
            process_noise=process_noise,
            measurement_noise=measurement_noise,
        ),
        heading_format=heading_format,
        north_reference=north_reference,
        magnetic_declination_deg=magnetic_declination_deg,
        auto_connect=True,
        **kwargs,
    )
    with DDM350B(cfg) as compass:
        result = compass.read()
        return result if isinstance(result, float) else None


def read_compass(
    port: str = "/dev/ttyS0",
    mode: OutputMode = OutputMode.POLLING,
    baudrate: int = 115200,
    **kwargs,
) -> Optional[CompassSample]:
    """读取一次三轴数据（兼容旧 read_compass）。"""
    cfg = CompassConfig(
        port=port,
        baudrate=baudrate,
        mode=mode,
        axis=Axis.ALL,
        auto_connect=True,
        **kwargs,
    )
    with DDM350B(cfg) as compass:
        return compass.read_full()


def calibrate(
    port: str = "/dev/ttyS0",
    baudrate: int = 115200,
    duration: int = 50,
    on_tick=None,
) -> bool:
    """设备内置磁校准：开始 → 等待 → 保存。"""
    cfg = CompassConfig(port=port, baudrate=baudrate, auto_connect=False)
    compass = DDM350B(cfg)
    if not compass.connect():
        return False
    try:
        return compass.calibrate(duration=duration, on_tick=on_tick)
    finally:
        compass.disconnect()


__all__ = [
    "DDM350B",
    "CompassConfig",
    "CompassSample",
    "CompassData",
    "OutputMode",
    "Axis",
    "HeadingFormat",
    "NorthReference",
    "KalmanConfig",
    "KalmanFilter1D",
    "CompassKalmanFilter",
    "AdaptiveCompassKalmanFilter",
    "FilteredCompassData",
    "HeadingWrap",
    "HeadingWrapReader",
    "WrappedHeadingData",
    "read_heading",
    "read_compass",
    "calibrate",
    "normalize_deg360",
    "to_heading_format",
    "angle_difference",
]
