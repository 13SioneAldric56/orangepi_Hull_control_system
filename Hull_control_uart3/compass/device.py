"""DDM350B 设备接口。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterator, Optional, Union

from . import protocol as proto
from .config import (
  Axis,
  CompassConfig,
  HeadingFormat,
  KalmanConfig,
  NorthReference,
  OutputMode,
)
from .filter import AdaptiveCompassKalmanFilter, CompassKalmanFilter, FilteredCompassData
from .heading import apply_north_reference, normalize_deg360, to_heading_format
from .transport import SerialTransport


@dataclass
class CompassSample:
  roll: float
  pitch: float
  heading: float
  heading_mag_deg360: float
  timestamp: float

  def pick(self, axis: Axis) -> Union[float, "CompassSample"]:
    if axis == Axis.ROLL:
      return self.roll
    if axis == Axis.PITCH:
      return self.pitch
    if axis == Axis.HEADING:
      return self.heading
    return self


class DDM350B:
  DEFAULT_PORT = "/dev/ttyS0"
  DEFAULT_BAUDRATE = 115200

  def __init__(
    self,
    port: str | CompassConfig | None = None,
    baudrate: int | None = None,
    timeout: float = 1.0,
    config: CompassConfig | None = None,
    **kwargs,
  ):
    if isinstance(port, CompassConfig):
      self._cfg = port
    else:
      cfg_kw = dict(kwargs)
      if port is not None:
        cfg_kw.setdefault("port", port)
      if baudrate is not None:
        cfg_kw.setdefault("baudrate", baudrate)
      cfg_kw.setdefault("timeout", timeout)
      self._cfg = config or CompassConfig(**cfg_kw)

    self._transport = SerialTransport(
      self._cfg.port, self._cfg.baudrate, self._cfg.timeout
    )
    self._mode = self._cfg.mode
    self._kalman: Optional[CompassKalmanFilter] = None
    self._last_error: Optional[str] = None
    self._init_kalman()

  @property
  def config(self) -> CompassConfig:
    return self._cfg

  @property
  def last_error(self) -> Optional[str]:
    return self._last_error

  def _init_kalman(self) -> None:
    k = self._cfg.kalman
    if not k.enabled:
      self._kalman = None
      return
    cls = AdaptiveCompassKalmanFilter if k.adaptive else CompassKalmanFilter
    self._kalman = cls(
      process_noise=k.process_noise,
      measurement_noise=k.measurement_noise,
      wrap_angles=[False, False, k.wrap_heading_in_kf],
    )

  def connect(self) -> bool:
    ok = self._transport.connect()
    if not ok:
      self._last_error = f"无法打开串口 {self._cfg.port}"
      return False
    self._last_error = None
    if self._mode != OutputMode.POLLING:
      self.set_mode(self._mode)
    return True

  def disconnect(self) -> None:
    self._transport.disconnect()

  def is_connected(self) -> bool:
    return self._transport.is_connected

  def set_mode(self, mode: OutputMode) -> bool:
    """设置输出模式。多数设备不回 0x8C 确认帧，写入成功即视为生效。"""
    mode_val = int(mode)
    commands = (
      proto.build_set_mode_cmd(mode_val),
      proto.build_set_mode_cmd_legacy(mode_val),
    )
    for cmd in commands:
      if not self._transport.write(cmd):
        continue
      time.sleep(0.08)
      response = self._transport.read_frame(timeout=0.25)
      if response and len(response) > 3:
        ack = response[3]
        if ack in (proto.RSP_SET_MODE, proto.CMD_SET_MODE):
          self._mode = mode
          self._cfg.mode = mode
          self._transport.clear_buffer()
          return True
        if len(response) > 4 and response[4] == 0x00:
          self._mode = mode
          self._cfg.mode = mode
          self._transport.clear_buffer()
          return True
      # 无回显时仍采纳（与 triple_axis_reader 行为一致）
      self._mode = mode
      self._cfg.mode = mode
      self._transport.clear_buffer()
      return True
    self._last_error = "设置模式命令发送失败"
    return False

  def set_magnetic_declination(self, degrees: float) -> bool:
    self._cfg.magnetic_declination_deg = degrees
    if not self._transport.write(proto.build_set_magnetic_declination(degrees)):
      return False
    self._transport.clear_buffer()
    response = self._transport.read_frame(timeout=0.5)
    return bool(response and len(response) > 4 and response[4] == 0x00)

  def set_kalman(
    self,
    enabled: bool | None = None,
    process_noise: float | None = None,
    measurement_noise: float | None = None,
    adaptive: bool | None = None,
  ) -> None:
    k = self._cfg.kalman
    if enabled is not None:
      k.enabled = enabled
    if process_noise is not None:
      k.process_noise = process_noise
    if measurement_noise is not None:
      k.measurement_noise = measurement_noise
    if adaptive is not None:
      k.adaptive = adaptive
    self._init_kalman()
    if self._kalman and (process_noise is not None or measurement_noise is not None):
      self._kalman.set_noise(process_noise, measurement_noise)

  def set_heading_format(self, fmt: HeadingFormat) -> None:
    self._cfg.heading_format = fmt

  def set_north_reference(self, ref: NorthReference) -> None:
    self._cfg.north_reference = ref

  def start_calibration(self) -> bool:
    if not self._transport.write(proto.build_start_calibration_cmd()):
      return False
    self._transport.clear_buffer()
    response = self._transport.read_frame(timeout=0.5)
    return bool(response and len(response) > 4 and response[4] == 0x00)

  def save_calibration(self) -> bool:
    if not self._transport.write(proto.build_save_calibration_cmd()):
      return False
    self._transport.clear_buffer()
    response = self._transport.read_frame(timeout=0.5)
    return bool(response and len(response) > 4 and response[4] == 0x00)

  def calibrate(
    self,
    duration: int = 50,
    on_tick: Callable[[int], None] | None = None,
  ) -> bool:
    if not self.is_connected() and not self.connect():
      return False
    if not self.start_calibration():
      self._last_error = "开始校准失败"
      return False
    for remaining in range(duration, 0, -1):
      if on_tick:
        on_tick(remaining)
      time.sleep(1)
    if not self.save_calibration():
      self._last_error = "保存校准失败"
      return False
    return True

  def read_raw(self) -> Optional[CompassSample]:
    if not self.is_connected():
      self._last_error = "未连接"
      return None

    if self._mode == OutputMode.POLLING:
      self._transport.clear_buffer()
      if not self._transport.write(proto.build_read_angles_cmd()):
        self._last_error = "发送读角命令失败"
        return None

    frame = self._transport.read_frame(timeout=self._cfg.effective_read_timeout())
    if not frame:
      self._last_error = "读取超时"
      return None

    raw = proto.parse_angle_frame(frame)
    if raw is None:
      self._last_error = "帧解析失败"
      return None

    heading_mag = normalize_deg360(raw.heading)
    roll, pitch = raw.roll, raw.pitch

    if self._kalman:
      filtered = self._kalman.update(raw)
      roll, pitch = filtered.roll, filtered.pitch
      heading_mag = normalize_deg360(filtered.heading)

    heading_out = apply_north_reference(
      heading_mag,
      self._cfg.north_reference,
      self._cfg.magnetic_declination_deg,
    )
    heading_out = to_heading_format(heading_out, self._cfg.heading_format)

    return CompassSample(
      roll=roll,
      pitch=pitch,
      heading=heading_out,
      heading_mag_deg360=heading_mag,
      timestamp=time.time(),
    )

  def read(self) -> Optional[Union[float, CompassSample]]:
    sample = self.read_raw()
    if sample is None:
      return None
    return sample.pick(self._cfg.axis)

  def read_full(self) -> Optional[CompassSample]:
    return self.read_raw()

  def iter_samples(self, interval: float = 0.0) -> Iterator[CompassSample]:
    while self.is_connected():
      sample = self.read_raw()
      if sample:
        yield sample
      if interval > 0:
        time.sleep(interval)

  def __enter__(self):
    if self._cfg.auto_connect:
      self.connect()
    return self

  def __exit__(self, *args):
    self.disconnect()

  def __call__(self) -> Optional[Union[float, CompassSample]]:
    return self.read()
