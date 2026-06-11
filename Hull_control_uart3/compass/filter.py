"""三轴罗盘卡尔曼滤波。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, NamedTuple, Optional

import numpy as np


class FilteredCompassData(NamedTuple):
  roll: float
  pitch: float
  heading: float
  roll_std: float
  pitch_std: float
  heading_std: float


@dataclass
class KalmanFilter1D:
  process_noise: float = 0.0001
  measurement_noise: float = 0.1
  initial_value: float = 0.0
  initial_covariance: float = 1.0

  def __post_init__(self):
    self._x = self.initial_value
    self._p = self.initial_covariance
    self._initialized = False

  def reset(self, value: float = 0.0, covariance: float = 1.0):
    self._x = value
    self._p = covariance
    self._initialized = False

  def update(self, measurement: float) -> float:
    if not self._initialized:
      self._x = measurement
      self._initialized = True
      return self._x
    x_pred = self._x
    p_pred = self._p + self.process_noise
    k = p_pred / (p_pred + self.measurement_noise)
    self._x = x_pred + k * (measurement - x_pred)
    self._p = (1 - k) * p_pred
    return self._x

  def get_std(self) -> float:
    return float(np.sqrt(self._p))


class CompassKalmanFilter:
  def __init__(
    self,
    process_noise: float = 0.001,
    measurement_noise: float = 0.1,
    wrap_angles: Optional[List[bool]] = None,
  ):
    if wrap_angles is None:
      wrap_angles = [False, False, True]
    self._roll_kf = KalmanFilter1D(process_noise=process_noise, measurement_noise=measurement_noise)
    self._pitch_kf = KalmanFilter1D(process_noise=process_noise, measurement_noise=measurement_noise)
    self._heading_kf = KalmanFilter1D(process_noise=process_noise, measurement_noise=measurement_noise)
    self._wrap_angles = wrap_angles
    self._last_heading: Optional[float] = None

  def _wrap_angle_diff(self, new_angle: float, old_angle: float) -> float:
    diff = new_angle - old_angle
    while diff > 180:
      diff -= 360
    while diff < -180:
      diff += 360
    return diff

  def update(self, data) -> FilteredCompassData:
    roll = self._roll_kf.update(data.roll)
    pitch = self._pitch_kf.update(data.pitch)
    heading_raw = data.heading
    if self._wrap_angles[2]:
      if self._last_heading is not None:
        heading_diff = self._wrap_angle_diff(heading_raw, self._last_heading)
        target_heading = self._last_heading + heading_diff
        while target_heading < 0:
          target_heading += 360
        while target_heading >= 360:
          target_heading -= 360
        heading = self._heading_kf.update(target_heading)
      else:
        heading = self._heading_kf.update(heading_raw)
      self._last_heading = heading
    else:
      heading = self._heading_kf.update(heading_raw)
    return FilteredCompassData(
      roll=roll,
      pitch=pitch,
      heading=heading,
      roll_std=self._roll_kf.get_std(),
      pitch_std=self._pitch_kf.get_std(),
      heading_std=self._heading_kf.get_std(),
    )

  def reset(self):
    self._roll_kf.reset()
    self._pitch_kf.reset()
    self._heading_kf.reset()
    self._last_heading = None

  def set_noise(
    self,
    process_noise: Optional[float] = None,
    measurement_noise: Optional[float] = None,
  ):
    if process_noise is not None:
      self._roll_kf.process_noise = process_noise
      self._pitch_kf.process_noise = process_noise
      self._heading_kf.process_noise = process_noise
    if measurement_noise is not None:
      self._roll_kf.measurement_noise = measurement_noise
      self._pitch_kf.measurement_noise = measurement_noise
      self._heading_kf.measurement_noise = measurement_noise


class AdaptiveCompassKalmanFilter(CompassKalmanFilter):
  def __init__(
    self,
    process_noise: float = 0.001,
    measurement_noise: float = 0.1,
    wrap_angles: Optional[List[bool]] = None,
    adaptation_rate: float = 0.1,
    min_measurement_noise: float = 0.05,
    max_measurement_noise: float = 1.0,
  ):
    super().__init__(process_noise, measurement_noise, wrap_angles)
    self._adaptation_rate = adaptation_rate
    self._min_measurement_noise = min_measurement_noise
    self._max_measurement_noise = max_measurement_noise
    self._last_roll = None
    self._last_pitch = None
    self._last_heading_raw = None

  def update(self, data) -> FilteredCompassData:
    roll_diff = abs(data.roll - self._last_roll) if self._last_roll is not None else 0.0
    pitch_diff = abs(data.pitch - self._last_pitch) if self._last_pitch is not None else 0.0
    heading_diff = (
      abs(self._wrap_angle_diff(data.heading, self._last_heading_raw))
      if self._last_heading_raw is not None
      else 0.0
    )
    total_diff = roll_diff + pitch_diff + heading_diff
    new_r = max(
      self._min_measurement_noise,
      min(self._max_measurement_noise, total_diff * self._adaptation_rate * 10),
    )
    self.set_noise(measurement_noise=new_r)
    self._last_roll = data.roll
    self._last_pitch = data.pitch
    self._last_heading_raw = data.heading
    return super().update(data)
