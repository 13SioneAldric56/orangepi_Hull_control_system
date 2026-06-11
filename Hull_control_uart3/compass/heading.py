"""航向角格式转换与磁偏角处理。"""

from __future__ import annotations

import math

from .config import HeadingFormat, NorthReference


def normalize_deg360(angle: float) -> float:
  wrapped = angle % 360.0
  if wrapped < 0:
    wrapped += 360.0
  return wrapped


def to_heading_format(angle_deg360: float, fmt: HeadingFormat) -> float:
  a = normalize_deg360(angle_deg360)
  if fmt == HeadingFormat.DEG360:
    return a
  if a > 180.0:
    return a - 360.0
  return a


def apply_north_reference(
  heading_mag_deg360: float,
  north: NorthReference,
  declination_deg: float,
) -> float:
  if north == NorthReference.TRUE:
    return normalize_deg360(heading_mag_deg360 + declination_deg)
  return normalize_deg360(heading_mag_deg360)


def angle_difference(angle1: float, angle2: float) -> float:
  diff = angle2 - angle1
  while diff > 180:
    diff -= 360
  while diff < -180:
    diff += 360
  return diff
