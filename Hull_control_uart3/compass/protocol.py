"""DDM350B 协议：组帧、校验、角度解析。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

FRAME_HEADER = 0x68
CMD_READ_ANGLES = 0x04
CMD_SET_MODE = 0x0C
CMD_SET_BAUDRATE = 0x0B
CMD_SET_MAG_DECLINATION = 0x06
CMD_READ_MAG_DECLINATION = 0x07
CMD_START_CALIBRATION = 0x08
CMD_SAVE_CALIBRATION = 0x0A
CMD_SET_ADDRESS = 0x0F

RSP_READ_ANGLES = 0x84
RSP_SET_MODE = 0x8C


@dataclass(frozen=True)
class RawAngles:
  roll: float
  pitch: float
  heading: float


def checksum(data: bytes) -> int:
  return sum(data) & 0xFF


def parse_angle_bytes(b1: int, b2: int, b3: int) -> float:
  """
  三字节 BCD → 角度（°）。
  d1: 符号 (0 正, 1 负); d2–d6: 百十个.十分百分位。
  """
  d1 = (b1 >> 4) & 0x0F
  d2 = b1 & 0x0F
  d3 = (b2 >> 4) & 0x0F
  d4 = b2 & 0x0F
  d5 = (b3 >> 4) & 0x0F
  d6 = b3 & 0x0F
  sign = -1 if d1 == 1 else 1
  value = d2 * 100 + d3 * 10 + d4 + d5 * 0.1 + d6 * 0.01
  return sign * value


def build_packet(length: int, address: int, cmd: int, data: bytes = b"") -> bytes:
  body = bytes([length, address, cmd]) + data
  chk = checksum(body)
  return bytes([FRAME_HEADER]) + body + bytes([chk])


def build_read_angles_cmd() -> bytes:
  return build_packet(0x04, 0x00, CMD_READ_ANGLES)


def build_set_mode_cmd(mode: int) -> bytes:
  """标准格式: 68 05 00 0C <mode> <chk>"""
  return build_packet(0x05, 0x00, CMD_SET_MODE, bytes([mode & 0xFF]))


def build_set_mode_cmd_legacy(mode: int) -> bytes:
  """兼容部分固件: 68 04 00 <mode> <chk>（无 0x0C 命令字）"""
  length, address, data = 0x04, 0x00, mode & 0xFF
  chk = (length + address + data) & 0xFF
  return bytes([FRAME_HEADER, length, address, data, chk])


def build_start_calibration_cmd() -> bytes:
  return build_packet(0x04, 0x00, CMD_START_CALIBRATION)


def build_save_calibration_cmd() -> bytes:
  return build_packet(0x04, 0x00, CMD_SAVE_CALIBRATION)


def build_set_magnetic_declination(degrees: float) -> bytes:
  sign = 0x80 if degrees < 0 else 0x00
  abs_deg = abs(degrees)
  int_part = int(abs_deg)
  dec_part = int((abs_deg - int_part) * 10)
  data1 = sign | ((int_part // 10) << 4) | (int_part % 10)
  data2 = dec_part << 4
  return build_packet(0x06, 0x00, CMD_SET_MAG_DECLINATION, bytes([data1, data2]))


def parse_angle_frame(frame: bytes) -> Optional[RawAngles]:
  if len(frame) < 13 or frame[0] != FRAME_HEADER or frame[3] != RSP_READ_ANGLES:
    return None
  roll = parse_angle_bytes(frame[4], frame[5], frame[6])
  pitch = parse_angle_bytes(frame[7], frame[8], frame[9])
  heading = parse_angle_bytes(frame[10], frame[11], frame[12])
  return RawAngles(roll=roll, pitch=pitch, heading=heading)


def find_frame(buffer: bytearray) -> Optional[bytes]:
  if len(buffer) < 2:
    return None
  try:
    start_idx = buffer.index(FRAME_HEADER)
    if start_idx > 0:
      del buffer[:start_idx]
  except ValueError:
    if len(buffer) > 1:
      del buffer[:-1]
    return None
  if len(buffer) < 2:
    return None
  frame_length = buffer[1]
  total_len = frame_length + 2
  if len(buffer) >= total_len:
    frame = bytes(buffer[:total_len])
    del buffer[:total_len]
    return frame
  return None
