"""
UART 电机控制帧（命令码 0x06）— /dev/ttyUSB1 等

帧格式（11 字节）:
  AA 55       帧头
  06          数据区长度（命令码起 6 字节）
  06          命令码
  L  R        左右轮档位字节（下位机 compare_map 查表得 PWM）
  20 00 09    固定尾（与下位机一致）
  CRC_L CRC_H 暂固定 66 76（后续再接入真实校验）

停转示例: AA 55 06 06 00 00 20 00 09 66 76
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

FRAME_HEADER = bytes((0xAA, 0x55))
CMD_LEN = 0x06
CMD_CODE = 0x06
FRAME_TOTAL = 11
FIXED_TAIL = bytes((0x20, 0x00, 0x09))
FIXED_CRC = bytes((0x66, 0x76))  # 暂固定，不做 Modbus 计算

# compare_map[index] -> PWM compare 值（与下位机 C 表一致）
COMPARE_MAP: Dict[int, int] = {
    0x00: 750,
    0xF0: 750,
    0x01: 750,
    0x02: 718,
    0x03: 687,
    0x04: 656,
    0x05: 625,
    0x06: 593,
    0x07: 562,
    0x08: 531,
    0x09: 500,
    0xF1: 750,
    0xF2: 781,
    0xF3: 812,
    0xF4: 843,
    0xF5: 875,
    0xF6: 906,
    0xF7: 937,
    0xF8: 968,
    0xF9: 1000,
}


def modbus_crc16(data: bytes) -> int:
    """Modbus RTU CRC16，初值 0xFFFF，多项式 0xA001。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def speed_to_index(speed: int, forward: bool) -> int:
    """
    将 0~100 速度映射为档位字节。

    - speed==0: 0x00（停）
    - forward: 0xF1~0xF9
    - reverse: 0x01~0x09（speed 越大越接近 0x09）
    """
    speed = max(0, min(100, int(speed)))
    if speed == 0:
        return 0x00
    if forward:
        step = int(round(speed * 8 / 100))
        step = max(0, min(8, step))
        return 0xF1 + step
    step = int(round(speed * 8 / 100))
    step = max(0, min(8, step))
    return 0x01 + step


def build_frame_v06(left_index: int, right_index: int) -> bytes:
    """组 11 字节控制帧。"""
    body = bytearray()
    body.extend(FRAME_HEADER)
    body.append(CMD_LEN)
    body.append(CMD_CODE)
    body.append(left_index & 0xFF)
    body.append(right_index & 0xFF)
    body.extend(FIXED_TAIL)
    body.extend(FIXED_CRC)
    return bytes(body)


def parse_frame_v06(frame: bytes) -> Optional[Tuple[int, int]]:
    """校验并解析帧，返回 (left_index, right_index)。"""
    if len(frame) != FRAME_TOTAL:
        return None
    if frame[0:2] != FRAME_HEADER or frame[2] != CMD_LEN or frame[3] != CMD_CODE:
        return None
    if frame[6:9] != FIXED_TAIL:
        return None
    if frame[9:11] != FIXED_CRC:
        return None
    return frame[4], frame[5]


if __name__ == "__main__":
    stop = build_frame_v06(0x00, 0x00)
    assert stop.hex() == "aa55060600002000096676", stop.hex()
    assert parse_frame_v06(stop) == (0x00, 0x00)
    print("停转帧:", stop.hex())

    fwd50 = build_frame_v06(0xF5, 0xF5)
    assert parse_frame_v06(fwd50) == (0xF5, 0xF5)
    print("50% 前进:", fwd50.hex())
