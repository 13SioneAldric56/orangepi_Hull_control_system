#!/usr/bin/env python3
"""
模拟发送电机控制帧（AA 55 ...），支持随机负载与串口回环读取校验。

帧格式（共 11 字节）：
  AA 55       帧头
  06          数据区长度（从命令码 05 到方向字节 0F，共 6 字节）
  05          命令码
  xx xx       左轮 PWM 值（大端 uint16，数值 = 占空百分比 * 12000 / 100）
  xx xx       右轮 PWM 值（同上）
  xx          方向：低 4 位；高 2 位=左轮、低 2 位=右轮。每侧：11 停、10 进、00 退
  CRC_L CRC_H Modbus-16，从首字节 AA 起算到方向字节止，线上低字节在前
"""

from __future__ import annotations

import argparse
import random
import struct
import threading
import time
from typing import Optional

import serial

# 与 serial_s.py 等保持一致的默认参数
DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_BAUD = 115200

FRAME_HEADER = bytes((0xAA, 0x55))
CMD_LEN = 0x06  # 05 + 2 + 2 + 1
CMD_CODE = 0x05
FRAME_TOTAL = 11

# 转向字节：每侧 2 位（左=bit3-2，右=bit1-0）
WHEEL_STOP = 0b11
WHEEL_FORWARD = 0b10
WHEEL_BACKWARD = 0b00


def modbus_crc16(data: bytes) -> int:
    """Modbus RTU CRC16，初值 0xFFFF，多项式 0xA001（反射）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def duty_percent_to_pwm_u16(percent: float) -> int:
    """占空百分比 -> PWM 寄存器值：percent * 12000 / 100，与示例 100% -> 0x2EE0 一致。"""
    v = int(round(float(percent) * 12000.0 / 100.0))
    return max(0, min(0xFFFF, v))


def direction_byte(left_wheel: int, right_wheel: int) -> int:
    """
    组装方向半字节。left_wheel / right_wheel 各取 2 位编码：
    WHEEL_STOP(11)、WHEEL_FORWARD(10)、WHEEL_BACKWARD(00)。
    """
    return (((left_wheel & 3) << 2) | (right_wheel & 3)) & 0x0F


def split_direction_byte(direction_low4: int) -> tuple[int, int]:
    """返回 (左轮2位码, 右轮2位码)。"""
    d = direction_low4 & 0x0F
    return (d >> 2) & 3, d & 3


def wheel_code_label(code: int) -> str:
    if code == WHEEL_STOP:
        return "停"
    if code == WHEEL_FORWARD:
        return "进"
    if code == WHEEL_BACKWARD:
        return "退"
    return f"未定义({code:02b})"


def format_direction_byte(direction_low4: int) -> str:
    lc, rc = split_direction_byte(direction_low4)
    return f"0x{direction_low4 & 0x0F:X} 左{wheel_code_label(lc)} 右{wheel_code_label(rc)}"


def build_frame(left_percent: float, right_percent: float, steering_byte: int) -> bytes:
    lp = duty_percent_to_pwm_u16(left_percent)
    rp = duty_percent_to_pwm_u16(right_percent)
    body_wo_crc = bytearray()
    body_wo_crc.extend(FRAME_HEADER)
    body_wo_crc.append(CMD_LEN)
    body_wo_crc.append(CMD_CODE)
    body_wo_crc.extend(struct.pack(">H", lp))
    body_wo_crc.extend(struct.pack(">H", rp))
    body_wo_crc.append(steering_byte & 0xFF)
    crc = modbus_crc16(bytes(body_wo_crc))
    # Modbus 线上：CRC 低字节在前
    body_wo_crc.append(crc & 0xFF)
    body_wo_crc.append((crc >> 8) & 0xFF)
    return bytes(body_wo_crc)


def parse_and_verify(frame: bytes) -> Optional[tuple[float, float, int]]:
    """
    校验一帧 11 字节数据；成功返回 (左占空%, 右占空%, 方向低4位)，否则 None。
    方向低 4 位：高 2 位左轮、低 2 位右轮（11 停 / 10 进 / 00 退）。
    """
    if len(frame) != FRAME_TOTAL:
        return None
    if frame[0:2] != FRAME_HEADER or frame[2] != CMD_LEN or frame[3] != CMD_CODE:
        return None
    data_for_crc = frame[:9]
    crc_rx = frame[9] | (frame[10] << 8)
    if modbus_crc16(data_for_crc) != crc_rx:
        return None
    lp = struct.unpack(">H", frame[4:6])[0]
    rp = struct.unpack(">H", frame[6:8])[0]
    d = frame[8] & 0x0F
    left_pct = lp * 100.0 / 12000.0
    right_pct = rp * 100.0 / 12000.0
    return (left_pct, right_pct, d)


def example_static_frame() -> bytes:
    """与注释示例一致：左 100%（0x2EE0）、右 50%（0x1770）、方向 0x0F（左右均停）。"""
    return build_frame(100.0, 50.0, direction_byte(WHEEL_STOP, WHEEL_STOP))


def _random_direction_byte() -> int:
    """随机合法轮控码（不含未约定的 01）。"""
    codes = (WHEEL_BACKWARD, WHEEL_FORWARD, WHEEL_STOP)
    return direction_byte(random.choice(codes), random.choice(codes))


def run_loopback(
    port: str,
    baud: int,
    interval_s: float,
    count: int,
) -> None:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
    )
    stop = threading.Event()
    rx_buf = bytearray()
    rx_lock = threading.Lock()
    stats_ok = 0
    stats_bad = 0

    def reader():
        nonlocal stats_ok, stats_bad
        while not stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            with rx_lock:
                rx_buf.extend(chunk)
                while len(rx_buf) >= FRAME_TOTAL:
                    frame = bytes(rx_buf[:FRAME_TOTAL])
                    del rx_buf[:FRAME_TOTAL]
                    parsed = parse_and_verify(frame)
                    if parsed is None:
                        stats_bad += 1
                        print(f"[RX 无效] {frame.hex()}")
                    else:
                        stats_ok += 1
                        lp, rp, d = parsed
                        print(
                            f"[RX OK] {frame.hex()} | 左≈{lp:.1f}% 右≈{rp:.1f}% "
                            f"{format_direction_byte(d)}"
                        )

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    try:
        for i in range(count):
            lp = random.uniform(0.0, 100.0)
            rp = random.uniform(0.0, 100.0)
            d = _random_direction_byte()
            pkt = build_frame(lp, rp, d)
            ser.write(pkt)
            ser.flush()
            print(
                f"[TX {i+1}/{count}] {pkt.hex()} | "
                f"左{lp:.1f}% 右{rp:.1f}% {format_direction_byte(d)}"
            )
            time.sleep(interval_s)
    finally:
        stop.set()
        time.sleep(0.3)
        ser.close()
    print(f"结束：校验通过 {stats_ok} 帧，无效/错位 {stats_bad} 段")


def main() -> None:
    p = argparse.ArgumentParser(description="模拟 AA55 电机帧发送与回环读取")
    p.add_argument("--port", default=DEFAULT_PORT, help="串口设备")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率")
    p.add_argument(
        "--once-example",
        action="store_true",
        help="仅打印与注释一致的示例帧（不写串口）",
    )
    p.add_argument(
        "--loopback",
        action="store_true",
        help="随机发送并读取回环数据（需 TX/RX 短接或虚拟串口对）",
    )
    p.add_argument("--interval", type=float, default=0.2, help="随机发送间隔（秒）")
    p.add_argument("--count", type=int, default=20, help="随机发送次数")
    args = p.parse_args()

    if args.once_example:
        ex = example_static_frame()
        print(f"示例帧: {ex.hex()}")
        assert ex.hex() == "aa5506052ee017700f2507"
        print("与 AA 55 06 05 2E E0 17 70 0F 25 07 一致。")
        return

    if args.loopback:
        run_loopback(args.port, args.baud, args.interval, args.count)
        return

    # 默认：发一帧随机数据（仍可从串口读回，若有回环）
    lp = random.uniform(0.0, 100.0)
    rp = random.uniform(0.0, 100.0)
    d = _random_direction_byte()
    pkt = build_frame(lp, rp, d)
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.5,
    )
    try:
        ser.reset_input_buffer()
        ser.write(pkt)
        ser.flush()
        print(f"[TX] {pkt.hex()} | 左{lp:.1f}% 右{rp:.1f}% {format_direction_byte(d)}")
        time.sleep(0.05)
        echo = ser.read(256)
        if echo:
            print(f"[RX] {echo.hex()}")
            if len(echo) >= FRAME_TOTAL:
                parsed = parse_and_verify(echo[:FRAME_TOTAL])
                if parsed:
                    print(
                        f"     解析: 左≈{parsed[0]:.1f}% 右≈{parsed[1]:.1f}% "
                        f"{format_direction_byte(parsed[2])}"
                    )
    finally:
        ser.close()


if __name__ == "__main__":
    main()
