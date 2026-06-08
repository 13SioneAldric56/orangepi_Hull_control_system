"""
SSH / 终端键盘遥控 — UART3 双轮（命令码 0x06）

控制逻辑与 keyboard_remote_stdin.py 相同；速度经串口发出。
每 50ms 检测一次：若无新的 WASD 输入则发送 0x00 停转帧。

用法: python3 keyboard_remote_uart_stdin.py [--uart-port /dev/ttyUSB1]
"""
from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import time
from pathlib import Path
from typing import Optional, Set

_car_dir = Path(__file__).resolve().parent
if str(_car_dir) not in sys.path:
    sys.path.insert(0, str(_car_dir))

from uart_dual_drive import UartDifferentialDrive, create_uart_dual_driver

MOTION_KEYS = frozenset({"w", "s", "a", "d"})
# 50ms 内无新的 WASD 输入则视为松键；轮询周期同步为 50ms
KEY_IDLE_SEC = 0.05
POLL_SEC = 0.05
STOP_DEBOUNCE_SEC = 0.0


def apply_motion(drive: UartDifferentialDrive, active: Set[str]) -> None:
    w, s, a, d = "w" in active, "s" in active, "a" in active, "d" in active

    if w and s:
        drive.stop()
        return
    if w:
        if a and not d:
            drive.differential_turn("差速左小弯")
        elif d and not a:
            drive.differential_turn("差速右小弯")
        else:
            drive.forward()
        return
    if s:
        drive.backward()
        return
    if a and d:
        drive.stop()
        return
    if a:
        drive.spin_left()
        return
    if d:
        drive.spin_right()
        return
    drive.stop()


def _compute_sig(active: Set[str]) -> tuple:
    w, s, a, d = "w" in active, "s" in active, "a" in active, "d" in active
    if w and s:
        return ("clash",)
    if w:
        if a ^ d:
            return ("w_turn", "L" if a else "R")
        return ("fwd",)
    if s:
        return ("back",)
    if a ^ d:
        return ("spin", "L" if a else "R")
    if not active & MOTION_KEYS:
        return ("stop",)
    return ("stop",)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="终端 WASD UART3 双轮遥控 (0x06)")
    p.add_argument("--speed", "-s", type=int, default=50, help="初始速度 0～100")
    p.add_argument(
        "--uart-port", default="/dev/ttyUSB1", help="电机 UART 设备，默认 /dev/ttyUSB1"
    )
    p.add_argument("--uart-baud", type=int, default=115200, help="波特率，默认 115200")
    p.add_argument(
        "--test-tx",
        action="store_true",
        help="仅测试串口发送（停→前进→停），不进入键盘循环",
    )
    p.add_argument(
        "--no-debug-tx",
        action="store_true",
        help="关闭每次发帧的十六进制日志",
    )
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    base_speed = max(0, min(100, args.speed))

    drive: Optional[UartDifferentialDrive] = None
    fd = sys.stdin.fileno()
    old_term: Optional[list] = None

    def restore_tty() -> None:
        nonlocal old_term
        if old_term is not None:
            try:
                import termios

                termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)
            except (OSError, termios.error):
                pass
            old_term = None

    def on_signal(_sig, _frame) -> None:
        if drive is not None:
            drive.stop()
        restore_tty()
        sys.exit(0)

    try:
        import termios
        import tty
    except ImportError:
        print("需要 Unix 终端（termios/tty），当前环境不可用。")
        sys.exit(1)

    print(f"初始化 UART 电机驱动 {args.uart_port} @ {args.uart_baud} …")
    drive = create_uart_dual_driver(
        port=args.uart_port,
        baud=args.uart_baud,
        base_speed=base_speed,
        debug_tx=not args.no_debug_tx,
    )
    drive.base_speed = base_speed

    if args.test_tx:
        import time as _time

        print("[test-tx] 停 → 前进 50% → 停（各间隔 0.5s）")
        drive.stop()
        _time.sleep(0.5)
        drive.forward()
        _time.sleep(0.5)
        drive.stop()
        drive.shutdown()
        print("[test-tx] 完成，请用逻辑分析仪/示波器查看 TX 引脚")
        return

    last_seen: dict[str, float] = {}
    prev_sigint = signal.signal(signal.SIGINT, on_signal)

    old_term = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    if not sys.stdin.isatty():
        print(
            "[警告] stdin 不是交互式 TTY，键盘可能无效。",
            file=sys.stderr,
        )

    print(
        f"UART 键盘遥控 | {args.uart_port} | W/S 前进/后退 | A/D 原地转\n"
        "空格/x 停 | +/- 调速 | q 退出\n"
        f"base_speed={drive.base_speed}% | 松键检测={int(KEY_IDLE_SEC * 1000)}ms"
    )

    last_cmd: Optional[tuple] = None
    stop_idle_since: Optional[float] = None

    try:
        while True:
            speed_dirty = False

            if select.select([fd], [], [], POLL_SEC)[0]:
                try:
                    chunk_b = os.read(fd, 4096)
                except OSError:
                    chunk_b = b""
                if not chunk_b:
                    break
                chunk = chunk_b.decode("utf-8", errors="ignore")
                ts = time.monotonic()
                for ch in chunk:
                    cl = ch.lower()
                    if cl in ("q",):
                        raise SystemExit(0)
                    if ch == " " or cl == "x":
                        last_seen.clear()
                        stop_idle_since = None
                        if last_cmd != ("stop",):
                            drive.stop()
                            last_cmd = ("stop",)
                        continue
                    if cl == "+" or cl == "=":
                        drive.base_speed = min(100, drive.base_speed + 5)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl == "-" or cl == "_":
                        drive.base_speed = max(0, drive.base_speed - 5)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if ch == "[":
                        drive.base_speed = max(0, drive.base_speed - 10)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if ch == "]":
                        drive.base_speed = min(100, drive.base_speed + 10)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl in "123456789":
                        drive.base_speed = min(100, (ord(cl) - ord("0")) * 10)
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl == "0":
                        drive.base_speed = 100
                        speed_dirty = True
                        print(f"[速度] base_speed={drive.base_speed}%")
                        continue
                    if cl in MOTION_KEYS:
                        last_seen[cl] = ts

            now = time.monotonic()
            active = {
                k
                for k, t in last_seen.items()
                if k in MOTION_KEYS and (now - t) <= KEY_IDLE_SEC
            }
            sig = _compute_sig(active)

            if sig != ("stop",):
                stop_idle_since = None
                if speed_dirty or sig != last_cmd:
                    apply_motion(drive, active)
                    last_cmd = sig
                continue

            if speed_dirty:
                speed_dirty = False
            if stop_idle_since is None:
                stop_idle_since = now
            if now - stop_idle_since < STOP_DEBOUNCE_SEC:
                continue

            if last_cmd is None:
                last_cmd = ("stop",)
            elif last_cmd != ("stop",):
                apply_motion(drive, active)
                last_cmd = ("stop",)

    finally:
        signal.signal(signal.SIGINT, prev_sigint)
        if drive is not None:
            drive.stop()
            drive.shutdown()
        restore_tty()


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
