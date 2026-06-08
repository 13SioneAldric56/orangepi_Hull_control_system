"""
双轮 UART 差速驱动 — /dev/ttyUSB1，命令码 0x06（11 字节档位帧）

接口与 esc_dual_drive.EscDifferentialDrive 对齐，供航向锁 / 键盘遥控复用。
不占用 pwmchip；每次速度变化通过串口发帧。
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

try:
    import serial
except ImportError:
    serial = None

try:
    from control_car.uart_motor_protocol import (
        COMPARE_MAP,
        build_frame_v06,
        speed_to_index,
    )
except ImportError:
    from uart_motor_protocol import COMPARE_MAP, build_frame_v06, speed_to_index

DEFAULT_UART_PORT = "/dev/ttyUSB1"
DEFAULT_BAUD = 115200
DEFAULT_MIN_SEND_INTERVAL = 0.05
DEFAULT_DEBUG_TX = True


class UartMotor:
    """单轮 UART 速度控制（由 UartDifferentialDrive 合并发帧）。"""

    def __init__(self, owner: "UartDifferentialDrive", side: str, label: str = ""):
        self._owner = owner
        self._side = side  # 'left' | 'right'
        self.label = label
        self._last_index = 0x00

    def _set_state(self, index: int) -> None:
        self._last_index = index & 0xFF
        if self._side == "right":
            self._owner._commit_motors()

    def forward(self, speed: int = 50) -> None:
        self._set_state(speed_to_index(speed, True))

    def reverse(self, speed: int = 50) -> None:
        self._set_state(speed_to_index(speed, False))

    def stop(self) -> None:
        self._set_state(0x00)

    def get_index(self) -> int:
        return self._last_index

    def shutdown(self) -> None:
        self.stop()
        self._owner._commit_motors(force=True)


class UartDifferentialDrive:
    """双轮 UART 差速控制器"""

    TURN_PROFILES = {
        "spin_left": (-1.0, 1.0),
        "spin_right": (1.0, -1.0),
        "原地左小转": (-0.3, 0.5),
        "原地右小转": (0.5, -0.3),
        "原地左中转": (-0.5, 0.8),
        "原地右中转": (0.8, -0.5),
        "原地左急转": (-0.8, 1.0),
        "原地右急转": (1.0, -0.8),
        "差速左微调": (0.5, 1.0),
        "差速右微调": (1.0, 0.5),
        "差速左小弯": (0.3, 1.0),
        "差速右小弯": (1.0, 0.3),
        "差速左中弯": (0.0, 1.0),
        "差速右中弯": (1.0, 0.0),
        "差速左大弯": (-0.3, 1.0),
        "差速右大弯": (1.0, -0.3),
        "差速左急弯": (-0.5, 1.0),
        "差速右急弯": (1.0, -0.5),
    }

    def __init__(
        self,
        port: str = DEFAULT_UART_PORT,
        baud: int = DEFAULT_BAUD,
        base_speed: int = 50,
        min_send_interval: float = DEFAULT_MIN_SEND_INTERVAL,
        debug_tx: bool = DEFAULT_DEBUG_TX,
    ):
        if serial is None:
            raise ImportError("需要 pyserial: pip install pyserial")
        self.port = port
        self.baud = baud
        self.base_speed = base_speed
        self.min_send_interval = max(0.0, float(min_send_interval))
        self.debug_tx = debug_tx
        self.left_motor = UartMotor(self, "left", label="左轮")
        self.right_motor = UartMotor(self, "right", label="右轮")
        self._ser: Optional[serial.Serial] = None
        self._is_moving = False
        self._last_sent: Optional[Tuple[int, int]] = None
        self._last_send_time = 0.0
        self._serial_ok = True

    def _invalidate_serial(self) -> None:
        """关闭失效串口，避免对已断开设备反复 write。"""
        self._serial_ok = False
        self._last_sent = None
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None

    def init(self) -> None:
        if self._ser and self._ser.is_open:
            self._serial_ok = True
            return
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
        self._ser.reset_input_buffer()
        self._serial_ok = True
        print(f"[UartDrive] 已打开 {self.port} @ {self.baud}")
        self.stop()

    def _flush_send(self, left_idx: int, right_idx: int, force: bool = False) -> bool:
        if not self._ser or not self._ser.is_open:
            return False
        pair = (left_idx & 0xFF, right_idx & 0xFF)
        now = time.monotonic()
        if (
            not force
            and pair == self._last_sent
            and (now - self._last_send_time) < self.min_send_interval
        ):
            return True
        pkt = build_frame_v06(pair[0], pair[1])
        try:
            n = self._ser.write(pkt)
            self._ser.flush()
        except (serial.SerialException, OSError) as exc:
            print(f"[UartDrive] 串口发送失败: {exc}", flush=True)
            self._invalidate_serial()
            return False
        if n != len(pkt):
            print(
                f"[UartDrive] 警告: 仅写入 {n}/{len(pkt)} 字节",
                flush=True,
            )
        if self.debug_tx:
            hex_str = " ".join(f"{b:02X}" for b in pkt)
            print(
                f"[UartDrive TX] {self.port} L=0x{pair[0]:02X} R=0x{pair[1]:02X} | {hex_str}",
                flush=True,
            )
        self._last_sent = pair
        self._last_send_time = now
        self._serial_ok = True
        return True

    def _apply_side(self, motor: UartMotor, factor: float, speed: int) -> None:
        wheel_speed = abs(int(speed * factor))
        if wheel_speed <= 0:
            motor.stop()
        elif factor >= 0:
            motor.forward(wheel_speed)
        else:
            motor.reverse(wheel_speed)

    def _commit_motors(self, force: bool = False) -> None:
        self._flush_send(
            self.left_motor.get_index(),
            self.right_motor.get_index(),
            force=force,
        )

    def forward(self, speed: int = None) -> None:
        if speed is None:
            speed = self.base_speed
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)
        self._commit_motors()
        self._is_moving = True
        print(f"[直行] UART 速度: {speed}%")

    def backward(self, speed: int = None) -> None:
        if speed is None:
            speed = self.base_speed
        self.left_motor.reverse(speed)
        self.right_motor.reverse(speed)
        self._commit_motors()
        self._is_moving = True
        print(f"[后退] UART 速度: {speed}%")

    def stop(self, brake: bool = True) -> None:
        self.left_motor._last_index = 0x00
        self.right_motor._last_index = 0x00
        if not self._flush_send(0x00, 0x00, force=True):
            print("[停止] UART 停止帧未发出（串口不可用）")
        self._is_moving = False
        if self._serial_ok:
            print("[停止] UART 档位 0x00")

    def differential_turn(self, direction: str, speed: int = None) -> None:
        if direction not in self.TURN_PROFILES:
            available = ", ".join(self.TURN_PROFILES.keys())
            raise ValueError(f"未知的转弯方向 '{direction}'，可用选项:\n{available}")
        if speed is None:
            speed = self.base_speed
        left_factor, right_factor = self.TURN_PROFILES[direction]
        self._apply_side(self.left_motor, left_factor, speed)
        self._apply_side(self.right_motor, right_factor, speed)
        self._commit_motors()
        self._is_moving = True
        li, ri = self.left_motor.get_index(), self.right_motor.get_index()
        print(
            f"[差速转弯] {direction} | 左档位 0x{li:02X}({COMPARE_MAP.get(li, '?')}) "
            f"右档位 0x{ri:02X}({COMPARE_MAP.get(ri, '?')})"
        )

    def turn_left(self, speed: int = None) -> None:
        self.differential_turn("差速左小弯", speed)

    def turn_right(self, speed: int = None) -> None:
        self.differential_turn("差速右小弯", speed)

    def spin_left(self, speed: int = None) -> None:
        self.differential_turn("spin_left", speed)

    def spin_right(self, speed: int = None) -> None:
        self.differential_turn("spin_right", speed)

    def custom_turn(
        self, left_factor: float, right_factor: float, speed: int = None
    ) -> None:
        if speed is None:
            speed = self.base_speed
        self._apply_side(self.left_motor, left_factor, speed)
        self._apply_side(self.right_motor, right_factor, speed)
        self._commit_motors()
        self._is_moving = True

    def is_moving(self) -> bool:
        return self._is_moving

    def get_turn_profiles(self) -> list:
        return list(self.TURN_PROFILES.keys())

    def shutdown(self) -> None:
        try:
            self.stop()
        except (OSError, serial.SerialException):
            pass
        self._invalidate_serial()
        print("[UartDrive] 串口已关闭")


def create_uart_dual_driver(
    port: str = DEFAULT_UART_PORT,
    baud: int = DEFAULT_BAUD,
    base_speed: int = 50,
    min_send_interval: float = DEFAULT_MIN_SEND_INTERVAL,
    debug_tx: bool = DEFAULT_DEBUG_TX,
) -> UartDifferentialDrive:
    driver = UartDifferentialDrive(
        port=port,
        baud=baud,
        base_speed=base_speed,
        min_send_interval=min_send_interval,
        debug_tx=debug_tx,
    )
    driver.init()
    return driver
