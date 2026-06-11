"""
双路电调差速驱动（水面推进器正反转，无 H 桥）
接口与 dual_motor_control.DifferentialDrive 对齐，供键盘遥控复用。
底层为 GPIO 软件 PWM（esc_motor_control）。
"""
try:
    from control_car.esc_motor_control import (
        EscMotor,
        create_left_esc,
        create_right_esc,
        unlock_dual_esc,
    )
except ImportError:
    from esc_motor_control import (
        EscMotor,
        create_left_esc,
        create_right_esc,
        unlock_dual_esc,
    )


class EscDifferentialDrive:
    """双电调差速控制器"""

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
        left_motor: EscMotor,
        right_motor: EscMotor,
        base_speed: int = 50,
    ):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.base_speed = base_speed
        self._is_moving = False

    def init(self) -> None:
        self.left_motor.init()
        self.right_motor.init()
        print("[EscDifferentialDrive] 双路软件 PWM 初始化完成")

    def _apply_side(self, motor: EscMotor, factor: float, speed: int) -> None:
        wheel_speed = abs(int(speed * factor))
        if factor >= 0:
            motor.forward(wheel_speed)
        else:
            motor.reverse(wheel_speed)

    def forward(self, speed: int = None) -> None:
        if speed is None:
            speed = self.base_speed
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)
        self._is_moving = True
        print(f"[直行] 速度: {speed}%")

    def backward(self, speed: int = None) -> None:
        if speed is None:
            speed = self.base_speed
        self.left_motor.reverse(speed)
        self.right_motor.reverse(speed)
        self._is_moving = True
        print(f"[后退] 速度: {speed}%")

    def stop(self, brake: bool = True) -> None:
        self.left_motor.stop()
        self.right_motor.stop()
        self._is_moving = False
        print("[停止] 中位 7.5%")

    def differential_turn(self, direction: str, speed: int = None) -> None:
        if direction not in self.TURN_PROFILES:
            available = ", ".join(self.TURN_PROFILES.keys())
            raise ValueError(f"未知的转弯方向 '{direction}'，可用选项:\n{available}")

        if speed is None:
            speed = self.base_speed

        left_factor, right_factor = self.TURN_PROFILES[direction]
        left_speed = abs(int(speed * left_factor))
        right_speed = abs(int(speed * right_factor))

        self._apply_side(self.left_motor, left_factor, speed)
        self._apply_side(self.right_motor, right_factor, speed)
        self._is_moving = True
        print(
            f"[差速转弯] {direction} | 左: {left_speed}% "
            f"{'正转' if left_factor >= 0 else '反转'} | 右: {right_speed}% "
            f"{'正转' if right_factor >= 0 else '反转'}"
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
        self._is_moving = True

    def is_moving(self) -> bool:
        return self._is_moving

    def get_turn_profiles(self) -> list:
        return list(self.TURN_PROFILES.keys())

    def shutdown(self) -> None:
        self.stop()
        self.left_motor.shutdown()
        self.right_motor.shutdown()


def create_esc_dual_driver(
    base_speed: int = 50, auto_unlock: bool = True
) -> EscDifferentialDrive:
    left = create_left_esc()
    right = create_right_esc()
    driver = EscDifferentialDrive(left, right, base_speed)
    driver.init()
    if auto_unlock:
        unlock_dual_esc(left, right)
    return driver
