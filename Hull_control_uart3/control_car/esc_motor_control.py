"""
电调（ESC）无刷电机控制模块 — GPIO 软件 PWM（sysfs）
适用于 APISQUEEN 等标准 50Hz 舵机协议电调

双路默认: 左 GPIO50 / 右 GPIO58（与 dual_motor_control 引脚一致）
初始化前会释放同脚硬件 pwmchip，避免与 GPIO 冲突。

说明：
  - 直接 export GPIO 输出 50Hz 脉宽调制
  - 文档「占空比」= range1000 档位，实际脉宽 = 档位/1000 × 20ms
  - 中位 75 = 1.5ms（解锁/停转）
"""
import threading
import time
from typing import Optional

try:
    from control_car.pwm_sysfs import pwm_release
except ImportError:
    from pwm_sysfs import pwm_release

# ===================== 引脚定义 =====================
ESC_MOTOR = {
    "pwm_gpio_num": 54,     # GPIO2_C3
}

PWM_CHIP_RELEASE = "pwmchip4"   # 同脚硬件 PWM，初始化前须释放
PWM_DEV_RELEASE = "pwm0"

PWM_FREQUENCY = 50          # Hz，周期 20ms
PERIOD_US = 20_000
PWM_RANGE = 1000

PULSE_STOP = 75             # 1.5ms 停转/解锁
PULSE_FORWARD_MIN = 75
PULSE_FORWARD_MAX = 100     # 2.0ms 正转最快
PULSE_REVERSE_MIN = 50      # 1.0ms 反转最快
PULSE_REVERSE_MAX = 75
UNLOCK_HOLD_SEC = 3.0


def pulse_to_us(pulse_value: int) -> int:
    pulse_value = max(0, min(PWM_RANGE, int(pulse_value)))
    return int(PERIOD_US * pulse_value / PWM_RANGE)


def pulse_to_ms(pulse_value: int) -> float:
    return pulse_to_us(pulse_value) / 1000.0


def pulse_to_duty_percent(pulse_value: int) -> float:
    """相对 20ms 周期的高电平占空比（%），供示波器对照。"""
    return pulse_value / PWM_RANGE * 100.0


def _delay_us(us: float) -> None:
    if us <= 0:
        return
    end = time.perf_counter() + us / 1_000_000
    while time.perf_counter() < end:
        pass


def release_pwmchip(pwm_chip: str = PWM_CHIP_RELEASE, pwm_dev: str = PWM_DEV_RELEASE) -> None:
    """关闭并释放同脚硬件 PWM，避免与 GPIO 冲突。"""
    enable_path = f"/sys/class/pwm/{pwm_chip}/{pwm_dev}/enable"
    try:
        with open(enable_path, "w") as f:
            f.write("0")
    except OSError:
        pass
    try:
        with open(f"/sys/class/pwm/{pwm_chip}/unexport", "w") as f:
            f.write("0\n" if "pwm0" in pwm_dev else "1\n")
    except OSError:
        pass


def gpio_export(pin: int) -> None:
    try:
        with open("/sys/class/gpio/export", "w") as f:
            f.write(f"{pin}\n")
        time.sleep(0.1)
    except OSError:
        pass


def gpio_unexport(pin: int) -> None:
    try:
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(f"{pin}\n")
    except OSError:
        pass


def gpio_set_output(pin: int) -> None:
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")


class _SoftPwmEngine:
    """单线程多路 50Hz GPIO 软件 PWM，避免双线程抢 CPU 造成脉宽抖动。"""

    _lock = threading.Lock()
    _channels: dict = {}
    _pending: dict = {}
    _thread: Optional[threading.Thread] = None
    _stop = threading.Event()

    @classmethod
    def register(
        cls,
        pin: int,
        pwm_chip: Optional[str] = None,
        pwm_dev: str = "pwm0",
    ) -> None:
        with cls._lock:
            if pin in cls._channels:
                return
            if pwm_chip:
                pwm_release(pwm_chip, pwm_dev)
            elif pin == ESC_MOTOR["pwm_gpio_num"]:
                release_pwmchip()
            gpio_export(pin)
            gpio_set_output(pin)
            vf = open(f"/sys/class/gpio/gpio{pin}/value", "w")
            cls._channels[pin] = {
                "vf": vf,
                "pulse_us": pulse_to_us(PULSE_STOP),
            }
            cls._pending[pin] = None
            if cls._thread is None or not cls._thread.is_alive():
                cls._stop.clear()
                cls._thread = threading.Thread(target=cls._run, daemon=True)
                cls._thread.start()

    @classmethod
    def set_pulse(cls, pin: int, pulse_value: int) -> None:
        with cls._lock:
            if pin not in cls._channels:
                return
            cls._pending[pin] = pulse_to_us(pulse_value)

    @classmethod
    def _write_level(cls, ch: dict, level: int) -> float:
        t0 = time.perf_counter()
        ch["vf"].write("1" if level else "0")
        ch["vf"].flush()
        return (time.perf_counter() - t0) * 1_000_000

    @classmethod
    def _delay_us(cls, us: float) -> None:
        if us <= 0:
            return
        end = time.perf_counter() + us / 1_000_000
        while time.perf_counter() < end:
            pass

    @classmethod
    def _run(cls) -> None:
        period_us = PERIOD_US
        while not cls._stop.is_set():
            cycle_start = time.perf_counter()

            with cls._lock:
                for pin, pending in cls._pending.items():
                    if pending is not None:
                        cls._channels[pin]["pulse_us"] = pending
                        cls._pending[pin] = None
                snap = {
                    pin: ch["pulse_us"] for pin, ch in cls._channels.items()
                }
                channels = dict(cls._channels)

            if not channels:
                time.sleep(0.01)
                continue

            for ch in channels.values():
                cls._write_level(ch, 1)

            fall_events = sorted((snap[pin], pin) for pin in snap)
            prev_us = 0.0
            for target_us, pin in fall_events:
                cls._delay_us(max(0.0, target_us - prev_us))
                if cls._stop.is_set():
                    break
                cls._write_level(channels[pin], 0)
                prev_us = target_us

            elapsed_us = (time.perf_counter() - cycle_start) * 1_000_000
            cls._delay_us(max(0.0, period_us - elapsed_us))

    @classmethod
    def unregister(cls, pin: int) -> None:
        with cls._lock:
            ch = cls._channels.pop(pin, None)
            cls._pending.pop(pin, None)
            if ch:
                try:
                    ch["vf"].write("0")
                    ch["vf"].flush()
                    ch["vf"].close()
                except OSError:
                    pass
                gpio_unexport(pin)
            if not cls._channels:
                cls._stop.set()
                if cls._thread and cls._thread.is_alive():
                    cls._thread.join(timeout=1.0)
                cls._thread = None


class GpioSoftPwm:
    """GPIO sysfs 软件 PWM（50Hz 舵机/电调协议）"""

    def __init__(self, pin: int, frequency: int = PWM_FREQUENCY):
        self.pin = pin
        self.period_us = int(1_000_000 / frequency)
        self._registered = False

    def init(
        self,
        pwm_chip: Optional[str] = None,
        pwm_dev: str = "pwm0",
    ) -> None:
        if self._registered:
            return
        _SoftPwmEngine.register(self.pin, pwm_chip, pwm_dev)
        self._registered = True

    def set_pulse(self, pulse_value: int) -> None:
        _SoftPwmEngine.set_pulse(self.pin, pulse_value)

    def stop(self) -> None:
        if not self._registered:
            return
        _SoftPwmEngine.unregister(self.pin)
        self._registered = False


class EscMotor:
    """电调控制（GPIO 软件 PWM）"""

    def __init__(self, config: Optional[dict] = None, label: str = ""):
        cfg = config or ESC_MOTOR
        self.pwm_gpio_num = cfg["pwm_gpio_num"]
        self.pwm_chip = cfg.get("pwm_chip")
        self.pwm_dev = cfg.get("pwm_dev", "pwm0")
        self.label = label or f"GPIO{self.pwm_gpio_num}"
        self._pwm = GpioSoftPwm(self.pwm_gpio_num)
        self._initialized = False
        self._unlocked = False
        self._pulse = PULSE_STOP

    def init(self) -> None:
        if self._initialized:
            return
        self._pwm.init(self.pwm_chip, self.pwm_dev)
        self._pwm.set_pulse(PULSE_STOP)
        self._initialized = True
        print(
            f"[{self.label}] GPIO{self.pwm_gpio_num} "
            f"软件PWM {PWM_FREQUENCY}Hz 中位={pulse_to_duty_percent(PULSE_STOP):.1f}%"
        )

    def _apply_pulse(self, pulse_value: int, log: bool = True) -> None:
        pulse_value = max(0, min(PWM_RANGE, int(pulse_value)))
        if pulse_value == self._pulse and self._initialized:
            return
        self._pulse = pulse_value
        if self._initialized:
            self._pwm.set_pulse(pulse_value)
        if log:
            print(
                f"[{self.label}] 档位={pulse_value} "
                f"脉宽={pulse_to_ms(pulse_value):.3f}ms "
                f"占空比={pulse_to_duty_percent(pulse_value):.1f}%"
            )

    def set_pulse(self, pulse_value: int) -> None:
        self.init()
        self._apply_pulse(pulse_value)

    def unlock(self, hold_sec: float = UNLOCK_HOLD_SEC) -> None:
        self.init()
        print(f"[{self.label}] 解锁，中位 {hold_sec}s ...")
        self._apply_pulse(PULSE_STOP, log=False)
        time.sleep(hold_sec)
        self._unlocked = True
        print(f"[{self.label}] 解锁完成")

    def stop(self) -> None:
        if self._initialized:
            self._apply_pulse(PULSE_STOP, log=False)

    def forward(self, speed: int = 50) -> None:
        self.init()
        speed = max(0, min(100, speed))
        pulse = PULSE_FORWARD_MIN + int(
            speed * (PULSE_FORWARD_MAX - PULSE_FORWARD_MIN) / 100
        )
        self._apply_pulse(pulse, log=False)

    def reverse(self, speed: int = 50) -> None:
        self.init()
        speed = max(0, min(100, speed))
        pulse = PULSE_REVERSE_MAX - int(
            speed * (PULSE_REVERSE_MAX - PULSE_REVERSE_MIN) / 100
        )
        self._apply_pulse(pulse, log=False)

    def shutdown(self) -> None:
        if not self._initialized:
            return
        self.stop()
        time.sleep(0.05)
        self._pwm.stop()
        gpio_unexport(self.pwm_gpio_num)
        self._initialized = False
        print(f"[{self.label}] GPIO 软件 PWM 已关闭")


def create_esc_motor(config: dict = None, auto_unlock: bool = True) -> EscMotor:
    motor = EscMotor(config)
    motor.init()
    if auto_unlock:
        motor.unlock()
    return motor


def unlock_dual_esc(
    left: EscMotor,
    right: EscMotor,
    hold_sec: float = UNLOCK_HOLD_SEC,
) -> None:
    """两路同时输出中位并保持 hold_sec 秒（电调解锁）。"""
    left.init()
    right.init()
    print(f"[ESC] 双路同时解锁，中位 {hold_sec}s ...")
    left._apply_pulse(PULSE_STOP, log=False)
    right._apply_pulse(PULSE_STOP, log=False)
    time.sleep(hold_sec)
    left._unlocked = True
    right._unlocked = True
    print("[ESC] 双路解锁完成")


def create_left_esc() -> EscMotor:
    try:
        from control_car.dual_motor_control import LEFT_MOTOR
    except ImportError:
        from dual_motor_control import LEFT_MOTOR
    return EscMotor(LEFT_MOTOR, label="左电调")


def create_right_esc() -> EscMotor:
    try:
        from control_car.dual_motor_control import RIGHT_MOTOR
    except ImportError:
        from dual_motor_control import RIGHT_MOTOR
    return EscMotor(RIGHT_MOTOR, label="右电调")
