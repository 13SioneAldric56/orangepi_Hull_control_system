"""
sysfs pwmchip 通用操作：切换 H 桥 / ESC 前先 unexport，再按模式写入 period。

H 桥: 20kHz (period 50_000 ns)
ESC:  50Hz  (period 20_000_000 ns)
"""
import time
from typing import Optional

PWM_PERIOD_HBRIDGE_NS = 50_000
PWM_PERIOD_ESC_NS = 20_000_000


def parse_pwm_dev_num(pwm_dev: str) -> int:
    """'pwm0' -> 0, 'pwm1' -> 1"""
    s = pwm_dev.strip().lower()
    if s.startswith("pwm"):
        s = s[3:]
    return int(s)


def pwm_attr_path(pwm_chip: str, pwm_dev: str, attr: str) -> str:
    return f"/sys/class/pwm/{pwm_chip}/{pwm_dev}/{attr}"


def _pwm_write(path: str, val: str, required: bool = True) -> None:
    try:
        with open(path, "w") as f:
            f.write(val)
    except OSError as e:
        if required:
            raise OSError(f"写入失败 {path}={val!r}: {e}") from e


def _pwm_read(path: str) -> Optional[str]:
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except OSError:
        return None


def gpio_unexport(pin: int) -> None:
    try:
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(f"{pin}\n")
    except OSError:
        pass


def pwm_release(pwm_chip: str, pwm_dev: str) -> None:
    """关闭并 unexport 指定 PWM 通道（切换模式前调用）。"""
    dev_num = parse_pwm_dev_num(pwm_dev)
    base = f"/sys/class/pwm/{pwm_chip}/{pwm_dev}"
    if _pwm_read(f"{base}/enable") is not None:
        _pwm_write(f"{base}/enable", "0", required=False)
        time.sleep(0.05)
    try:
        with open(f"/sys/class/pwm/{pwm_chip}/unexport", "w") as f:
            f.write(f"{dev_num}\n")
    except OSError:
        pass
    time.sleep(0.15)


def pwm_setup(
    pwm_chip: str,
    pwm_dev: str,
    period_ns: int,
    duty_ns: int = 0,
    *,
    gpio_num: Optional[int] = None,
    enable: bool = True,
) -> None:
    """
    完整初始化：unexport → export → period → duty_cycle → enable。

    Rockchip 等驱动在「新 export」后必须先写 period，再写 duty，最后 enable；
    在 period 未配置前写 enable=0 会 EINVAL。
    """
    if gpio_num is not None:
        gpio_unexport(gpio_num)

    pwm_release(pwm_chip, pwm_dev)

    dev_num = parse_pwm_dev_num(pwm_dev)
    export_path = f"/sys/class/pwm/{pwm_chip}/export"
    period_path = pwm_attr_path(pwm_chip, pwm_dev, "period")

    if _pwm_read(period_path) is None:
        try:
            with open(export_path, "w") as f:
                f.write(f"{dev_num}\n")
        except OSError as e:
            raise OSError(
                f"无法 export {pwm_chip} 通道 {dev_num} ({pwm_dev}): {e}"
            ) from e
        time.sleep(0.2)
        if _pwm_read(period_path) is None:
            raise FileNotFoundError(
                f"export 后未找到 {pwm_chip}/{pwm_dev}，请检查 npwm 与 pwm_dev 配置"
            )
    else:
        # 通道已存在（export 未释放干净）：先关输出再改 period
        _pwm_write(pwm_attr_path(pwm_chip, pwm_dev, "enable"), "0", required=False)
        time.sleep(0.05)

    duty_ns = max(0, min(period_ns, int(duty_ns)))
    _pwm_write(period_path, str(period_ns))
    _pwm_write(pwm_attr_path(pwm_chip, pwm_dev, "duty_cycle"), str(duty_ns))
    try:
        _pwm_write(
            pwm_attr_path(pwm_chip, pwm_dev, "polarity"), "normal", required=False
        )
    except OSError:
        pass
    if enable:
        _pwm_write(pwm_attr_path(pwm_chip, pwm_dev, "enable"), "1")
    else:
        _pwm_write(pwm_attr_path(pwm_chip, pwm_dev, "enable"), "0", required=False)


def pwm_set_duty_ns(
    pwm_chip: str, pwm_dev: str, duty_ns: int, period_ns: int
) -> None:
    duty_ns = max(0, min(period_ns, int(duty_ns)))
    _pwm_write(pwm_attr_path(pwm_chip, pwm_dev, "duty_cycle"), str(duty_ns))
