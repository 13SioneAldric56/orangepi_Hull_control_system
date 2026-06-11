"""
双电机差速驱动控制模块
适用于 Orange Pi 等 Linux SBC 平台
"""
import time

try:
    from control_car.pwm_sysfs import (
        PWM_PERIOD_HBRIDGE_NS,
        gpio_unexport,
        pwm_release,
        pwm_setup,
        pwm_set_duty_ns,
    )
except ImportError:
    from pwm_sysfs import (
        PWM_PERIOD_HBRIDGE_NS,
        gpio_unexport,
        pwm_release,
        pwm_setup,
        pwm_set_duty_ns,
    )


# ===================== 左轮 引脚定义 =====================
LEFT_MOTOR = {
    "pwm_gpio_num": 50,     # GPIO2_C3  PWM0_M2
    "pwm_chip": "pwmchip2",
    "pwm_dev": "pwm0",
    "in1": 49,             # GPIO2_D4  前进
    "in2": 48,             # GPIO1_C4  后退
}

# ===================== 右轮 引脚定义 =====================
RIGHT_MOTOR = {
    "pwm_gpio_num": 58,     # GPIO1_D2  PWM2_M2
    "pwm_chip": "pwmchip0",
    "pwm_dev": "pwm0",
    "in1": 92,              # GPIO1_D1  前进
    "in2": 52,              # GPIO1_D0  后退
}

PERIOD_NS = PWM_PERIOD_HBRIDGE_NS  # 20kHz；切换前由 pwm_setup 先 unexport


# ===================== GPIO 通用控制函数 =====================
def gpio_export(pin):
    """导出GPIO引脚"""
    try:
        with open("/sys/class/gpio/export", "w") as f:
            f.write(f"{pin}\n")
        time.sleep(0.1)
    except OSError:
        pass

def gpio_set_output(pin):
    """设置引脚为输出模式"""
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")

def gpio_write(pin, level):
    """输出电平"""
    with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
        f.write(f"{level}")


# ===================== PWM 控制函数 =====================
def pwm_init(pwm_chip, pwm_dev, gpio_num=None):
    """初始化 PWM（先 unexport 再按 H 桥 20kHz 写 period）"""
    pwm_setup(
        pwm_chip,
        pwm_dev,
        PERIOD_NS,
        duty_ns=0,
        gpio_num=gpio_num,
        enable=True,
    )


def pwm_set_duty(pwm_chip, pwm_dev, duty_percent):
    """设置PWM占空比"""
    duty_percent = max(0, min(100, duty_percent))
    duty_ns = int(PERIOD_NS * duty_percent / 100)
    pwm_set_duty_ns(pwm_chip, pwm_dev, duty_ns, PERIOD_NS)


def pwm_enable(pwm_chip, pwm_dev):
    """开启PWM"""
    with open(f"/sys/class/pwm/{pwm_chip}/{pwm_dev}/enable", "w") as f:
        f.write("1")


def pwm_disable(pwm_chip, pwm_dev):
    """关闭PWM"""
    with open(f"/sys/class/pwm/{pwm_chip}/{pwm_dev}/enable", "w") as f:
        f.write("0")


# ===================== 单电机控制类 =====================
class Motor:
    """单电机控制类"""

    def __init__(self, config):
        self.pwm_chip = config["pwm_chip"]
        self.pwm_dev = config["pwm_dev"]
        self.pwm_gpio_num = config.get("pwm_gpio_num")
        self.in1 = config["in1"]
        self.in2 = config["in2"]
        self._initialized = False

    def init(self):
        """初始化电机引脚"""
        if self._initialized:
            return
        gpio_export(self.in1)
        gpio_export(self.in2)
        gpio_set_output(self.in1)
        gpio_set_output(self.in2)
        pwm_init(self.pwm_chip, self.pwm_dev, gpio_num=self.pwm_gpio_num)
        self._initialized = True

    def shutdown(self):
        """释放 PWM（便于切换到 ESC 模式）"""
        if not self._initialized:
            return
        pwm_disable(self.pwm_chip, self.pwm_dev)
        pwm_release(self.pwm_chip, self.pwm_dev)
        self._initialized = False

    def forward(self, duty=50):
        """正转"""
        gpio_write(self.in1, 1)
        gpio_write(self.in2, 0)
        pwm_set_duty(self.pwm_chip, self.pwm_dev, duty)

    def backward(self, duty=50):
        """反转"""
        gpio_write(self.in1, 0)
        gpio_write(self.in2, 1)
        pwm_set_duty(self.pwm_chip, self.pwm_dev, duty)

    def coast(self):
        """滑行停止"""
        gpio_write(self.in1, 0)
        gpio_write(self.in2, 0)

    def brake(self):
        """刹车停止"""
        gpio_write(self.in1, 1)
        gpio_write(self.in2, 1)

    def stop(self, brake=True):
        """停止电机"""
        if brake:
            self.brake()
        else:
            self.coast()


# ===================== 差速驱动类 =====================
class DifferentialDrive:
    """
    差速驱动控制器
    实现直行、转向、原地旋转等功能
    """

    # 转弯差速参数配置
    TURN_PROFILES = {
        # 名称: (左轮速度系数, 右轮速度系数, 说明)
        # 系数范围 0.0 ~ 1.0，值越小速度越慢

        # --- 原地旋转（车辆中心为轴）---
        "spin_left":        ( -1.0,  1.0),  # 原地左转
        "spin_right":       (  1.0, -1.0),  # 原地右转

        # --- 原地小旋转 ---
        "原地左小转":       ( -0.3,  0.5),  # 左轮反转慢，右轮正转中速
        "原地右小转":       (  0.5, -0.3),  # 左轮中速正转，右轮反转慢

        # --- 原地中旋转 ---
        "原地左中转":       ( -0.5,  0.8),  # 左轮反转中速，右轮正转快速
        "原地右中转":       (  0.8, -0.5),  # 左轮快速正转，右轮反转中速

        # --- 原地急旋转 ---
        "原地左急转":       ( -0.8,  1.0),  # 左轮反转快速，右轮全速正转
        "原地右急转":       (  1.0, -0.8),  # 左轮全速正转，右轮反转快速

        # --- 差速转弯（小角度，适合轻微修正方向） ---
        "差速左微调":       (  0.5,  1.0),  # 左轮50% 右轮100%
        "差速右微调":       (  1.0,  0.5),  # 左轮100% 右轮50%

        # --- 差速转弯（小角度） ---
        "差速左小弯":       (  0.3,  1.0),  # 左轮30% 右轮100%
        "差速右小弯":       (  1.0,  0.3),  # 左轮100% 右轮30%

        # --- 差速转弯（中角度） ---
        "差速左中弯":       (  0.0,  1.0),  # 左轮停 右轮100%
        "差速右中弯":       (  1.0,  0.0),  # 左轮100% 右轮停

        # --- 差速转弯（大角度，适合窄空间掉头） ---
        "差速左大弯":       ( -0.3,  1.0),  # 左轮反转30% 右轮100%
        "差速右大弯":       (  1.0, -0.3),  # 左轮100% 右轮反转30%

        # --- 差速转弯（急弯，适合原地掉头） ---
        "差速左急弯":       ( -0.5,  1.0),  # 左轮反转50% 右轮100%
        "差速右急弯":       (  1.0, -0.5),  # 左轮100% 右轮反转50%
    }

    def __init__(self, left_motor: Motor, right_motor: Motor, base_speed: int = 50):
        """
        初始化差速驱动控制器

        Args:
            left_motor: 左轮电机实例
            right_motor: 右轮电机实例
            base_speed: 基础速度 (0-100)
        """
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.base_speed = base_speed
        self._is_moving = False

    def init(self):
        """初始化左右两个电机"""
        self.left_motor.init()
        self.right_motor.init()
        print("[DifferentialDrive] 全部电机初始化完成")

    # ==================== 基础运动接口 ====================

    def forward(self, speed: int = None):
        """
        直行

        Args:
            speed: 速度 0-100，默认使用 base_speed
        """
        if speed is None:
            speed = self.base_speed
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)
        self._is_moving = True
        print(f"[直行] 速度: {speed}%")

    def backward(self, speed: int = None):
        """
        后退

        Args:
            speed: 速度 0-100，默认使用 base_speed
        """
        if speed is None:
            speed = self.base_speed
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)
        self._is_moving = True
        print(f"[后退] 速度: {speed}%")

    def stop(self, brake: bool = True):
        """
        停止

        Args:
            brake: True=刹车停止(急停)，False=滑行停止
        """
        if brake:
            self.left_motor.brake()
            self.right_motor.brake()
            print("[停止] 刹车急停")
        else:
            self.left_motor.coast()
            self.right_motor.coast()
            print("[停止] 滑行缓停")
        self._is_moving = False

    # ==================== 差速转弯接口 ====================

    def differential_turn(self, direction: str, speed: int = None):
        """
        使用预设差速参数转弯

        Args:
            direction: 转弯方向名称，可选值:
                - "spin_left": 原地左转
                - "spin_right": 原地右转
                - "原地左小转" / "原地右小转"
                - "原地左中转" / "原地右中转"
                - "原地左急转" / "原地右急转"
                - "差速左微调" / "差速右微调"
                - "差速左小弯" / "差速右小弯"
                - "差速左中弯" / "差速右中弯"
                - "差速左大弯" / "差速右大弯"
                - "差速左急弯" / "差速右急弯"
            speed: 基础速度 0-100，默认使用 base_speed
        """
        if direction not in self.TURN_PROFILES:
            available = ", ".join(self.TURN_PROFILES.keys())
            raise ValueError(f"未知的转弯方向 '{direction}'，可用选项:\n{available}")

        if speed is None:
            speed = self.base_speed

        left_factor, right_factor = self.TURN_PROFILES[direction]

        # 计算实际速度（考虑正反转）
        left_speed = abs(int(speed * left_factor))
        right_speed = abs(int(speed * right_factor))

        # 根据系数符号决定方向
        if left_factor >= 0:
            self.left_motor.forward(left_speed)
        else:
            self.left_motor.backward(left_speed)

        if right_factor >= 0:
            self.right_motor.forward(right_speed)
        else:
            self.right_motor.backward(right_speed)

        self._is_moving = True
        print(f"[差速转弯] {direction} | 左轮: {left_speed}% {'正转' if left_factor >= 0 else '反转'} | 右轮: {right_speed}% {'正转' if right_factor >= 0 else '反转'}")

    def turn_left(self, speed: int = None):
        """左转（使用差速左小弯）"""
        self.differential_turn("差速左小弯", speed)

    def turn_right(self, speed: int = None):
        """右转（使用差速右小弯）"""
        self.differential_turn("差速右小弯", speed)

    def spin_left(self, speed: int = None):
        """原地左旋转"""
        self.differential_turn("spin_left", speed)

    def spin_right(self, speed: int = None):
        """原地右旋转"""
        self.differential_turn("spin_right", speed)

    def custom_turn(self, left_factor: float, right_factor: float, speed: int = None):
        """
        自定义差速转弯

        Args:
            left_factor: 左轮速度系数 (-1.0 ~ 1.0)
                         正值=正转，负值=反转，0=停止
            right_factor: 右轮速度系数 (-1.0 ~ 1.0)
            speed: 基础速度 0-100
        """
        if speed is None:
            speed = self.base_speed

        left_speed = abs(int(speed * left_factor))
        right_speed = abs(int(speed * right_factor))

        if left_factor >= 0:
            self.left_motor.forward(left_speed)
        else:
            self.left_motor.backward(left_speed)

        if right_factor >= 0:
            self.right_motor.forward(right_speed)
        else:
            self.right_motor.backward(right_speed)

        self._is_moving = True
        print(f"[自定义差速] 左轮: {left_speed}% {'+' if left_factor >= 0 else '-'} | 右轮: {right_speed}% {'+' if right_factor >= 0 else '-'}")

    # ==================== 状态查询 ====================

    def is_moving(self) -> bool:
        """查询是否在运动中"""
        return self._is_moving

    def get_turn_profiles(self) -> list:
        """获取所有可用的转弯配置名称"""
        return list(self.TURN_PROFILES.keys())

    def shutdown(self) -> None:
        """停止并 unexport PWM（便于切换到 ESC 模式）"""
        self.stop()
        self.left_motor.shutdown()
        self.right_motor.shutdown()
        print("[DifferentialDrive] H桥 PWM 已释放")


# ===================== 便捷工厂函数 =====================
def create_dual_motor_driver(base_speed: int = 50) -> DifferentialDrive:
    """
    创建双电机差速驱动实例

    Args:
        base_speed: 默认基础速度 (0-100)

    Returns:
        DifferentialDrive 实例
    """
    left = Motor(LEFT_MOTOR)
    right = Motor(RIGHT_MOTOR)
    driver = DifferentialDrive(left, right, base_speed)
    driver.init()
    return driver
