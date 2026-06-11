"""罗盘配置与枚举定义。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum


class OutputMode(IntEnum):
    """DDM350B 输出模式（与协议一致）。"""

    POLLING = 0x00
    AUTO_5HZ = 0x01
    AUTO_15HZ = 0x02
    AUTO_25HZ = 0x03
    AUTO_35HZ = 0x04
    AUTO_50HZ = 0x05
    AUTO_100HZ = 0x06


class Axis(str, Enum):
    """读取时返回的单轴选择。"""

    ROLL = "roll"
    PITCH = "pitch"
    HEADING = "heading"
    ALL = "all"


class HeadingFormat(str, Enum):
    """航向角显示范围。"""

    DEG360 = "0-360"
    DEG180 = "±180"


class NorthReference(str, Enum):
    """航向参考：磁北或真北（真北 = 磁北 + 磁偏角）。"""

    MAGNETIC = "magnetic"
    TRUE = "true"


@dataclass
class KalmanConfig:
    enabled: bool = False
    process_noise: float = 0.001
    measurement_noise: float = 0.1
    adaptive: bool = False
    wrap_heading_in_kf: bool = False


@dataclass
class CompassConfig:
    port: str = "/dev/ttyS0"
    baudrate: int = 115200
    timeout: float = 1.0
    mode: OutputMode = OutputMode.POLLING
    axis: Axis = Axis.ALL
    kalman: KalmanConfig = field(default_factory=KalmanConfig)
    heading_format: HeadingFormat = HeadingFormat.DEG360
    north_reference: NorthReference = NorthReference.MAGNETIC
    magnetic_declination_deg: float = 0.0
    read_timeout: float | None = None
    auto_connect: bool = True

    def effective_read_timeout(self) -> float:
        if self.read_timeout is not None:
            return self.read_timeout
        if self.mode == OutputMode.POLLING:
            return self.timeout
        hz_map = {
            OutputMode.AUTO_5HZ: 5,
            OutputMode.AUTO_15HZ: 15,
            OutputMode.AUTO_25HZ: 25,
            OutputMode.AUTO_35HZ: 35,
            OutputMode.AUTO_50HZ: 50,
            OutputMode.AUTO_100HZ: 100,
        }
        hz = hz_map.get(self.mode, 10)
        return max(0.05, 2.0 / hz)
