import serial
import time
import threading
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass
from datetime import datetime
import re


@dataclass
class GPSSatellite:
    """卫星信息"""
    prn: int  # 卫星PRN号
    elevation: float  # 仰角(度)
    azimuth: float  # 方位角(度)
    snr: float  # 信噪比(dB)


@dataclass
class GPSPosition:
    """GPS定位信息"""
    latitude: float  # 纬度(度)
    longitude: float  # 经度(度)
    altitude: float  # 海拔高度(米)
    timestamp: datetime  # 时间戳
    fix_quality: int  # 定位质量(0=无效,1=GPS,2=DGPS,4=RTK)
    num_satellites: int  # 使用的卫星数
    hdop: float  # 水平精度因子
    vdop: float  # 垂直精度因子
    pdop: float  # 位置精度因子
    speed: float  # 速度(节)
    course: float  # 航向(度)
    satellites: list  # 卫星列表


class NMEAParser:
    """NMEA 4.11协议解析器"""

    # NMEA语句类型
    SENTENCE_TYPES = {
        'GPGGA': '_parse_gga', 'GAGGA': '_parse_gga', 'GBGGA': '_parse_gga', 'GNGGA': '_parse_gga',
        'GPGSA': '_parse_gsa', 'GAGSA': '_parse_gsa', 'GBGSA': '_parse_gsa', 'GNGSA': '_parse_gsa',
        'GPGSV': '_parse_gsv', 'GAGSV': '_parse_gsv', 'GBGSV': '_parse_gsv', 'GQGSV': '_parse_gsv', 'GLGSV': '_parse_gsv',
        'GPRMC': '_parse_rmc', 'GARMC': '_parse_rmc', 'GBRMC': '_parse_rmc', 'GNRMC': '_parse_rmc',
        'GPVTG': '_parse_vtg', 'GAVTG': '_parse_vtg', 'GBVTG': '_parse_vtg', 'GNVTG': '_parse_vtg',
        'GPGLL': '_parse_gll', 'GAGLL': '_parse_gll', 'GBGLL': '_parse_gll', 'GNGLL': '_parse_gll',
        'GPTXT': '_parse_txt', 'GATXT': '_parse_txt', 'GBTXT': '_parse_txt', 'GNTXT': '_parse_txt',
    }

    def __init__(self):
        self.position = GPSPosition(
            latitude=0.0,
            longitude=0.0,
            altitude=0.0,
            timestamp=datetime.now(),
            fix_quality=0,
            num_satellites=0,
            hdop=0.0,
            vdop=0.0,
            pdop=0.0,
            speed=0.0,
            course=0.0,
            satellites=[]
        )
        self.satellites = []
        self._gsv_cache = {}  # 缓存GSV数据

    @staticmethod
    def _verify_checksum(sentence: str) -> bool:
        """验证NMEA语句校验和"""
        if '*' not in sentence:
            return False

        data, checksum = sentence.rsplit('*', 1)
        data = data.strip()
        if data[0] != '$':
            return False

        # 计算校验和（从$后第一个字符开始，异或所有字符）
        calc_checksum = 0
        for char in data[1:]:
            calc_checksum ^= ord(char)

        return f"{calc_checksum:02X}" == checksum.strip().upper()

    @staticmethod
    def _nmea_to_decimal(degree_str: str, direction: str) -> float:
        """将NMEA格式的度分格式转换为十进制度数"""
        if not degree_str:
            return 0.0

        # 提取度数和分数
        degree_float = float(degree_str)
        degrees = int(degree_float / 100)
        minutes = degree_float - (degrees * 100)
        decimal = degrees + minutes / 60.0

        # 根据方向调整符号
        if direction in ['S', 'W']:
            decimal = -decimal

        return decimal

    @staticmethod
    def _parse_time(time_str: str, date_str: Optional[str] = None) -> datetime:
        """解析NMEA时间和日期"""
        if not time_str:
            return datetime.now()

        # 时间格式: hhmmss.sss
        hours = int(time_str[0:2])
        minutes = int(time_str[2:4])
        seconds = int(float(time_str[4:]))

        # 日期格式: ddmmyy
        if date_str:
            day = int(date_str[0:2])
            month = int(date_str[2:4])
            year = int(date_str[4:6]) + 2000
            return datetime(year, month, day, hours, minutes, seconds)
        else:
            return datetime.now().replace(hour=hours, minute=minutes, second=seconds)

    def _parse_gga(self, fields: list) -> None:
        """解析GGA语句 - 定位信息"""
        if len(fields) < 14:
            return

        # 字段映射: 0:时间, 1:纬度, 2:南北纬, 3:经度, 4:东西经,
        # 5:质量, 6:卫星数, 7:HDOP, 8:海拔, 9:海拔单位, 10:大地水准面, 11:单位, 12:差分年龄, 13:差分基准站

        time_str = fields[0]
        if fields[1] and fields[2] and fields[3] and fields[4]:
            self.position.latitude = self._nmea_to_decimal(fields[1], fields[2])
            self.position.longitude = self._nmea_to_decimal(fields[3], fields[4])

        self.position.fix_quality = int(fields[5]) if fields[5] else 0
        self.position.num_satellites = int(fields[6]) if fields[6] else 0
        self.position.hdop = float(fields[7]) if fields[7] else 0.0

        if fields[8] and fields[9]:
            self.position.altitude = float(fields[8])

        if time_str:
            self.position.timestamp = self._parse_time(time_str)

    def _parse_gsa(self, fields: list) -> None:
        """解析GSA语句 - DOP和活动卫星"""
        if len(fields) < 17:
            return

        # 字段映射: 0:模式, 1:定位类型, 2:PRN1-12, 13:PDOP, 14:HDOP, 15:VDOP

        self.position.pdop = float(fields[14]) if fields[14] else 0.0
        self.position.hdop = float(fields[15]) if fields[15] else 0.0
        self.position.vdop = float(fields[16]) if fields[16] else 0.0

    def _parse_gsv(self, fields: list) -> None:
        """解析GSV语句 - 可见卫星信息"""
        if len(fields) < 4:
            return

        # 字段映射: 0:消息总数, 1:消息序号, 2:可见卫星数,
        # 3+: 每颗卫星信息(PRN,仰角,方位角,SNR)*4

        total_msgs = int(fields[0])
        msg_num = int(fields[1])
        num_satellites_view = int(fields[2])

        # 解析卫星信息(每4个字段一组)
        sat_data = fields[3:]
        self.satellites = []

        for i in range(0, len(sat_data), 4):
            if i + 3 < len(sat_data):
                prn = int(sat_data[i]) if sat_data[i] else 0
                elev = float(sat_data[i + 1]) if sat_data[i + 1] else 0.0
                azim = float(sat_data[i + 2]) if sat_data[i + 2] else 0.0
                snr = float(sat_data[i + 3]) if sat_data[i + 3] else 0.0

                if prn > 0:
                    self.satellites.append(GPSSatellite(prn, elev, azim, snr))

        self.position.satellites = self.satellites

    def _parse_rmc(self, fields: list) -> None:
        """解析RMC语句 - 推荐最小定位信息"""
        if len(fields) < 12:
            return

        # 字段映射: 0:时间, 1:状态, 2:纬度, 3:南北纬, 4:经度, 5:东西经,
        # 6:速度, 7:航向, 8:日期, 9:磁偏角, 10:方向, 11:模式

        time_str = fields[0]
        status = fields[1]  # A=有效, V=无效

        if status == 'A' and fields[2] and fields[3] and fields[4] and fields[5]:
            self.position.latitude = self._nmea_to_decimal(fields[2], fields[3])
            self.position.longitude = self._nmea_to_decimal(fields[4], fields[5])

        if fields[6]:
            self.position.speed = float(fields[6]) * 1.852  # 节转km/h

        if fields[7]:
            self.position.course = float(fields[7])

        if fields[8]:  # 日期
            self.position.timestamp = self._parse_time(time_str, fields[8])

    def _parse_vtg(self, fields: list) -> None:
        """解析VTG语句 - 地面速度信息"""
        if len(fields) < 8:
            return

        # 字段映射: 0:航向真北, 1:符号, 2:磁北, 3:符号,
        # 4:地速(节), 5:符号, 6:地速(km/h), 7:符号

        if fields[5]:  # 节
            self.position.speed = float(fields[4])
        elif fields[6]:  # km/h
            self.position.speed = float(fields[6]) / 1.852

        if fields[0]:
            self.position.course = float(fields[0])

    def _parse_gll(self, fields: list) -> None:
        """解析GLL语句 - 地理位置"""
        if len(fields) < 6:
            return

        # 字段映射: 0:纬度, 1:南北纬, 2:经度, 3:东西经, 4:时间, 5:状态

        status = fields[5] if len(fields) > 5 else 'V'
        if status == 'A' and fields[0] and fields[1] and fields[2] and fields[3]:
            self.position.latitude = self._nmea_to_decimal(fields[0], fields[1])
            self.position.longitude = self._nmea_to_decimal(fields[2], fields[3])

            if fields[4]:
                self.position.timestamp = self._parse_time(fields[4])

    def _parse_txt(self, fields: list) -> None:
        """解析TXT语句 - 文本信息"""
        pass  # 可打印文本信息

    def parse_sentence(self, sentence: str) -> bool:
        """解析单个NMEA语句"""
        sentence = sentence.strip()

        if not sentence.startswith('$'):
            return False

        # 跳过校验和验证（部分GPS模块输出校验和可能有误）
        # if not self._verify_checksum(sentence):
        #     return False

        try:
            # 移除$和校验和部分
            data_part = sentence[1:].split('*')[0]
            fields = data_part.split(',')

            if len(fields) < 2:
                return False

            sentence_type = fields[0]
            if sentence_type in self.SENTENCE_TYPES:
                method_name = self.SENTENCE_TYPES[sentence_type]
                method = getattr(self, method_name)
                method(fields[1:])  # 跳过语句类型字段
                return True

        except Exception as e:
            print(f"解析错误: {e}")

        return False

    def get_position(self) -> GPSPosition:
        """获取当前定位信息"""
        return self.position


class GPSReader:
    """GPS串口读取器"""

    def __init__(self, port: str = 'COM3', baudrate: int = 9600,
                 timeout: int = 1, nmea_version: str = '4.11'):
        """
        初始化GPS读取器

        Args:
            port: 串口号 (如 COM3)
            baudrate: 波特率 (NMEA通常使用9600)
            timeout: 读取超时(秒)
            nmea_version: NMEA协议版本
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.nmea_version = nmea_version
        self.serial_conn: Optional[serial.Serial] = None
        self.parser = NMEAParser()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.callback: Optional[Callable[[GPSPosition], None]] = None

    def connect(self) -> bool:
        """连接到串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"已连接到 {self.port}, 波特率: {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    def disconnect(self):
        """断开串口连接"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"已断开 {self.port}")

    def _read_loop(self):
        """后台读取循环"""
        buffer = ""

        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                data = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                if data:
                    buffer += data.decode('utf-8', errors='ignore')

                    # 按行分割
                    lines = buffer.split('\n')
                    buffer = lines.pop()  # 保留不完整的行

                    for line in lines:
                        line = line.strip()
                        if line and line.startswith('$'):
                            # 调试输出原始数据
                            if hasattr(self, 'debug') and self.debug:
                                print(f"[RAW] {line}")

                            self.parser.parse_sentence(line)

                            # 如果有回调函数，发送定位信息
                            if self.callback:
                                self.callback(self.parser.get_position())

                time.sleep(0.01)

            except Exception as e:
                print(f"读取错误: {e}")
                time.sleep(0.1)

    def start_reading(self, callback: Optional[Callable[[GPSPosition], None]] = None, debug: bool = False):
        """
        开始后台读取GPS数据

        Args:
            callback: 数据回调函数，接收GPSPosition参数
            debug: 是否打印原始NMEA数据
        """
        if not self.serial_conn:
            if not self.connect():
                return False

        self.callback = callback
        self.debug = debug
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("GPS读取线程已启动")
        return True

    def read_single_sentence(self, max_attempts: int = 100) -> Optional[str]:
        """
        读取单个NMEA语句(阻塞式)

        Returns:
            NMEA语句字符串
        """
        if not self.serial_conn:
            if not self.connect():
                return None

        buffer = ""
        for _ in range(max_attempts):
            try:
                data = self.serial_conn.read_until(b'\n')
                buffer = data.decode('utf-8', errors='ignore').strip()

                if buffer and buffer.startswith('$'):
                    return buffer

            except Exception as e:
                print(f"读取失败: {e}")

            time.sleep(0.05)

        return None


def print_position(position: GPSPosition):
    """打印定位信息的回调函数"""
    print(f"\n{'='*60}")
    print(f"时间: {position.timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"纬度: {position.latitude:.6f}°")
    print(f"经度: {position.longitude:.6f}°")
    print(f"海拔: {position.altitude:.1f} 米")
    print(f"速度: {position.speed:.1f} km/h")
    print(f"航向: {position.course:.1f}°")
    print(f"定位质量: {position.fix_quality} (0=无效,1=GPS,2=DGPS,4=RTK)")
    print(f"使用卫星数: {position.num_satellites}")
    print(f"HDOP: {position.hdop:.2f}  VDOP: {position.vdop:.2f}  PDOP: {position.pdop:.2f}")

    if position.satellites:
        print(f"\n可见卫星 ({len(position.satellites)}颗):")
        for sat in position.satellites[:10]:  # 只显示前10颗
            print(f"  PRN{sat.prn:2d}: 仰角={sat.elevation:5.1f}° 方位角={sat.azimuth:5.1f}° SNR={sat.snr:4.1f}dB")


def main():
    """主函数示例"""
    print("=" * 60)
    print("NMEA 4.11 GPS 解析器")
    print("=" * 60)

    # 创建GPS读取器
    gps = GPSReader(port='/dev/ttyS1', baudrate=115200)

    # 方式1: 使用回调函数(非阻塞)
    print("\n方式1: 启动后台读取(按Ctrl+C停止)...")
    if gps.start_reading(callback=print_position):
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n正在停止...")
        finally:
            gps.disconnect()

    # 方式2: 单条读取(阻塞式)
    # print("\n方式2: 读取单个语句...")
    # sentence = gps.read_single_sentence()
    # if sentence:
    #     print(f"原始数据: {sentence}")
    #     gps.parser.parse_sentence(sentence)
    #     print_position(gps.parser.get_position())
    # gps.disconnect()


if __name__ == '__main__':
    main()
