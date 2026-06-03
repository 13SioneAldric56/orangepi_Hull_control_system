"""
GPS导航控制器模块

结合GPS定位、电子罗盘与航向角锁定控制，实现自主导航功能

功能特性:
1. 车头方向：默认前进若干秒校准；可选 use_forward_heading_calibration=False 静止读取罗盘
2. 由目标GPS经纬度和当前经纬度计算目标航向角；可选 bearing_recompute_interval 按周期重算方位角
3. 将目标航向角发送给HeadingLockController进行差速修正
4. 支持到达判断（距离阈值5m）
5. 集成GPS数据回调接收

使用示例:
    from Gps.gps_navigation_controller import GPSNavigationController

    controller = GPSNavigationController(
        target_lat=22.123456,
        target_lon=113.123456,
        compass_port='/dev/ttyS0',
        gps_port='/dev/ttyS1'
    )
    controller.initialize()      # 初始化并校准车头方向
    controller.navigate()      # 开始导航
"""

import sys
import time
import math
import threading
from typing import TYPE_CHECKING, Optional, Tuple, Callable
from dataclasses import dataclass

sys.path.insert(0, __file__.rsplit('/', 1)[0] if '/' in __file__ else '.')

_root_path = __file__.rsplit('/Gps/', 1)[0] if '/Gps/' in __file__ else __file__.rsplit('/', 1)[0]
sys.path.insert(0, _root_path)

from Gps.gps import GPSReader, GPSPosition
from compass import OutputMode

if TYPE_CHECKING:
    from heading_lock_control import HeadingLockController


@dataclass
class NavigationState:
    """导航状态数据"""
    current_lat: float           # 当前纬度
    current_lon: float           # 当前经度
    target_lat: float            # 目标纬度
    target_lon: float           # 目标经度
    distance_to_target: float    # 到目标距离(米)
    bearing_angle: float         # 方位角(°)
    target_heading: float       # 目标航向角(°)
    current_heading: float       # 当前航向角(°)
    heading_error: float        # 航向偏差(°)
    is_initialized: bool         # 是否已初始化
    is_arrived: bool            # 是否已到达
    compass_calibrated: bool     # 罗盘是否已校准


class GPSNavigationController:
    """
    GPS导航控制器

    负责任务:
    1. 管理GPS数据接收
    2. 计算目标航向角
    3. 与HeadingLockController交互进行航向修正
    """

    def __init__(
        self,
        target_lat: float,
        target_lon: float,
        compass_port: str = '/dev/ttyS0',
        gps_port: str = '/dev/ttyS1',
        gps_baudrate: int = 38400,
        arrival_threshold: float = 5.0,
        calibration_duration: float = 2.0,
        calibration_speed: int = 80,
        update_interval: float = 0.1,
        heading_lock_config: dict = None,
        use_forward_heading_calibration: bool = True,
        bearing_recompute_interval: float = 0.0,
    ):
        """
        初始化GPS导航控制器

        Args:
            target_lat: 目标纬度
            target_lon: 目标经度
            compass_port: 电子罗盘串口路径
            gps_port: GPS串口路径
            gps_baudrate: GPS波特率
            arrival_threshold: 到达阈值(米)，默认5米
            calibration_duration: 校准持续时间(秒)，默认2秒
            calibration_speed: 校准时小车速度 (0-100)
            update_interval: 导航更新间隔(秒)
            heading_lock_config: HeadingLockController配置字典
            use_forward_heading_calibration: True 时前进若干秒校准车头；False 时以当前罗盘航向为车头，不前进
            bearing_recompute_interval: 大于0时，按该间隔(秒)重新计算到目标的方位角；0 表示每次更新都重算
        """
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.compass_port = compass_port
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self.arrival_threshold = arrival_threshold
        self.calibration_duration = calibration_duration
        self.calibration_speed = calibration_speed
        self.update_interval = update_interval
        self.use_forward_heading_calibration = use_forward_heading_calibration
        self.bearing_recompute_interval = max(0.0, float(bearing_recompute_interval))

        self._heading_lock_config = heading_lock_config or {}
        self._heading_lock_config.setdefault('compass_port', compass_port)

        self._gps_reader: Optional[GPSReader] = None
        self._heading_lock: Optional["HeadingLockController"] = None

        self._current_position: Optional[GPSPosition] = None
        self._position_lock = threading.Lock()

        self._is_running = False
        self._is_initialized = False
        self._is_arrived = False

        self._calibration_heading: float = 0.0
        self._nav_thread: Optional[threading.Thread] = None

        self._callbacks: dict = {}
        self._bearing_cached: Optional[float] = None
        self._last_bearing_wall_time: float = 0.0

        self._state = NavigationState(
            current_lat=0.0,
            current_lon=0.0,
            target_lat=target_lat,
            target_lon=target_lon,
            distance_to_target=float('inf'),
            bearing_angle=0.0,
            target_heading=0.0,
            current_heading=0.0,
            heading_error=0.0,
            is_initialized=False,
            is_arrived=False,
            compass_calibrated=False
        )

    def _on_gps_update(self, position: GPSPosition):
        """GPS数据回调"""
        with self._position_lock:
            self._current_position = position

        if position.fix_quality == 0:
            return

        self._state.current_lat = position.latitude
        self._state.current_lon = position.longitude

        if 'on_position_update' in self._callbacks:
            self._callbacks['on_position_update'](position)

    def _init_gps(self) -> bool:
        """初始化GPS读取器"""
        try:
            self._gps_reader = GPSReader(
                port=self.gps_port,
                baudrate=self.gps_baudrate
            )

            if not self._gps_reader.connect():
                print(f"[GPS导航] 错误: 无法连接到GPS {self.gps_port}")
                return False

            self._gps_reader.start_reading(
                callback=self._on_gps_update,
                debug=False
            )
            print(f"[GPS导航] GPS已连接到 {self.gps_port}, 波特率: {self.gps_baudrate}")
            return True

        except Exception as e:
            print(f"[GPS导航] GPS初始化失败: {e}")
            return False

    def _init_heading_lock(self) -> bool:
        """初始化航向角锁定控制器"""
        try:
            from heading_lock_control import HeadingLockController

            config = self._heading_lock_config.copy()
            config['compass_port'] = self.compass_port
            config.setdefault('base_speed', self.calibration_speed)
            config.setdefault('deviation_threshold', 5.0)
            config.setdefault('pid_kp', 20)
            config.setdefault('pid_ki', 0.1)
            config.setdefault('pid_kd', 1000)
            config.setdefault('update_interval', self.update_interval)

            self._heading_lock = HeadingLockController(**config)
            print(f"[GPS导航] HeadingLockController已初始化")
            return True

        except Exception as e:
            print(f"[GPS导航] HeadingLockController初始化失败: {e}")
            return False

    def _wait_for_gps_fix(self, timeout: float = 30.0) -> bool:
        """等待GPS定位成功"""
        print(f"[GPS导航] 等待GPS定位 (超时{timeout}秒)...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            with self._position_lock:
                if self._current_position and self._current_position.fix_quality > 0:
                    pos = self._current_position
                    print(f"[GPS导航] GPS定位成功: {pos.latitude:.6f}°, {pos.longitude:.6f}°")
                    print(f"[GPS导航] 定位质量: {pos.fix_quality}, 卫星数: {pos.num_satellites}")
                    return True
            time.sleep(0.5)

        print("[GPS导航] GPS定位超时")
        return False

    def _calibrate_heading(self) -> bool:
        """
        校准车头方向

        流程:
        1. 小车前进2秒
        2. 记录此时的罗盘读数作为基准方向
        """
        print(f"[GPS导航] 开始校准车头方向 (前进{self.calibration_duration}秒)...")

        if not self._heading_lock.start(calibrate=False):
            print("[GPS导航] HeadingLockController启动失败")
            return False

        self._heading_lock._driver.forward(self.calibration_speed)
        print(f"[GPS导航] 小车前进中，速度: {self.calibration_speed}%")

        time.sleep(self.calibration_duration)

        current_heading = self._heading_lock.get_current_heading()
        if current_heading is None:
            print("[GPS导航] 无法读取罗盘数据")
            self._heading_lock.stop()
            return False

        self._calibration_heading = current_heading
        self._state.compass_calibrated = True
        self._state.current_heading = current_heading

        self._heading_lock._driver.stop()
        print(f"[GPS导航] 车头方向已校准: {self._calibration_heading:.1f}°")

        return True

    def _calibrate_heading_without_motion(self, settle_time: float = 0.35) -> bool:
        """
        不前进，以当前罗盘读数作为车头航向（适用于水面载具等场景）。
        """
        print("[GPS导航] 无前进车头校准: 使用当前罗盘航向作为车头方向")

        if not self._heading_lock.start(calibrate=False):
            print("[GPS导航] HeadingLockController启动失败")
            return False

        if settle_time > 0:
            time.sleep(settle_time)

        current_heading = self._heading_lock.get_current_heading()
        if current_heading is None:
            print("[GPS导航] 无法读取罗盘数据")
            self._heading_lock.stop()
            return False

        self._calibration_heading = current_heading
        self._state.compass_calibrated = True
        self._state.current_heading = current_heading

        self._heading_lock._driver.stop()
        print(f"[GPS导航] 车头方向(无前进): {self._calibration_heading:.1f}°")

        return True

    def initialize(self) -> bool:
        """
        初始化导航系统

        流程:
        1. 初始化GPS读取器
        2. 等待GPS定位成功
        3. 初始化HeadingLockController
        4. 校准车头方向（可选前进校准或静止读取罗盘）

        Returns:
            初始化是否成功
        """
        print("\n" + "=" * 60)
        print("  GPS导航控制器初始化")
        print("=" * 60)

        if not self._init_gps():
            return False

        if not self._wait_for_gps_fix():
            self._cleanup()
            return False

        if not self._init_heading_lock():
            self._cleanup()
            return False

        if self.use_forward_heading_calibration:
            if not self._calibrate_heading():
                self._cleanup()
                return False
        else:
            if not self._calibrate_heading_without_motion():
                self._cleanup()
                return False

        self._state.is_initialized = True
        self._is_initialized = True

        print(f"[GPS导航] 初始化完成!")
        print(f"  目标位置: ({self.target_lat:.6f}, {self.target_lon:.6f})")
        print(f"  到达阈值: {self.arrival_threshold}米")
        if self.bearing_recompute_interval > 0:
            print(f"  方位角重算周期: {self.bearing_recompute_interval}秒")

        if 'on_initialized' in self._callbacks:
            self._callbacks['on_initialized']()

        return True

    @staticmethod
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        计算从点1到点2的方位角

        使用Vincenty公式的简化版本，适用于短距离

        Args:
            lat1, lon1: 起点纬度和经度(度)
            lat2, lon2: 终点纬度和经度(度)

        Returns:
            方位角(度)，0-360，0=北，90=东，180=南，270=西
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)

        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

        bearing = math.atan2(x, y)
        bearing_deg = math.degrees(bearing)

        bearing_deg = (bearing_deg + 360) % 360

        return bearing_deg

    @staticmethod
    def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        计算两点之间的距离（使用Haversine公式）

        Args:
            lat1, lon1: 起点纬度和经度(度)
            lat2, lon2: 终点纬度和经度(度)

        Returns:
            距离(米)
        """
        R = 6371000

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        a = math.sin(delta_lat / 2) ** 2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def _calculate_target_heading(self) -> Optional[float]:
        """
        计算方位角（从当前位置到目标位置的方向）

        Returns:
            bearing: 方位角(度)，0-360，0=北，90=东，180=南，270=西
        """
        if not self._state.compass_calibrated:
            return None

        bearing = self.calculate_bearing(
            self._state.current_lat,
            self._state.current_lon,
            self.target_lat,
            self.target_lon
        )

        self._state.bearing_angle = bearing
        self._state.target_heading = bearing  # 目标航向直接使用方位角

        return bearing

    def _update_navigation(self):
        """更新导航状态

        航向误差计算: error = bearing - current_heading
        - error > 0: 目标在右侧，需要右转
        - error < 0: 目标在左侧，需要左转
        """
        with self._position_lock:
            if not self._current_position or self._current_position.fix_quality == 0:
                return

        self._state.distance_to_target = self.calculate_distance(
            self._state.current_lat,
            self._state.current_lon,
            self.target_lat,
            self.target_lon
        )

        self._state.current_heading = self._heading_lock.get_current_heading() or 0

        now = time.time()
        if self.bearing_recompute_interval <= 0:
            recompute_bearing = True
        else:
            recompute_bearing = (
                self._bearing_cached is None
                or (now - self._last_bearing_wall_time) >= self.bearing_recompute_interval
            )

        if recompute_bearing:
            bearing = self.calculate_bearing(
                self._state.current_lat,
                self._state.current_lon,
                self.target_lat,
                self.target_lon
            )
            self._bearing_cached = bearing
            self._last_bearing_wall_time = now
        else:
            bearing = self._bearing_cached if self._bearing_cached is not None else 0.0

        self._state.bearing_angle = bearing

        # 航向误差 = 方位角 - 当前航向
        error = bearing - self._state.current_heading
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        self._state.heading_error = error
        self._state.target_heading = bearing

        if self._state.distance_to_target <= self.arrival_threshold:
            self._state.is_arrived = True
            self._is_arrived = True
            print(f"[GPS导航] 已到达目标! 距离: {self._state.distance_to_target:.1f}米")

            if 'on_arrived' in self._callbacks:
                self._callbacks['on_arrived']()

    def _navigation_loop(self):
        """导航主循环

        航向修正逻辑:
        1. 更新导航状态（计算方位角、误差）
        2. 同步 HeadingLock 目标航向并调用 PID 差速驱动电机
        """
        print("[GPS导航] 导航循环已启动")

        while self._is_running and not self._is_arrived:
            self._update_navigation()

            # 同步目标航向并执行 PID 差速（勿每周期 set_target_heading，否则会重置积分）
            if not self._is_arrived and self._heading_lock:
                self._heading_lock.sync_target_heading(self._state.target_heading)
                current_heading = self._heading_lock.get_current_heading()
                if current_heading is not None:
                    continuous_heading = None
                    if self._heading_lock.use_heading_wrap and self._heading_lock._heading_wrap_reader:
                        continuous_heading = self._heading_lock.get_continuous_heading()
                    err = self._heading_lock._calculate_heading_error(
                        current_heading, continuous_heading
                    )
                    self._heading_lock._apply_pid_correction(err)

            if 'on_state_update' in self._callbacks:
                self._callbacks['on_state_update'](self._state)

            time.sleep(self.update_interval)

        print("[GPS导航] 导航循环已结束")

    def navigate(self):
        """
        开始导航

        在调用此方法之前，必须先调用 initialize()
        """
        if not self._is_initialized:
            print("[GPS导航] 错误: 请先调用 initialize()")
            return

        self._is_running = True
        self._nav_thread = threading.Thread(target=self._navigation_loop, daemon=True)
        self._nav_thread.start()
        print("[GPS导航] 导航已启动")

    def stop(self):
        """停止导航"""
        self._is_running = False

        if self._heading_lock:
            self._heading_lock.stop()

        if self._gps_reader:
            self._gps_reader.disconnect()

        if self._nav_thread and self._nav_thread.is_alive():
            self._nav_thread.join(timeout=2)

        print("[GPS导航] 已停止")

    def _cleanup(self):
        """清理资源"""
        if self._heading_lock:
            self._heading_lock.stop()
        if self._gps_reader:
            self._gps_reader.disconnect()

    def set_target(self, lat: float, lon: float):
        """设置新的目标位置"""
        self.target_lat = lat
        self.target_lon = lon
        self._state.target_lat = lat
        self._state.target_lon = lon
        self._is_arrived = False
        self._state.is_arrived = False
        self._bearing_cached = None
        self._last_bearing_wall_time = 0.0
        print(f"[GPS导航] 目标已更新: ({lat:.6f}, {lon:.6f})")

    def register_callback(self, event: str, callback: Callable):
        """
        注册回调函数

        Args:
            event: 事件类型
                - 'on_initialized': 初始化完成
                - 'on_position_update': GPS位置更新
                - 'on_state_update': 导航状态更新
                - 'on_arrived': 到达目标
            callback: 回调函数
        """
        self._callbacks[event] = callback

    def get_state(self) -> NavigationState:
        """获取当前导航状态"""
        return self._state

    def get_current_position(self) -> Optional[Tuple[float, float]]:
        """获取当前位置 (纬度, 经度)"""
        with self._position_lock:
            if self._current_position and self._current_position.fix_quality > 0:
                return (self._current_position.latitude, self._current_position.longitude)
        return None

    def is_gps_link_healthy(self) -> bool:
        """GPS 串口已打开且后台读取线程仍在运行（用于检测掉线、拔线等）。"""
        reader = self._gps_reader
        if reader is None:
            return False
        conn = getattr(reader, 'serial_conn', None)
        if conn is None or not conn.is_open:
            return False
        th = getattr(reader, 'thread', None)
        if th is not None and not th.is_alive():
            return False
        return True

    def is_navigation_thread_alive(self) -> bool:
        """导航主循环线程是否仍在运行（navigate() 启动后用于检测异常退出）。"""
        t = self._nav_thread
        return t is not None and t.is_alive()

    def wait_for_arrival(self, timeout: float = None) -> bool:
        """
        等待到达目标

        Args:
            timeout: 超时时间(秒)，None表示无限等待

        Returns:
            是否成功到达
        """
        start_time = time.time()

        while self._is_running:
            if self._is_arrived:
                return True

            if timeout and (time.time() - start_time) > timeout:
                print("[GPS导航] 等待到达超时")
                return False

            time.sleep(0.5)

        return False

    def __enter__(self):
        """支持 with 语句"""
        if not self.initialize():
            raise RuntimeError("初始化失败")
        return self

    def __exit__(self, *args):
        """退出 with 语句时停止"""
        self.stop()

    def run_interactive(self, duration: float = None):
        """
        交互式运行导航

        Args:
            duration: 运行持续时间(秒)，None表示直到到达目标
        """
        if not self._is_initialized:
            print("[GPS导航] 正在初始化...")
            if not self.initialize():
                print("[GPS导航] 初始化失败")
                return

        self.navigate()

        try:
            while self._is_running and not self._is_arrived:
                state = self._state

                print("\033[2J\033[H")
                print("╔══════════════════════════════════════════════════════════════╗")
                print("║                    GPS导航控制系统                            ║")
                print("╠══════════════════════════════════════════════════════════════╣")

                if state.compass_calibrated:
                    print(f"║  校准方向: {state.current_heading:>6.1f}°                                      ║")
                print(f"║  当前位置: {state.current_lat:>10.6f}°, {state.current_lon:>10.6f}°          ║")
                print(f"║  目标位置: {state.target_lat:>10.6f}°, {state.target_lon:>10.6f}°          ║")
                print("╠══════════════════════════════════════════════════════════════╣")
                print(f"║  距目标:   {state.distance_to_target:>8.1f} 米                                   ║")
                print(f"║  方位角:   {state.bearing_angle:>8.1f}°                                   ║")
                print(f"║  目标航向: {state.target_heading:>8.1f}°                                   ║")
                print(f"║  当前航向: {state.current_heading:>8.1f}°                                   ║")
                print(f"║  航向偏差: {state.heading_error:>+8.1f}°                                   ║")
                print("╠══════════════════════════════════════════════════════════════╣")

                if state.is_arrived:
                    print("║                    ✓ 已到达目标!                               ║")
                elif state.is_initialized:
                    print("║                    ⟳ 导航中...                                  ║")
                else:
                    print("║                    ○ 初始化中...                                ║")

                print("╚══════════════════════════════════════════════════════════════╝")

                if duration:
                    elapsed = time.time() - (getattr(self, '_start_time', time.time()))
                    if elapsed >= duration:
                        break

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[GPS导航] 用户中断")

        finally:
            self.stop()


def demo_navigation():
    """导航演示"""
    print("\n" + "=" * 60)
    print("  GPS导航控制器演示")
    print("=" * 60)

    controller = GPSNavigationController(
        target_lat=22.400980,
        target_lon=113.539615,
        compass_port='/dev/ttyS0',
        gps_port='/dev/ttyS1',
        gps_baudrate=38400,
        arrival_threshold=5.0,
        calibration_duration=2.0
    )

    controller.run_interactive()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='GPS导航控制器')
    parser.add_argument('--target-lat', type=float, required=True, help='目标纬度')
    parser.add_argument('--target-lon', type=float, required=True, help='目标经度')
    parser.add_argument('--compass-port', type=str, default='/dev/ttyS0', help='罗盘串口')
    parser.add_argument('--gps-port', type=str, default='/dev/ttyS1', help='GPS串口')
    parser.add_argument('--gps-baudrate', type=int, default=38400, help='GPS波特率')
    parser.add_argument('--threshold', type=float, default=5.0, help='到达阈值(米)')
    parser.add_argument('--cal-time', type=float, default=2.0, help='校准时间(秒)')
    parser.add_argument('--duration', type=float, default=None, help='运行时长(秒)')
    parser.add_argument('--no-forward-cal', action='store_true',
                        help='禁用前进车头校准，使用当前罗盘航向')
    parser.add_argument('--bearing-interval', type=float, default=0.0,
                        help='方位角重算周期(秒)，0表示每次导航周期都重算')

    args = parser.parse_args()

    controller = GPSNavigationController(
        target_lat=args.target_lat,
        target_lon=args.target_lon,
        compass_port=args.compass_port,
        gps_port=args.gps_port,
        gps_baudrate=args.gps_baudrate,
        arrival_threshold=args.threshold,
        calibration_duration=args.cal_time,
        use_forward_heading_calibration=not args.no_forward_cal,
        bearing_recompute_interval=args.bearing_interval,
    )

    controller.run_interactive(duration=args.duration)
