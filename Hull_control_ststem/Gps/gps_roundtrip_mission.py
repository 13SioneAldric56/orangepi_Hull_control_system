"""
GPS 往返任务：与 heading_lock_control / GPSNavigationController 默认一致的控制逻辑
（前进车头校准、HeadingLock 默认周期与死区、每个导航周期重算方位角）-> 驶向目的地
-> 到达后停电机并单次倒计时停泊 -> 返回启动点。

可调参数见下方常量与命令行参数。磁力/辅助融合可通过 MagneticAssistProvider 预留接口接入。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional, Protocol, runtime_checkable

# 项目根目录 (Hull_control_ststem)，与 gps_navigation_controller 一致
_Gps_dir = os.path.dirname(os.path.abspath(__file__))
_root_path = os.path.dirname(_Gps_dir)
if _root_path not in sys.path:
    sys.path.insert(0, _root_path)

from Gps.gps_navigation_controller import GPSNavigationController, NavigationState
from compass import OutputMode
from heading_lock_control import HeadingLockController

# ---------------------------------------------------------------------------
# 任务默认（往返任务独有；其余与 heading_lock_control.py __main__ 一致）
# ---------------------------------------------------------------------------
ARRIVAL_THRESHOLD_M = 5.0
DWELL_SEC = 5.0
# 到达停止 / 开始停泊前：双轮后退时长（秒）
REVERSE_ON_STOP_SEC = 5.0
# 与 heading_lock --gps 内嵌 GPSNavigationController 一致：0 = 每导航周期重算方位角
BEARING_RECOMPUTE_SEC = 0.0

# ---------------------------------------------------------------------------
# 串口 / 电机 / PID：与 heading_lock_control.py __main__ CLI 默认保持一致
# ---------------------------------------------------------------------------
DEFAULT_COMPASS_PORT = '/dev/ttyS0'
DEFAULT_GPS_PORT = '/dev/ttyS1'
DEFAULT_GPS_BAUDRATE = 115200
DEFAULT_UART_PORT = '/dev/ttyUSB0'
DEFAULT_UART_BAUD = 115200
# 无 --esc/--hbridge 时默认 uart3；与 heading_lock 一致请显式传 --hbridge
DEFAULT_MOTOR_DRIVER = 'uart3'

BASE_SPEED = 80
DEVIATION_THRESHOLD_DEG = 10.0
PID_KP = 0.8
PID_KI = 0.0
PID_KD = 1.5
PID_P_FULL_SCALE_DEG = 25.0
MAX_TURN_STRENGTH = 0.2
MIN_TURN_STRENGTH = 0.05

# heading_lock --mode auto_50hz
HEADING_LOCK_UPDATE_INTERVAL_SEC = 0.02
DEFAULT_COMPASS_MODE = OutputMode.AUTO_50HZ
DEFAULT_USE_HEADING_WRAP = True
DEFAULT_ESC_AUTO_UNLOCK = True

# 与 heading_lock_control.run_loop 相同的面板刷新间隔（秒）
DISPLAY_INTERVAL_SEC = 0.1


class _PhaseLabel:
    """可变的任务阶段标题，供 on_state_update 与停泊循环读取。"""

    def __init__(self, text: str) -> None:
        self.text = text


def _tick_heading_lock_stats(hl: HeadingLockController, error: float, error_sum: list) -> None:
    """同步 run_loop 中的统计字段，使面板统计行有数据。"""
    hl._iteration_count += 1
    error_sum[0] += abs(error)
    hl._stats['avg_error'] = error_sum[0] / hl._iteration_count
    if abs(error) >= hl.deviation_threshold:
        hl._stats['correction_count'] += 1
    if abs(error) > hl._stats['max_error']:
        hl._stats['max_error'] = abs(error)


def _heading_from_nav_state(
    nav: GPSNavigationController,
    state: NavigationState,
) -> tuple[float, float | None, float]:
    """
    从导航状态取航向/误差，勿再读罗盘串口。
    导航线程已在 _navigation_loop 中独占读 compass，主线程再 get_current_heading()
    会触发 SerialException（多线程争用 /dev/ttyS0）。
    """
    hl = nav._heading_lock
    current_heading = state.current_heading
    error = state.heading_error
    continuous_heading = None
    if hl is not None and hl.use_heading_wrap and hl._heading_wrap_reader:
        # 仅读缓存，不调用 update()/read_raw()
        continuous_heading = hl._heading_wrap_reader.get_heading()
    if hl is not None:
        hl._target_heading = state.target_heading
    return current_heading, continuous_heading, error


def _refresh_heading_lock_panel(
    nav: GPSNavigationController,
    *,
    phase: _PhaseLabel,
    leg_start_time: float,
    mission_start_time: float,
    subtitle_extra: str = '',
) -> None:
    """调用内层 HeadingLockController._update_display，与 heading_lock GPS 模式一致。"""
    hl = nav._heading_lock
    if hl is None:
        return

    state = nav._state
    current_heading, continuous_heading, error = _heading_from_nav_state(nav, state)

    subtitle = f'[GPS往返] {phase.text}'
    if subtitle_extra:
        subtitle = f'{subtitle}  {subtitle_extra}'

    elapsed = time.time() - mission_start_time
    hl._update_display(
        current_heading,
        error,
        elapsed,
        continuous_heading,
        gps_state=state,
        subtitle_line=subtitle,
    )


def _make_heading_lock_display_handler(
    nav: GPSNavigationController,
    phase: _PhaseLabel,
    leg_start_time: float,
    mission_start_time: float,
    min_interval_sec: float = DISPLAY_INTERVAL_SEC,
):
    """导航线程回调：刷新 heading_lock 信息面板（覆盖式）。"""
    last_display = 0.0
    error_sum = [0.0]

    def _on_state(state: NavigationState) -> None:
        nonlocal last_display
        now = time.time()
        if min_interval_sec > 0.0 and (now - last_display) < min_interval_sec:
            return
        last_display = now

        hl = nav._heading_lock
        if hl is not None:
            _, _, err = _heading_from_nav_state(nav, state)
            _tick_heading_lock_stats(hl, err, error_sum)

        _refresh_heading_lock_panel(
            nav,
            phase=phase,
            leg_start_time=leg_start_time,
            mission_start_time=mission_start_time,
        )

    return _on_state


@runtime_checkable
class MagneticAssistProvider(Protocol):
    """预留：后期接入磁力计/多传感器融合时，在停泊期周期回调。"""

    def on_dwell_tick(self, navigation: GPSNavigationController, elapsed_in_dwell: float) -> None:
        ...


class _NoOpMagneticAssist:
    def on_dwell_tick(self, navigation: GPSNavigationController, elapsed_in_dwell: float) -> None:
        return None


def _snapshot_home(nav: GPSNavigationController) -> tuple[float, float]:
    pos = nav.get_current_position()
    if pos is not None:
        return float(pos[0]), float(pos[1])
    st = nav.get_state()
    return float(st.current_lat), float(st.current_lon)


def _run_backward_pulse(
    nav: GPSNavigationController,
    duration_sec: float,
    speed: int,
    *,
    label: str = '到达',
) -> None:
    """停止或停泊前：双轮后退若干秒后刹车。"""
    if duration_sec <= 0:
        return
    hl = getattr(nav, '_heading_lock', None)
    driver = getattr(hl, '_driver', None) if hl is not None else None
    if driver is None:
        print(f'[往返任务] {label}：无电机驱动，跳过后退')
        return
    if not hasattr(driver, 'backward'):
        print(f'[往返任务] {label}：驱动不支持后退，跳过')
        return

    print(f'[往返任务] {label}：后退 {duration_sec:.0f}s，速度 {speed}%')
    try:
        driver.backward(speed)
        deadline = time.time() + duration_sec
        while time.time() < deadline:
            time.sleep(0.05)
    except Exception as exc:
        print(f'[往返任务] {label}：后退异常: {exc}')
    finally:
        try:
            driver.stop()
        except Exception as exc:
            print(f'[往返任务] {label}：后退后停止失败: {exc}')
    print(f'[往返任务] {label}：后退结束，电机已停止')


def _emergency_brake_motors(nav: GPSNavigationController) -> None:
    """意外断连时尽快切断差速输出（再调用 nav.stop() 做完整清理）。"""
    hl = getattr(nav, '_heading_lock', None)
    driver = getattr(hl, '_driver', None) if hl is not None else None
    if driver is not None:
        try:
            driver.stop()
            print('[往返任务] 紧急刹车: 电机已停止')
        except Exception as exc:
            print(f'[往返任务] 紧急刹车调用失败: {exc}')


def _shutdown_on_ctrl_c(nav: GPSNavigationController) -> None:
    """Ctrl+C：先刹车再释放导航/GPS/罗盘资源。"""
    print('\n[往返任务] 收到 Ctrl+C，正在刹车并停止导航...')
    _emergency_brake_motors(nav)
    try:
        nav.stop()
    except Exception as exc:
        print(f'[往返任务] 停止导航时异常（可忽略）: {exc}')


def _resolve_motor_driver(args: argparse.Namespace) -> str:
    """与 heading_lock_control.py __main__ 电机选项解析一致。"""
    if getattr(args, 'uart3', False):
        return 'uart3'
    if getattr(args, 'esc', False):
        return 'esc'
    if getattr(args, 'hbridge', False):
        return 'hbridge'
    return DEFAULT_MOTOR_DRIVER


def _wait_arrival_with_link_watch(
    nav: GPSNavigationController,
    *,
    timeout: float | None,
    poll_sec: float = 0.25,
    phase: _PhaseLabel | None = None,
    leg_start_time: float | None = None,
    mission_start_time: float | None = None,
) -> bool:
    """
    等待到达；若 GPS 串口/读线程异常或导航线程崩溃，则紧急刹车并 stop。
    正常到达时返回 True；超时、主动停止、链路异常返回 False。
    """
    start = time.time()
    last_panel_time = 0.0
    while nav._is_running:
        if nav._is_arrived:
            return True
        if timeout is not None and (time.time() - start) > timeout:
            print('[GPS导航] 等待到达超时')
            return False
        if not nav.is_gps_link_healthy():
            print('[往返任务] GPS 连接异常（串口关闭或读取线程结束），执行紧急刹车')
            _emergency_brake_motors(nav)
            nav.stop()
            return False
        if nav._nav_thread is not None and not nav.is_navigation_thread_alive() and not nav._is_arrived:
            print('[往返任务] 导航线程异常结束，执行紧急刹车')
            _emergency_brake_motors(nav)
            nav.stop()
            return False
        now = time.time()
        if (
            phase is not None
            and leg_start_time is not None
            and mission_start_time is not None
            and nav._is_running
            and (now - last_panel_time) >= DISPLAY_INTERVAL_SEC
        ):
            _refresh_heading_lock_panel(
                nav,
                phase=phase,
                leg_start_time=leg_start_time,
                mission_start_time=mission_start_time,
            )
            last_panel_time = now
        time.sleep(poll_sec)
    return False


def run_roundtrip_mission(
    dest_lat: float,
    dest_lon: float,
    *,
    compass_port: str = DEFAULT_COMPASS_PORT,
    gps_port: str = DEFAULT_GPS_PORT,
    gps_baudrate: int = DEFAULT_GPS_BAUDRATE,
    arrival_threshold_m: float = ARRIVAL_THRESHOLD_M,
    dwell_sec: float = DWELL_SEC,
    bearing_recompute_sec: float = BEARING_RECOMPUTE_SEC,
    base_speed: int = BASE_SPEED,
    deviation_threshold_deg: float = DEVIATION_THRESHOLD_DEG,
    pid_kp: float = PID_KP,
    pid_ki: float = PID_KI,
    pid_kd: float = PID_KD,
    pid_p_full_scale_deg: float = PID_P_FULL_SCALE_DEG,
    max_turn_strength: float = MAX_TURN_STRENGTH,
    min_turn_strength: float = MIN_TURN_STRENGTH,
    motor_driver: str = DEFAULT_MOTOR_DRIVER,
    uart_port: str = DEFAULT_UART_PORT,
    uart_baud: int = DEFAULT_UART_BAUD,
    esc_auto_unlock: bool = DEFAULT_ESC_AUTO_UNLOCK,
    use_heading_wrap: bool = DEFAULT_USE_HEADING_WRAP,
    magnetic_assist: Optional[MagneticAssistProvider] = None,
    display_interval_sec: float = DISPLAY_INTERVAL_SEC,
    reverse_on_stop_sec: float = REVERSE_ON_STOP_SEC,
) -> None:
    """
    执行：前进车头校准（与默认 GPS 导航 / heading_lock GPS 流程一致）
    -> 去目的地 -> 后退若干秒 -> 停泊(停电机, 单次 dwell 倒计时) -> 回启动经纬度 -> 后退 -> 结束。
    """
    assist: MagneticAssistProvider = magnetic_assist or _NoOpMagneticAssist()

    heading_lock_config = HeadingLockController.build_heading_lock_config(
        compass_port=compass_port,
        base_speed=base_speed,
        deviation_threshold=deviation_threshold_deg,
        pid_kp=pid_kp,
        pid_ki=pid_ki,
        pid_kd=pid_kd,
        pid_p_full_scale_deg=pid_p_full_scale_deg,
        max_turn_strength=max_turn_strength,
        min_turn_strength=min_turn_strength,
        update_interval=HEADING_LOCK_UPDATE_INTERVAL_SEC,
        compass_mode=DEFAULT_COMPASS_MODE,
        use_heading_wrap=use_heading_wrap,
        arrival_threshold_m=arrival_threshold_m,
        motor_driver=motor_driver,
        esc_auto_unlock=esc_auto_unlock,
        uart_port=uart_port,
        uart_baud=uart_baud,
    )

    nav = GPSNavigationController(
        target_lat=dest_lat,
        target_lon=dest_lon,
        compass_port=compass_port,
        gps_port=gps_port,
        gps_baudrate=gps_baudrate,
        arrival_threshold=arrival_threshold_m,
        calibration_duration=2.0,
        calibration_speed=base_speed,
        update_interval=HEADING_LOCK_UPDATE_INTERVAL_SEC,
        heading_lock_config=heading_lock_config,
        use_forward_heading_calibration=True,
        bearing_recompute_interval=bearing_recompute_sec,
    )

    try:
        print('\n' + '=' * 60)
        print('  GPS 往返任务')
        print('=' * 60)
        print(f'  到达阈值: {arrival_threshold_m} m')
        print(f'  停泊时长: {dwell_sec} s (自首次到达起连续计时，不重置)')
        print(f'  停止/停泊前后退: {reverse_on_stop_sec:.0f} s')
        motor_desc = motor_driver
        if motor_driver == 'uart3':
            motor_desc = f'uart3 ({uart_port} @ {uart_baud})'
        print(
            f'  电机/PID: driver={motor_desc} speed={base_speed} '
            f'threshold={deviation_threshold_deg}° '
            f'Kp={pid_kp} Ki={pid_ki} Kd={pid_kd} '
            f'pid_scale={pid_p_full_scale_deg}° max_turn={max_turn_strength}'
        )
        if bearing_recompute_sec > 0:
            print(f'  方位角重算: 每 {bearing_recompute_sec} s')
        else:
            print('  方位角重算: 每个导航周期（与默认 GPSNavigationController 一致）')
        print('=' * 60)

        if not nav.initialize():
            print('[往返任务] 初始化失败')
            return

        mission_start = time.time()
        phase = _PhaseLabel('航段1 · 驶向目的地')
        leg_start = mission_start
        nav.register_callback(
            'on_state_update',
            _make_heading_lock_display_handler(
                nav, phase, leg_start, mission_start, display_interval_sec,
            ),
        )
        print(
            f'[往返任务] 已启用航向锁信息面板（与 heading_lock_control 相同，'
            f'每 {display_interval_sec:.1f}s 刷新）'
        )

        home_lat, home_lon = _snapshot_home(nav)
        print(f'[往返任务] 记录返航点(启动点): ({home_lat:.7f}, {home_lon:.7f})')

        # ---------- 航段 1：驶向目的地 ----------
        nav.navigate()
        if not _wait_arrival_with_link_watch(
            nav,
            timeout=None,
            phase=phase,
            leg_start_time=leg_start,
            mission_start_time=mission_start,
        ):
            print('[往返任务] 未能到达目的地或已停止')
            if nav._is_running:
                nav.stop()
            return

        if nav._nav_thread is not None:
            nav._nav_thread.join(timeout=30.0)

        _run_backward_pulse(
            nav, reverse_on_stop_sec, base_speed, label='到达目的地·停泊前',
        )
        print('[往返任务] 已到达，停泊期内电机保持停止')

        dwell_deadline = time.time() + dwell_sec
        phase.text = '停泊倒计时'
        print(f'[往返任务] 停泊倒计时 {dwell_sec:.0f}s 开始（水面漂移不重置计时）')
        while time.time() < dwell_deadline:
            remaining = dwell_deadline - time.time()
            assist.on_dwell_tick(nav, elapsed_in_dwell=dwell_sec - remaining)
            _refresh_heading_lock_panel(
                nav,
                phase=phase,
                leg_start_time=leg_start,
                mission_start_time=mission_start,
                subtitle_extra=f'剩余 {remaining:.0f}s',
            )
            time.sleep(min(display_interval_sec, 0.25))

        # ---------- 航段 2：返回启动点 ----------
        print(f'[往返任务] 返航目标: ({home_lat:.7f}, {home_lon:.7f})')
        nav.set_target(home_lat, home_lon)
        phase.text = '航段2 · 返航启动点'
        leg_start = time.time()
        nav.navigate()
        if not _wait_arrival_with_link_watch(
            nav,
            timeout=None,
            phase=phase,
            leg_start_time=leg_start,
            mission_start_time=mission_start,
        ):
            print('[往返任务] 返航未完成或已停止')
            if nav._is_running:
                nav.stop()
        else:
            if nav._nav_thread is not None:
                nav._nav_thread.join(timeout=30.0)
            _run_backward_pulse(
                nav, reverse_on_stop_sec, base_speed, label='返航到达·停止前',
            )
            print('[往返任务] 已到达返航点')

        sys.stdout.write('\n')
        nav.stop()
        print('[往返任务] 结束')
    except KeyboardInterrupt:
        sys.stdout.write('\n')
        _shutdown_on_ctrl_c(nav)
        raise SystemExit(130) from None


def main() -> None:
    parser = argparse.ArgumentParser(
        description='GPS 往返任务（PID/GPS 默认与 heading_lock --gps 一致；电机默认 --uart3）'
    )
    parser.add_argument('--dest-lat', type=float, required=True, help='目的地纬度')
    parser.add_argument('--dest-lon', type=float, required=True, help='目的地经度')
    parser.add_argument(
        '-p', '--port', '--compass-port',
        type=str,
        default=DEFAULT_COMPASS_PORT,
        dest='compass_port',
        help='罗盘串口，与 heading_lock -p 一致',
    )
    parser.add_argument('--gps-port', type=str, default=DEFAULT_GPS_PORT, help='GPS 串口')
    parser.add_argument(
        '--gps-baudrate', type=int, default=DEFAULT_GPS_BAUDRATE,
        help='GPS 波特率，与 heading_lock --gps-baudrate 一致',
    )
    parser.add_argument(
        '--arrival-threshold',
        type=float,
        default=ARRIVAL_THRESHOLD_M,
        dest='arrival_threshold',
        help='到达阈值(米)，与 heading_lock --arrival-threshold 一致',
    )
    parser.add_argument('--dwell', type=float, default=DWELL_SEC, help='到达后停泊秒数')
    parser.add_argument(
        '--reverse-sec',
        type=float,
        default=REVERSE_ON_STOP_SEC,
        help='每次到达停止/停泊前后退秒数，0=禁用，默认 5',
    )
    parser.add_argument(
        '--bearing-interval', type=float, default=BEARING_RECOMPUTE_SEC,
        help='方位角重算周期(秒)，0=每导航周期重算（与 heading_lock --gps 一致）',
    )
    parser.add_argument(
        '-s', '--speed', type=int, default=BASE_SPEED,
        help='基础速度 0-100，与 heading_lock -s 一致',
    )
    parser.add_argument(
        '-t', '--threshold',
        type=float,
        default=DEVIATION_THRESHOLD_DEG,
        dest='heading_threshold',
        help='航向偏差死区(度)，与 heading_lock -t 一致',
    )
    parser.add_argument('--kp', type=float, default=PID_KP, help='PID Kp')
    parser.add_argument('--ki', type=float, default=PID_KI, help='PID Ki')
    parser.add_argument('--kd', type=float, default=PID_KD, help='PID Kd')
    parser.add_argument(
        '--pid-scale-deg', type=float, default=PID_P_FULL_SCALE_DEG,
        help='P 项满量程航向误差(度)',
    )
    parser.add_argument(
        '--max-turn', type=float, default=MAX_TURN_STRENGTH,
        help='最大差速比例 0~1',
    )
    parser.add_argument(
        '--no-wrap', action='store_true',
        help='禁用航向角回环（与 heading_lock --no-wrap 一致）',
    )

    motor_group = parser.add_mutually_exclusive_group()
    motor_group.add_argument(
        '--uart3', action='store_true',
        help='UART 串口电机帧 /dev/ttyUSB0 命令码 0x06（与 heading_lock --uart3 一致）',
    )
    motor_group.add_argument(
        '--esc', action='store_true',
        help='GPIO 软件 PWM 电调（与 heading_lock --esc 一致）',
    )
    motor_group.add_argument(
        '--hbridge', action='store_true',
        help='H 桥驱动（与 heading_lock 无电机参数时默认一致）',
    )
    parser.add_argument(
        '--uart-port', type=str, default=DEFAULT_UART_PORT,
        help='UART 电机串口（仅 --uart3）',
    )
    parser.add_argument(
        '--uart-baud', type=int, default=DEFAULT_UART_BAUD,
        help='UART 电机波特率（仅 --uart3）',
    )
    parser.add_argument(
        '--no-esc-unlock', action='store_true',
        help='ESC 模式跳过启动时双路 3s 中位解锁',
    )
    parser.add_argument(
        '--display-interval',
        type=float,
        default=DISPLAY_INTERVAL_SEC,
        help='信息面板刷新间隔(秒)，与 heading_lock run_loop 一致，默认 0.1',
    )

    args = parser.parse_args()
    motor_driver = _resolve_motor_driver(args)

    run_roundtrip_mission(
        args.dest_lat,
        args.dest_lon,
        compass_port=args.compass_port,
        gps_port=args.gps_port,
        gps_baudrate=args.gps_baudrate,
        arrival_threshold_m=args.arrival_threshold,
        dwell_sec=args.dwell,
        bearing_recompute_sec=args.bearing_interval,
        base_speed=args.speed,
        deviation_threshold_deg=args.heading_threshold,
        pid_kp=args.kp,
        pid_ki=args.ki,
        pid_kd=args.kd,
        pid_p_full_scale_deg=args.pid_scale_deg,
        max_turn_strength=args.max_turn,
        motor_driver=motor_driver,
        uart_port=args.uart_port,
        uart_baud=args.uart_baud,
        esc_auto_unlock=not args.no_esc_unlock,
        use_heading_wrap=not args.no_wrap,
        magnetic_assist=None,
        display_interval_sec=args.display_interval,
        reverse_on_stop_sec=args.reverse_sec,
    )


if __name__ == '__main__':
    main()


# 默认即 uart3 + 与 heading_lock --gps --uart3 相同 PID/GPS 参数，仅填目的地即可:
# python3 Gps/gps_roundtrip_mission.py --dest-lat <纬度> --dest-lon <经度>