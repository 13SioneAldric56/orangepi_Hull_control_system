"""
船体端局域网 TCP 控制集成

与微信小程序 / Android 使用同一套 TCP 行 JSON 协议。
船体监听（默认 0.0.0.0:9000），客户端连入后周期性上报 state。

服务端 → 客户端:
    {"type":"state","current":{"lat":..,"lon":..},"target":{"lat":..,"lon":..},"heading":..,"running":false}

客户端 → 服务端:
    {"type":"set_target","target":{"lat":..,"lon":..}}
    {"type":"start"}
    {"type":"stop"}

用法:
    python lan_control_hull.py
    python lan_control_hull.py --config lan_control_config.yaml
    python lan_control_hull.py --legacy   # 原 LAN 导航备案（lan_control_config_legacy.yaml）
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:
    yaml = None

_root = os.path.dirname(os.path.abspath(__file__))
if _root not in sys.path:
    sys.path.insert(0, _root)

from compass import OutputMode
from compass.wrap import HeadingWrapReader
from Gps.gps import GPSPosition, GPSReader
from heading_lock_control import (
    GPS_AVAILABLE,
    GpsNavigationRunOptions,
    HeadingLockController,
    resolve_compass_settings,
    start_gps_navigation_session,
)

DEFAULT_CONFIG_PATH = os.path.join(_root, "lan_control_config.yaml")
LEGACY_CONFIG_PATH = os.path.join(_root, "lan_control_config_legacy.yaml")

# 导航档案：cli_gps 与本地 heading_lock_control.py --gps 一致；lan_legacy 为原 LAN 行为备案
NAVIGATION_PROFILE_CLI_GPS = {
    "arrival_threshold": 5.0,
    "base_speed": 70,
    "motor_driver": "uart3",
    "esc_auto_unlock": True,
    "pid_kp": 0.8,
    "pid_ki": 0.1,
    "pid_kd": 1.5,
    "max_turn_strength": 0.2,
    "deviation_threshold": 10.0,
    "pid_p_full_scale_deg": 25.0,
    "compass_mode": "auto_50hz",
    "use_heading_wrap": True,
    "uart_port": "/dev/ttyUSB1",
    "uart_baud": 115200,
    "calibration_duration": 2.0,
    "use_forward_heading_calibration": True,
    "bearing_recompute_interval": 0.0,
    "run_duration_sec": 0.0,
    "post_arrival_reverse_sec": 0.0,
    "post_arrival_reverse_speed": None,
    "uart_debug_tx": None,
    "confirm_interactive": False,
}

NAVIGATION_PROFILE_LAN_LEGACY = {
    "arrival_threshold": 5.0,
    "base_speed": 70,
    "motor_driver": "uart3",
    "esc_auto_unlock": True,
    "pid_kp": 0.8,
    "pid_ki": 0.1,
    "pid_kd": 1.5,
    "max_turn_strength": 0.2,
    "deviation_threshold": 5.0,
    "pid_p_full_scale_deg": 25.0,
    "compass_mode": "auto_50hz",
    "use_heading_wrap": True,
    "uart_port": "/dev/ttyUSB1",
    "uart_baud": 115200,
    "calibration_duration": 2.0,
    "use_forward_heading_calibration": True,
    "bearing_recompute_interval": 0.0,
    "run_duration_sec": 0.0,
    "post_arrival_reverse_sec": 5.0,
    "post_arrival_reverse_speed": None,
    "uart_debug_tx": False,
    "confirm_interactive": False,
}

NAVIGATION_PROFILES = {
    "cli_gps": NAVIGATION_PROFILE_CLI_GPS,
    "lan_legacy": NAVIGATION_PROFILE_LAN_LEGACY,
}
COORD_DECIMALS = 6
LAN_SCAN_PING_TIMEOUT_S = 0.5
LAN_SCAN_MAX_WORKERS = 64


def round_coord(value: float) -> float:
    return round(float(value), COORD_DECIMALS)


def _iter_local_ipv4_networks() -> List[Tuple[str, ipaddress.IPv4Network, str]]:
    """读取本机 global IPv4 地址，返回 (网卡名, 网段, 本机 IP)。"""
    entries: List[Tuple[str, ipaddress.IPv4Network, str]] = []
    try:
        proc = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return entries

    for line in proc.stdout.splitlines():
        parts = line.split()
        try:
            ifname = parts[1]
            addr_part = parts[3]
            interface = ipaddress.ip_interface(addr_part)
            entries.append((ifname, interface.network, str(interface.ip)))
        except (IndexError, ValueError):
            continue
    return entries


def _probe_host_online(ip: str, timeout_s: float) -> bool:
    wait_s = max(1, int(timeout_s))
    try:
        proc = subprocess.run(
            ["ping", "-c", "1", "-W", str(wait_s), ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout_s + 1.0,
            check=False,
        )
        return proc.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return False


def _scan_network_online_hosts(
    network: ipaddress.IPv4Network,
    timeout_s: float,
    max_workers: int,
) -> List[str]:
    hosts = [str(host) for host in network.hosts()]
    if not hosts:
        return []

    online: List[str] = []
    workers = max(1, min(max_workers, len(hosts)))
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(_probe_host_online, host, timeout_s): host for host in hosts}
        for future in as_completed(futures):
            host = futures[future]
            try:
                if future.result():
                    online.append(host)
            except Exception:
                pass
    return sorted(online, key=lambda item: ipaddress.ip_address(item))


def scan_and_print_lan_online(
    timeout_s: float = LAN_SCAN_PING_TIMEOUT_S,
    max_workers: int = LAN_SCAN_MAX_WORKERS,
) -> None:
    """启动前扫描本机所在网段，打印响应 ping 的在线 IP。"""
    entries = _iter_local_ipv4_networks()
    if not entries:
        print("[网络] 未检测到可用 IPv4 网卡，跳过局域网扫描")
        return

    print("[网络] 扫描局域网在线设备...")
    for ifname, _, local_ip in entries:
        print(f"[网络] 本机 {ifname}: {local_ip}")

    seen_networks: Dict[str, ipaddress.IPv4Network] = {}
    for _, network, _ in entries:
        seen_networks[str(network)] = network

    for net_label, network in seen_networks.items():
        online = _scan_network_online_hosts(network, timeout_s, max_workers)
        if online:
            print(f"[网络] {net_label} 在线 {len(online)} 台: {', '.join(online)}")
        else:
            print(f"[网络] {net_label} 未发现响应 ping 的设备")


@dataclass
class HullRuntimeState:
    current_lat: float = 0.0
    current_lon: float = 0.0
    target_lat: float = 0.0
    target_lon: float = 0.0
    heading: float = 0.0
    running: bool = False
    gps_valid: bool = False
    target_set: bool = False


class StandbySensors:
    """待机阶段读取 GPS 与罗盘，供 state 上报。"""

    def __init__(self, compass_port: str, gps_port: str, gps_baudrate: int):
        self.compass_port = compass_port
        self.gps_port = gps_port
        self.gps_baudrate = gps_baudrate
        self._gps_reader: Optional[GPSReader] = None
        self._compass_reader: Optional[HeadingWrapReader] = None
        self._position: Optional[GPSPosition] = None
        self._lock = threading.Lock()
        self._active = False

    def start(self) -> bool:
        self.stop()
        ok_gps = self._start_gps()
        ok_compass = self._start_compass()
        self._active = ok_gps or ok_compass
        return self._active

    def stop(self) -> None:
        if self._gps_reader is not None:
            try:
                self._gps_reader.disconnect()
            except OSError:
                pass
            self._gps_reader = None
        if self._compass_reader is not None:
            try:
                self._compass_reader.stop()
            except OSError:
                pass
            self._compass_reader = None
        self._active = False

    def _start_gps(self) -> bool:
        try:
            reader = GPSReader(port=self.gps_port, baudrate=self.gps_baudrate)
            if not reader.connect():
                print(f"[待机] GPS 连接失败: {self.gps_port}")
                return False
            reader.start_reading(callback=self._on_gps, debug=False)
            self._gps_reader = reader
            print(f"[待机] GPS 已连接: {self.gps_port}")
            return True
        except Exception as exc:
            print(f"[待机] GPS 初始化失败: {exc}")
            return False

    def _start_compass(self) -> bool:
        try:
            reader = HeadingWrapReader(port=self.compass_port, compass_mode=OutputMode.AUTO_50HZ)
            if not reader.start():
                print(f"[待机] 罗盘连接失败: {self.compass_port}")
                return False
            self._compass_reader = reader
            print(f"[待机] 罗盘已连接: {self.compass_port}")
            return True
        except Exception as exc:
            print(f"[待机] 罗盘初始化失败: {exc}")
            return False

    def _on_gps(self, position: GPSPosition) -> None:
        with self._lock:
            self._position = position

    def snapshot(self) -> Tuple[float, float, float, bool]:
        lat = lon = 0.0
        heading = 0.0
        gps_valid = False

        with self._lock:
            pos = self._position
            if pos is not None and pos.fix_quality > 0:
                lat = float(pos.latitude)
                lon = float(pos.longitude)
                if lat != 0.0 or lon != 0.0:
                    gps_valid = True

        if self._compass_reader is not None:
            data = self._compass_reader.update()
            if data is not None:
                heading = float(data.wrapped_heading)

        if not gps_valid:
            lat = 0.0
            lon = 0.0

        return lat, lon, heading, gps_valid


def _navigation_defaults(profile_name: str) -> Dict[str, Any]:
    key = (profile_name or "cli_gps").strip().lower()
    if key not in NAVIGATION_PROFILES:
        known = ", ".join(sorted(NAVIGATION_PROFILES))
        raise ValueError(f"未知 navigation_profile: {profile_name!r}，可选: {known}")
    return dict(NAVIGATION_PROFILES[key])


def load_config(path: str) -> Dict[str, Any]:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"配置文件不存在: {path}")
    if yaml is None:
        raise RuntimeError("需要 PyYAML: pip install pyyaml")

    with open(path, "r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp) or {}

    listen = data.get("listen") or data.get("server") or {}
    report = data.get("report") or {}
    hardware = data.get("hardware") or {}
    navigation = data.get("navigation") or {}

    profile_name = str(navigation.get("profile", navigation.get("navigation_profile", "cli_gps")))
    nav_defaults = _navigation_defaults(profile_name)

    def _nav(key: str, cast=float, default_key: Optional[str] = None):
        dk = default_key or key
        if key in navigation:
            val = navigation[key]
            if val is None and dk in ("post_arrival_reverse_speed", "uart_debug_tx"):
                return nav_defaults.get(dk)
            return cast(val) if cast is not bool else bool(val)
        return nav_defaults[dk]

    reverse_speed_raw = navigation.get(
        "post_arrival_reverse_speed", nav_defaults["post_arrival_reverse_speed"]
    )
    post_arrival_reverse_speed = (
        int(reverse_speed_raw) if reverse_speed_raw is not None else None
    )

    uart_debug_raw = navigation.get("uart_debug_tx", nav_defaults["uart_debug_tx"])
    uart_debug_tx = None if uart_debug_raw is None else bool(uart_debug_raw)

    run_duration = float(
        navigation.get("run_duration_sec", nav_defaults["run_duration_sec"])
    )

    return {
        "config_path": os.path.abspath(path),
        "navigation_profile": profile_name,
        "listen_host": str(listen.get("host", "0.0.0.0")),
        "listen_port": int(listen.get("port", 9000)),
        "report_interval": float(report.get("interval", 0.5)),
        "compass_port": str(hardware.get("compass_port", "/dev/ttyS0")),
        "gps_port": str(hardware.get("gps_port", "/dev/ttyS1")),
        "gps_baudrate": int(hardware.get("gps_baudrate", 115200)),
        "arrival_threshold": _nav("arrival_threshold", float),
        "base_speed": _nav("base_speed", int),
        "motor_driver": str(_nav("motor_driver", str)),
        "esc_auto_unlock": _nav("esc_auto_unlock", bool),
        "pid_kp": _nav("pid_kp", float),
        "pid_ki": _nav("pid_ki", float),
        "pid_kd": _nav("pid_kd", float),
        "max_turn_strength": _nav("max_turn_strength", float),
        "deviation_threshold": _nav("deviation_threshold", float),
        "pid_p_full_scale_deg": _nav("pid_p_full_scale_deg", float),
        "compass_mode": str(_nav("compass_mode", str)),
        "use_heading_wrap": _nav("use_heading_wrap", bool),
        "uart_port": str(_nav("uart_port", str)),
        "uart_baud": int(_nav("uart_baud", int)),
        "calibration_duration": _nav("calibration_duration", float),
        "use_forward_heading_calibration": _nav("use_forward_heading_calibration", bool),
        "bearing_recompute_interval": _nav("bearing_recompute_interval", float),
        "run_duration_sec": run_duration,
        "post_arrival_reverse_sec": _nav("post_arrival_reverse_sec", float),
        "post_arrival_reverse_speed": post_arrival_reverse_speed,
        "uart_debug_tx": uart_debug_tx,
        "confirm_interactive": _nav("confirm_interactive", bool),
    }


class LanControlHull:
    """船体端 TCP 服务端 + GPS 导航集成。"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self._standby = StandbySensors(
            compass_port=config["compass_port"],
            gps_port=config["gps_port"],
            gps_baudrate=config["gps_baudrate"],
        )
        self._sock: Optional[socket.socket] = None
        self._server_sock: Optional[socket.socket] = None
        self._client_addr: Optional[tuple] = None
        self._sock_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._runtime = HullRuntimeState()
        self._running = True
        self._connected = False
        self._nav_thread: Optional[threading.Thread] = None
        self._controller: Optional[HeadingLockController] = None
        self._abort_nav = threading.Event()

    def _set_runtime(self, **kwargs) -> None:
        with self._state_lock:
            for key, value in kwargs.items():
                setattr(self._runtime, key, value)

    def _get_runtime(self) -> HullRuntimeState:
        with self._state_lock:
            return HullRuntimeState(
                current_lat=self._runtime.current_lat,
                current_lon=self._runtime.current_lon,
                target_lat=self._runtime.target_lat,
                target_lon=self._runtime.target_lon,
                heading=self._runtime.heading,
                running=self._runtime.running,
                gps_valid=self._runtime.gps_valid,
                target_set=self._runtime.target_set,
            )

    def _build_state_message(self) -> Dict[str, Any]:
        runtime = self._get_runtime()
        controller = self._controller

        if runtime.running and controller is not None and controller._gps_enabled and controller._gps_navigation:
            nav = controller._gps_navigation
            state = nav._state
            pos_ok = state.current_lat != 0.0 or state.current_lon != 0.0
            current_lat = round_coord(state.current_lat) if pos_ok else 0.0
            current_lon = round_coord(state.current_lon) if pos_ok else 0.0
            heading = round(state.current_heading, 1) if state.compass_calibrated else 0.0
            target_lat = round_coord(state.target_lat)
            target_lon = round_coord(state.target_lon)
        else:
            lat, lon, heading, gps_valid = self._standby.snapshot()
            current_lat = round_coord(lat) if gps_valid else 0.0
            current_lon = round_coord(lon) if gps_valid else 0.0
            heading = round(heading, 1) if gps_valid else 0.0
            if runtime.target_set:
                target_lat = round_coord(runtime.target_lat)
                target_lon = round_coord(runtime.target_lon)
            else:
                target_lat = 0.0
                target_lon = 0.0

        return {
            "type": "state",
            "current": {"lat": current_lat, "lon": current_lon},
            "target": {"lat": target_lat, "lon": target_lon},
            "heading": heading,
            "running": runtime.running,
        }

    def _send_ack(self, cmd: str, ok: bool, message: str = "") -> None:
        payload: Dict[str, Any] = {"type": "ack", "cmd": cmd, "ok": ok}
        if message:
            payload["message"] = message
        self._send_json(payload)

    def _send_json(self, payload: Dict[str, Any]) -> bool:
        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        with self._sock_lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall(line.encode("utf-8"))
                return True
            except OSError as exc:
                print(f"[网络] 发送失败: {exc}")
                self._mark_disconnected()
                return False

    def _send_status(self, state: str) -> None:
        self._send_json({"type": "status", "state": state})

    def _mark_disconnected(self, sock: Optional[socket.socket] = None) -> None:
        with self._sock_lock:
            if sock is not None and self._sock is not sock:
                return
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
            self._client_addr = None
        self._connected = False

    def _safe_stop_navigation(self, reason: str) -> None:
        runtime = self._get_runtime()
        if not runtime.running and self._controller is None:
            return
        print(f"[导航] 安全停车: {reason}")
        self._abort_nav.set()
        controller = self._controller
        if controller is not None:
            controller._is_running = False
            try:
                if controller._driver:
                    controller._driver.stop()
            except OSError:
                pass
        if self._nav_thread and self._nav_thread.is_alive():
            self._nav_thread.join(timeout=5.0)
        self._controller = None
        self._set_runtime(running=False)
        if not self._standby._active:
            self._standby.start()

    def _start_listener(self) -> bool:
        if self._server_sock is not None:
            return True

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self.config["listen_host"], self.config["listen_port"]))
            sock.listen(1)
            sock.settimeout(1.0)
        except OSError as exc:
            print(f"[网络] 监听失败 {self.config['listen_host']}:{self.config['listen_port']} -> {exc}")
            try:
                sock.close()
            except OSError:
                pass
            return False

        self._server_sock = sock
        print(f"[网络] 已在 {self.config['listen_host']}:{self.config['listen_port']} 监听，等待手机连入")
        return True

    def _accept_loop(self) -> None:
        while self._running and self._server_sock is not None:
            try:
                conn, addr = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    print("[网络] accept 失败")
                break

            conn.settimeout(1.0)

            with self._sock_lock:
                old_sock = self._sock
                old_addr = self._client_addr

            if old_sock is not None:
                print(f"[网络] 新客户端 {addr[0]}:{addr[1]} 接入，断开旧连接 {old_addr}")
                try:
                    old_sock.close()
                except OSError:
                    pass

            with self._sock_lock:
                self._sock = conn
                self._client_addr = addr
            self._connected = True
            print(f"[网络] 客户端已接入: {addr[0]}:{addr[1]}")

            recv_thread = threading.Thread(
                target=self._recv_loop,
                args=(conn, addr),
                daemon=True,
                name=f"client-{addr[0]}:{addr[1]}",
            )
            recv_thread.start()

    def _recv_loop(self, conn: socket.socket, addr: tuple) -> None:
        buffer = ""
        while self._running and self._connected:
            with self._sock_lock:
                sock = self._sock
            if sock is None or sock is not conn:
                break
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                print(f"[网络] 接收异常: {exc}")
                break

            if not chunk:
                print(f"[网络] 客户端断开: {addr[0]}:{addr[1]}")
                break

            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if line:
                    self._handle_command_line(line)

        was_running = self._get_runtime().running
        self._mark_disconnected(conn)
        if was_running:
            self._safe_stop_navigation("客户端断开")

    def _handle_command_line(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as exc:
            print(f"[网络] 无效 JSON: {exc} | {line[:120]}")
            return

        msg_type = str(msg.get("type", "")).lower()

        if msg_type == "set_target":
            self._handle_set_target(msg)
            return
        if msg_type == "start":
            self._handle_start()
            return
        if msg_type == "stop":
            self._handle_stop()
            return

        # 兼容旧版 command 协议（lan_control_server.py 调试客户端）
        if msg_type == "command":
            self._handle_legacy_command(msg)
            return

        print(f"[网络] 忽略未知消息 type={msg_type!r}: {line[:120]}")

    def _parse_target(self, msg: Dict[str, Any]) -> Optional[Tuple[float, float]]:
        target = msg.get("target") or {}
        try:
            lat = round_coord(target["lat"])
            lon = round_coord(target["lon"])
        except (KeyError, TypeError, ValueError):
            return None
        return lat, lon

    def _handle_set_target(self, msg: Dict[str, Any]) -> None:
        runtime = self._get_runtime()
        if runtime.running:
            print("[命令] 运行中忽略 set_target")
            self._send_ack("set_target", False, "运行中不可修改目标")
            return

        parsed = self._parse_target(msg)
        if parsed is None:
            print("[命令] set_target 缺少有效 target.lat / target.lon")
            self._send_ack("set_target", False, "坐标格式无效")
            return

        lat, lon = parsed
        self._set_runtime(target_lat=lat, target_lon=lon, target_set=True)
        print(f"[命令] 目标已设置: ({lat:.6f}, {lon:.6f})")
        self._send_ack("set_target", True)

    def _handle_legacy_command(self, msg: Dict[str, Any]) -> None:
        action = str(msg.get("action", "")).lower()
        if action == "start":
            parsed = self._parse_target(msg)
            if parsed is None:
                print("[命令] start 缺少有效 target.lat / target.lon")
                self._send_ack("start", False, "坐标格式无效")
                return
            lat, lon = parsed
            self._set_runtime(target_lat=lat, target_lon=lon, target_set=True)
            self._handle_start()
            return
        if action == "stop":
            self._handle_stop()
            return
        print(f"[命令] 未知 action: {action}")

    def _handle_start(self) -> None:
        runtime = self._get_runtime()
        if runtime.running:
            print("[命令] 已在运行中，忽略 start")
            self._send_ack("start", False, "已在运行中")
            return

        if not runtime.target_set:
            print("[命令] 未设置目标，拒绝 start")
            self._send_ack("start", False, "请先设置目标点")
            return

        _, _, _, gps_valid = self._standby.snapshot()
        if not gps_valid:
            print("[命令] GPS 无效，拒绝启动")
            self._set_runtime(
                current_lat=0.0,
                current_lon=0.0,
                heading=0.0,
                running=False,
                gps_valid=False,
            )
            self._send_ack("start", False, "GPS 无效")
            return

        lat = runtime.target_lat
        lon = runtime.target_lon
        self._set_runtime(running=True, gps_valid=True)
        self._abort_nav.clear()
        self._nav_thread = threading.Thread(
            target=self._navigation_worker,
            args=(lat, lon),
            daemon=True,
            name="gps-navigation",
        )
        self._nav_thread.start()
        print(f"[命令] 收到 start，目标=({lat:.5f}, {lon:.5f})")
        self._send_ack("start", True)

    def _handle_stop(self) -> None:
        if self._get_runtime().running:
            self._safe_stop_navigation("收到 stop 命令")
            self._send_ack("stop", True)
        else:
            self._send_ack("stop", True)

    def _build_controller(self) -> HeadingLockController:
        compass_mode, update_interval = resolve_compass_settings(self.config["compass_mode"])
        return HeadingLockController(
            compass_port=self.config["compass_port"],
            base_speed=self.config["base_speed"],
            deviation_threshold=self.config["deviation_threshold"],
            pid_kp=self.config["pid_kp"],
            pid_ki=self.config["pid_ki"],
            pid_kd=self.config["pid_kd"],
            max_turn_strength=self.config["max_turn_strength"],
            pid_p_full_scale_deg=self.config["pid_p_full_scale_deg"],
            arrival_threshold_m=self.config["arrival_threshold"],
            motor_driver=self.config["motor_driver"],
            esc_auto_unlock=self.config["esc_auto_unlock"],
            compass_mode=compass_mode,
            update_interval=update_interval,
            use_heading_wrap=self.config["use_heading_wrap"],
            uart_port=self.config["uart_port"],
            uart_baud=self.config["uart_baud"],
        )

    def _gps_run_options(self) -> GpsNavigationRunOptions:
        duration = self.config["run_duration_sec"]
        return GpsNavigationRunOptions(
            calibration_duration=self.config["calibration_duration"],
            use_forward_heading_calibration=self.config["use_forward_heading_calibration"],
            bearing_recompute_interval=self.config["bearing_recompute_interval"],
            confirm_interactive=self.config["confirm_interactive"],
            duration=None if duration <= 0 else duration,
            post_arrival_reverse_sec=self.config["post_arrival_reverse_sec"],
            post_arrival_reverse_speed=self.config["post_arrival_reverse_speed"],
            uart_debug_tx=self.config["uart_debug_tx"],
        )

    def _navigation_worker(self, lat: float, lon: float) -> None:
        if not GPS_AVAILABLE:
            print("[导航] GPS 模块不可用")
            self._set_runtime(running=False)
            self._send_json({"type": "status", "state": "error", "message": "GPS 模块不可用"})
            return

        self._standby.stop()
        controller = self._build_controller()
        self._controller = controller
        started = False

        try:
            session = start_gps_navigation_session(
                controller,
                target_lat=lat,
                target_lon=lon,
                gps_port=self.config["gps_port"],
                gps_baudrate=self.config["gps_baudrate"],
                run_options=self._gps_run_options(),
                abort_event=self._abort_nav,
            )
            started = session.started

            if not session.started:
                self._send_json({"type": "status", "state": "error", "message": "GPS 导航初始化或启动失败"})
                return

            if session.arrived:
                self._send_status("arrived")

        except Exception as exc:
            print(f"[导航] 运行异常: {exc}")
            if started:
                self._send_json({"type": "status", "state": "error", "message": str(exc)})
        finally:
            try:
                if started and (controller._is_running or controller._gps_navigation is not None):
                    controller.stop()
            except Exception as exc:
                print(f"[导航] 停止清理异常: {exc}")
            self._controller = None
            self._set_runtime(running=False)
            self._standby.start()
            print("[导航] 已回到待机")

    def _report_loop(self) -> None:
        interval = max(0.2, float(self.config["report_interval"]))
        while self._running:
            if self._connected:
                lat, lon, heading, gps_valid = self._standby.snapshot()
                if not self._get_runtime().running:
                    self._set_runtime(
                        current_lat=lat if gps_valid else 0.0,
                        current_lon=lon if gps_valid else 0.0,
                        heading=heading if gps_valid else 0.0,
                        gps_valid=gps_valid,
                    )
                self._send_json(self._build_state_message())
            time.sleep(interval)

    def run(self) -> None:
        scan_and_print_lan_online()

        if not self._standby.start():
            print("[待机] 传感器未就绪，将上报全 0，直至硬件可用")

        report_thread = threading.Thread(target=self._report_loop, daemon=True, name="state-report")
        report_thread.start()

        print("=" * 60)
        print("  船体端局域网控制（TCP 服务端）")
        print("=" * 60)
        print(f"配置: {self.config.get('config_path', DEFAULT_CONFIG_PATH)}")
        print(f"导航档案: {self.config.get('navigation_profile', 'cli_gps')} "
              f"(备案配置: {LEGACY_CONFIG_PATH})")
        print(f"监听: {self.config['listen_host']}:{self.config['listen_port']}")
        print(f"上报周期: {self.config['report_interval']}s")
        local_ips = [local_ip for _, _, local_ip in _iter_local_ipv4_networks()]
        if local_ips:
            connect_hint = " / ".join(f"{ip}:{self.config['listen_port']}" for ip in local_ips)
            print(f"手机/地面站请连接: {connect_hint}")
        else:
            print(f"手机/地面站请连接本机 IP:{self.config['listen_port']}")
        print("=" * 60)

        if not self._start_listener():
            self._standby.stop()
            return

        accept_thread = threading.Thread(target=self._accept_loop, daemon=True, name="accept-loop")
        accept_thread.start()

        try:
            while self._running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[退出] 用户中断")
        finally:
            self._running = False
            self._safe_stop_navigation("程序退出")
            self._mark_disconnected()
            if self._server_sock is not None:
                try:
                    self._server_sock.close()
                except OSError:
                    pass
                self._server_sock = None
            self._standby.stop()
            print("[退出] 船体端已停止")


def main() -> None:
    parser = argparse.ArgumentParser(description="船体端局域网 TCP 控制")
    parser.add_argument(
        "--config",
        type=str,
        default=DEFAULT_CONFIG_PATH,
        help=f"配置文件路径，默认 {DEFAULT_CONFIG_PATH}",
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help=f"使用原 LAN 导航备案配置 ({LEGACY_CONFIG_PATH})",
    )
    args = parser.parse_args()

    config_path = LEGACY_CONFIG_PATH if args.legacy else args.config
    config = load_config(config_path)
    if args.legacy:
        print(f"[配置] 已加载备案档案 lan_legacy: {config_path}")
    app = LanControlHull(config)
    app.run()


if __name__ == "__main__":
    main()
