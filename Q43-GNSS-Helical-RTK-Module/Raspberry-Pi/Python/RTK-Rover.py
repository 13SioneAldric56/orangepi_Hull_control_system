#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# RTK-Rover.py

"""
Raspberry Pi RTK NTRIP Tool (no Flask warning)
- Default serial: /dev/ttyUSB0
- Default baud rate: 115200
- Default NTRIP: ntrip.geodetic.gov.hk:2101 / Mountpoint HKCL_32
- Automatically starts RTK after launch
- Web page can edit and save parameters to rtk_config.json
- Uses Python standard library HTTP server instead of Flask
"""

import os
import time
import json
import math
import glob
import base64
import socket
import struct
import fcntl
import threading
from binascii import hexlify
from collections import deque
from urllib.parse import parse_qs, urlparse
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

# ---------------- Defaults ----------------
BAUD_RATE = 115200
HTTP_PORT = 5000

DEFAULT_SERIAL = "/dev/ttyUSB0"
DEFAULT_CASTER_HOST = "ntrip.geodetic.gov.hk"
DEFAULT_CASTER_PORT = "2101"
DEFAULT_MOUNTPOINT = "HKCL_32"
DEFAULT_USERNAME = ""
DEFAULT_PASSWORD = ""

AUTO_START_ON_BOOT = True
CONFIG_FILE = "rtk_config.json"

# ---------- Global state ----------
serial_port = None
serial_thread = None
ntrip_thread = None
stop_event = threading.Event()
serial_lock = threading.Lock()
state_lock = threading.Lock()

last_gga = ""
last_gga_time = 0.0

status = {
    "serial": "Disconnected",
    "ntrip": "Disconnected",
    "message": "",
}

config = {
    "serial_port": DEFAULT_SERIAL,
    "caster_host": DEFAULT_CASTER_HOST,
    "caster_port": DEFAULT_CASTER_PORT,
    "mountpoint": DEFAULT_MOUNTPOINT,
    "username": DEFAULT_USERNAME,
    "password": DEFAULT_PASSWORD,
}

rtk_info = {
    "fix_quality": "0",
    "fix_text": "Invalid",
    "lat": "",
    "lon": "",
    "map_lat": "",
    "map_lon": "",
    "event": "",
    "offset_cm": "",
}

bias_ref_lat = None
bias_ref_lon = None

nmea_buffer = deque(maxlen=300)
rtcm_buffer = deque(maxlen=300)
sourcetable_mountpoints = []

QUALITY_MAP = {
    "0": "Invalid",
    "1": "GPS Fix",
    "2": "DGPS",
    "4": "RTK Fixed",
    "5": "RTK Float",
}

# ---------- China / GCJ-02 transform ----------
PI = 3.14159265358979323846
A = 6378245.0
EE = 0.00669342162296594323


def out_of_china(lat, lon):
    if lon < 72.004 or lon > 137.8347:
        return True
    if lat < 0.8293 or lat > 55.8271:
        return True
    return False


def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320.0 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return ret


def transform_lon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lat, lon):
    if out_of_china(lat, lon):
        return lat, lon
    dlat = transform_lat(lon - 105.0, lat - 35.0)
    dlon = transform_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * PI)
    dlon = (dlon * 180.0) / (A / sqrtmagic * math.cos(radlat) * PI)
    return lat + dlat, lon + dlon


def distance_m(lat1, lon1, lat2, lon2):
    r = 6378137.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c


# ---------- Config load/save ----------
def load_config_from_file():
    global config
    if not os.path.exists(CONFIG_FILE):
        return
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            for key in config.keys():
                if key in data and isinstance(data[key], str):
                    config[key] = data[key].strip()
        if not config.get("serial_port"):
            config["serial_port"] = DEFAULT_SERIAL
        if not config.get("caster_host"):
            config["caster_host"] = DEFAULT_CASTER_HOST
        if not config.get("caster_port"):
            config["caster_port"] = DEFAULT_CASTER_PORT
        if not config.get("mountpoint"):
            config["mountpoint"] = DEFAULT_MOUNTPOINT
        print(f"Config loaded from {CONFIG_FILE}")
    except Exception as e:
        print(f"Failed to load config: {e}")


def save_config_to_file():
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config, f, ensure_ascii=False, indent=2)
        print(f"Config saved to {CONFIG_FILE}")
    except Exception as e:
        print(f"Failed to save config: {e}")


load_config_from_file()


# ---------- Serial port scan ----------
def scan_serial_ports():
    patterns = [
        "/dev/ttyS*",
        "/dev/ttyAMA*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyXRUSB*",
    ]
    ports = []
    for pattern in patterns:
        for port in glob.glob(pattern):
            if os.path.exists(port):
                ports.append(port)
    return sorted(set(ports))


# ---------- NTRIP sourcetable ----------
def fetch_ntrip_sourcetable(host, port, user="", password=""):
    mountpoints = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((host, int(port)))

    request_data = "GET / HTTP/1.1\r\n"
    request_data += f"Host: {host}\r\n"
    request_data += "User-Agent: NTRIP client\r\n"
    request_data += "Ntrip-Version: Ntrip/2.0\r\n"
    request_data += "Connection: close\r\n"
    if user or password:
        auth_str = f"{user}:{password}"
        auth_b64 = base64.b64encode(auth_str.encode()).decode()
        request_data += f"Authorization: Basic {auth_b64}\r\n"
    request_data += "\r\n"

    sock.sendall(request_data.encode("ascii", errors="ignore"))

    data = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        data += chunk
    sock.close()

    text = data.decode(errors="ignore")
    for line in text.splitlines():
        line_stripped = line.strip()
        if line_stripped.upper().startswith("STR;"):
            parts = line_stripped.split(";")
            if len(parts) > 1 and parts[1]:
                mountpoints.append(parts[1])

    seen = set()
    ordered = []
    for mountpoint in mountpoints:
        if mountpoint not in seen:
            seen.add(mountpoint)
            ordered.append(mountpoint)
    return ordered


# ---------- Helper parsing ----------
def parse_lat_lon_from_gga(fields):
    lat = ""
    lon = ""
    if len(fields) > 4 and fields[2] and fields[3]:
        value = fields[2]
        deg = int(value[0:2])
        minutes = float(value[2:])
        lat_val = deg + minutes / 60.0
        if fields[3] == "S":
            lat_val = -lat_val
        lat = f"{lat_val:.8f}"
    if len(fields) > 6 and fields[4] and fields[5]:
        value = fields[4]
        deg = int(value[0:3])
        minutes = float(value[3:])
        lon_val = deg + minutes / 60.0
        if fields[5] == "W":
            lon_val = -lon_val
        lon = f"{lon_val:.8f}"
    return lat, lon


# ---------- Serial thread ----------
def serial_reader():
    global last_gga, last_gga_time, serial_port, bias_ref_lat, bias_ref_lon

    with state_lock:
        status["serial"] = "Connected, reading NMEA..."
        prev_fix_quality = rtk_info["fix_quality"]

    try:
        while not stop_event.is_set():
            with serial_lock:
                if serial_port is None:
                    with state_lock:
                        status["serial"] = "Serial closed"
                    break
                line = serial_port.readline()

            if not line:
                continue

            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue

            if not text:
                continue

            with state_lock:
                nmea_buffer.append(text)

            if text.startswith("$GPGGA") or text.startswith("$GNGGA"):
                last_gga = text
                last_gga_time = time.time()

                fields = text.split(",")
                if len(fields) > 6:
                    fix_q = fields[6]
                    lat, lon = parse_lat_lon_from_gga(fields)

                    with state_lock:
                        rtk_info["fix_quality"] = fix_q
                        rtk_info["fix_text"] = QUALITY_MAP.get(fix_q, "Unknown")
                        if lat:
                            rtk_info["lat"] = lat
                        if lon:
                            rtk_info["lon"] = lon

                    try:
                        if lat and lon:
                            wlat = float(lat)
                            wlon = float(lon)
                            mlat, mlon = wgs84_to_gcj02(wlat, wlon)
                            with state_lock:
                                rtk_info["map_lat"] = f"{mlat:.8f}"
                                rtk_info["map_lon"] = f"{mlon:.8f}"

                            if fix_q in ("4", "5"):
                                if bias_ref_lat is None or bias_ref_lon is None:
                                    bias_ref_lat = wlat
                                    bias_ref_lon = wlon
                                d_m = distance_m(bias_ref_lat, bias_ref_lon, wlat, wlon)
                                with state_lock:
                                    rtk_info["offset_cm"] = f"{d_m * 100.0:.1f} cm"
                    except Exception:
                        with state_lock:
                            rtk_info["map_lat"] = ""
                            rtk_info["map_lon"] = ""

                    if fix_q != prev_fix_quality:
                        with state_lock:
                            if fix_q == "4":
                                rtk_info["event"] = f"RTK FIXED at {rtk_info['lat']}, {rtk_info['lon']}"
                            elif fix_q == "5":
                                rtk_info["event"] = f"RTK FLOAT at {rtk_info['lat']}, {rtk_info['lon']}"
                            elif fix_q in ("1", "2"):
                                rtk_info["event"] = "GNSS fix acquired"
                            else:
                                rtk_info["event"] = "No valid fix"
                        prev_fix_quality = fix_q

    except Exception as e:
        with state_lock:
            status["serial"] = f"Serial error: {e}"
    finally:
        with serial_lock:
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
                serial_port = None
        with state_lock:
            status["serial"] = "Disconnected"


# ---------- NTRIP thread ----------
def ntrip_client():
    global serial_port

    host = (config.get("caster_host") or "").strip()
    port = (config.get("caster_port") or "").strip()
    mount = (config.get("mountpoint") or "").lstrip("/").strip()
    user = (config.get("username") or "").strip()
    password = (config.get("password") or "").strip()

    if not host or not port or not mount:
        with state_lock:
            status["ntrip"] = "Caster not configured"
        return

    while not stop_event.is_set():
        sock = None
        try:
            with state_lock:
                status["ntrip"] = f"Connecting {host}:{port}..."
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((host, int(port)))

            headers = [
                f"GET /{mount} HTTP/1.1",
                "User-Agent: NTRIP client",
                "Accept: */*",
                "Connection: keep-alive",
                f"Host: {host}",
            ]
            if user or password:
                auth_str = f"{user}:{password}"
                auth_enc = base64.b64encode(auth_str.encode()).decode()
                headers.append(f"Authorization: Basic {auth_enc}")

            request_data = "\r\n".join(headers) + "\r\n\r\n"
            sock.sendall(request_data.encode("ascii", errors="ignore"))

            with state_lock:
                status["ntrip"] = "Connected, waiting RTCM..."
            sock.settimeout(0.1)
            last_gga_sent = 0.0

            while not stop_event.is_set():
                try:
                    data = sock.recv(1024)
                    if not data:
                        with state_lock:
                            status["ntrip"] = "NTRIP closed, reconnecting..."
                        break
                    with state_lock:
                        rtcm_buffer.append(hexlify(data[:80]).decode())
                    with serial_lock:
                        if serial_port is not None:
                            serial_port.write(data)
                except socket.timeout:
                    pass

                if last_gga and (time.time() - last_gga_sent > 5):
                    send_line = last_gga + "\r\n"
                    try:
                        sock.sendall(send_line.encode("ascii", errors="ignore"))
                        last_gga_sent = time.time()
                    except Exception as e:
                        with state_lock:
                            status["ntrip"] = f"NTRIP error: send GGA failed ({e})"
                        break

                time.sleep(0.01)

        except Exception as e:
            with state_lock:
                status["ntrip"] = f"NTRIP error: {e}"
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

        if not stop_event.is_set():
            time.sleep(1)

    with state_lock:
        status["ntrip"] = "Disconnected"


# ---------- Start/Stop core ----------
def start_rtk_from_current_config():
    global serial_port, serial_thread, ntrip_thread, stop_event, bias_ref_lat, bias_ref_lon

    if not (config.get("serial_port") or "").strip():
        config["serial_port"] = DEFAULT_SERIAL
    if not (config.get("caster_host") or "").strip():
        config["caster_host"] = DEFAULT_CASTER_HOST
    if not (config.get("caster_port") or "").strip():
        config["caster_port"] = DEFAULT_CASTER_PORT
    if not (config.get("mountpoint") or "").strip():
        config["mountpoint"] = DEFAULT_MOUNTPOINT

    stop_event.set()
    time.sleep(0.3)
    stop_event = threading.Event()

    with state_lock:
        status["message"] = ""

    try:
        with serial_lock:
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
            serial_port = serial.Serial(config["serial_port"], BAUD_RATE, timeout=1)
        with state_lock:
            status["serial"] = f"Connected {config['serial_port']} @ {BAUD_RATE}"
    except Exception as e:
        with state_lock:
            status["serial"] = f"Failed to open serial: {e}"
            status["message"] = "Check serial device and permission"
        return False

    with state_lock:
        rtk_info["fix_quality"] = "0"
        rtk_info["fix_text"] = "Invalid"
        rtk_info["lat"] = ""
        rtk_info["lon"] = ""
        rtk_info["map_lat"] = ""
        rtk_info["map_lon"] = ""
        rtk_info["event"] = ""
        rtk_info["offset_cm"] = ""
        nmea_buffer.clear()
        rtcm_buffer.clear()
        status["message"] = "RTK started"

    bias_ref_lat = None
    bias_ref_lon = None

    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    ntrip_thread = threading.Thread(target=ntrip_client, daemon=True)
    ntrip_thread.start()
    return True


def stop_rtk():
    global serial_port
    stop_event.set()
    with serial_lock:
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
            serial_port = None
    with state_lock:
        status["serial"] = "Disconnected"
        status["ntrip"] = "Disconnected"
        status["message"] = "RTK stopped"


def update_config_from_fields(fields):
    serial_text = first_field(fields, "serial_port") or first_field(fields, "serial_port_select")
    config["serial_port"] = (serial_text or "").strip() or DEFAULT_SERIAL
    config["caster_host"] = (first_field(fields, "caster_host") or "").strip() or DEFAULT_CASTER_HOST
    config["caster_port"] = (first_field(fields, "caster_port") or "").strip() or DEFAULT_CASTER_PORT
    config["mountpoint"] = (first_field(fields, "mountpoint") or "").strip() or DEFAULT_MOUNTPOINT
    config["username"] = (first_field(fields, "username") or "").strip()
    config["password"] = (first_field(fields, "password") or "").strip()


def first_field(fields, key, default=""):
    value = fields.get(key, [default])
    if isinstance(value, list):
        return value[0] if value else default
    return value


def format_last_gga_time():
    if last_gga_time:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(last_gga_time))
    return ""


def get_status_snapshot():
    with state_lock:
        return {
            "serial": status["serial"],
            "ntrip": status["ntrip"],
            "message": status["message"],
            "fix_text": rtk_info["fix_text"],
            "lat": rtk_info["lat"],
            "lon": rtk_info["lon"],
            "map_lat": rtk_info["map_lat"],
            "map_lon": rtk_info["map_lon"],
            "last_gga_time": format_last_gga_time(),
            "event": rtk_info["event"],
            "offset_cm": rtk_info["offset_cm"],
        }


# ---------- Network URL display ----------
def get_iface_ip(ifname):
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(
            fcntl.ioctl(
                sock.fileno(),
                0x8915,
                struct.pack("256s", ifname[:15].encode("utf-8")),
            )[20:24]
        )
    except Exception:
        return None
    finally:
        if sock:
            sock.close()


def print_urls():
    found = False
    seen = set()
    try:
        for _, ifname in socket.if_nameindex():
            if ifname == "lo":
                continue
            ip = get_iface_ip(ifname)
            if not ip or ip in seen:
                continue
            seen.add(ip)
            print(f"{ifname}:\n  http://{ip}:{HTTP_PORT}")
            found = True
    except Exception:
        pass

    if not found:
        print(f"localhost:\n  http://127.0.0.1:{HTTP_PORT}")


# ---------- HTML ----------
def build_index_html():
    snapshot = get_status_snapshot()
    serial_ports = scan_serial_ports()

    with state_lock:
        config_snapshot = dict(config)
        mountpoints_snapshot = list(sourcetable_mountpoints)

    options_html = ['<option value="">-- Select /dev/tty* --</option>']
    for port in serial_ports:
        selected = ' selected' if port == config_snapshot["serial_port"] else ''
        options_html.append(f'<option value="{escape(port)}"{selected}>{escape(port)}</option>')

    sourcetable_html = ['<option value="">-- Click Update NTRIP Source Table --</option>']
    for mp in mountpoints_snapshot:
        sourcetable_html.append(f'<option value="{escape(mp)}">{escape(mp)}</option>')

    return f"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Raspberry Pi RTK NTRIP Tool</title>
<style>
body {{ font-family: sans-serif; max-width: 100%; margin: 20px; background: #87CEEB; }}
h2 {{ margin-top: 0; }}
label {{ display: inline-block; width: 130px; }}
input[type=text], input[type=number], input[type=password], select {{ width: 260px; padding: 4px; margin: 4px 0; }}
button {{ padding: 6px 16px; margin: 0 4px; }}
fieldset {{ margin-top: 15px; }}
.top-buttons {{ display: flex; gap: 12px; margin-bottom: 16px; }}
.top-buttons form {{ margin: 0; }}
#main {{ display: flex; gap: 24px; align-items: flex-start; }}
.col-left {{ flex: 0 0 380px; min-width: 280px; }}
.col-right {{ flex: 1.0; min-width: 0; }}
.stream-row {{ display: flex; gap: 16px; margin-top: 12px; }}
.stream-col {{ flex: 1; min-width: 0; }}
.stream-box {{ width: 100%; height: 220px; background:#111; padding:6px; overflow-y:scroll; white-space:pre; font-size:12px; }}
#nmea_box {{ color:#0f0; }}
#rtcm_box {{ color:#0ff; }}
.status-box span.label {{ font-weight: bold; }}
.status-box div {{ margin: 2px 0; }}
.event-ok {{ color: #00ff00; font-weight: bold; }}
.event-float {{ color: #ffa500; font-weight: bold; }}
#rtk_banner {{ margin: 6px 0 12px; padding: 8px 12px; border-radius: 6px; font-weight: bold; font-size: 18px; background: #333; color: #ccc; }}
.banner-fixed {{ background: #004d00; color: #00ff00; }}
.banner-float {{ background: #4a2d00; color: #ffa500; }}
.banner-dgps {{ background: #002b36; color: #00e5ff; }}
.banner-none {{ background: #333; color: #ccc; }}
.fix-rtk {{ color: #00ff00; font-weight:bold; font-size: 18px; }}
.fix-float {{ color: #ffa500; font-weight:bold; font-size: 18px; }}
.fix-dgps {{ color: #00e5ff; font-weight:bold; font-size: 18px; }}
.small {{ font-size: 12px; opacity: 0.85; }}
</style>
</head>
<body>
<h2>Raspberry Pi RTK NTRIP Tool</h2>

<div class="top-buttons">
  <form method="post" action="/start"><button type="submit">Start RTK</button></form>
  <form method="post" action="/stop"><button type="submit">Stop RTK</button></form>
</div>

<div class="small">
Default: {escape(DEFAULT_SERIAL)} @ {BAUD_RATE} |
NTRIP: {escape(DEFAULT_CASTER_HOST)}:{escape(DEFAULT_CASTER_PORT)} / {escape(DEFAULT_MOUNTPOINT)} |
AutoStart: {'ON' if AUTO_START_ON_BOOT else 'OFF'}
</div>

<div id="main">
<div class="col-left">
<form method="post" action="/start">
  <fieldset>
    <legend>Serial Settings</legend>
    <label>Serial device:</label>
    <select name="serial_port_select" onchange="document.getElementById('serial_port').value=this.value;">
      {''.join(options_html)}
    </select><br>
    <label>Serial (manual):</label>
    <input type="text" id="serial_port" name="serial_port" value="{escape(config_snapshot['serial_port'])}" placeholder="/dev/ttyUSB0">
    <span> Baud: {BAUD_RATE}</span>
  </fieldset>

  <fieldset>
    <legend>NTRIP Caster</legend>
    <label>Address:</label>
    <input type="text" name="caster_host" value="{escape(config_snapshot['caster_host'])}" placeholder="ntrip.geodetic.gov.hk"><br>
    <label>Port:</label>
    <input type="number" name="caster_port" value="{escape(config_snapshot['caster_port'])}" placeholder="2101"><br>
    <label>Mount Point:</label>
    <input type="text" name="mountpoint" id="mountpoint_input" value="{escape(config_snapshot['mountpoint'])}" placeholder="HKCL_32"><br>

    <label>Source Table:</label>
    <select id="mountpoint_select" onchange="if(this.value){{document.getElementById('mountpoint_input').value=this.value;}}">
      {''.join(sourcetable_html)}
    </select><br>
    <button type="button" onclick="updateSourceTable()">Update NTRIP Source Table</button><br><br>

    <label>Username:</label>
    <input type="text" name="username" value="{escape(config_snapshot['username'])}" placeholder=""><br>
    <label>Password:</label>
    <input type="password" name="password" value="{escape(config_snapshot['password'])}" autocomplete="off" placeholder="">
  </fieldset>

  <fieldset>
    <legend>Send GNSS Command</legend>
    <label>Command:</label>
    <input type="text" name="cmd" id="cmd_input" value="$PQTMSRR*4B">
    <button type="submit" formaction="/send_cmd">Send</button>
    <button type="button" onclick="document.getElementById('cmd_input').value='$PQTMSRR*4B';">PQTMSRR</button>
  </fieldset>

  <div style="margin-top:10px;">
    <button type="submit">Start RTK</button>
    <button type="submit" formaction="/save_config">Save config</button>
    <button type="submit" formaction="/load_config">Load config</button>
  </div>
</form>
</div>

<div class="col-right">
<h3>Status</h3>
<div id="rtk_banner" class="banner-none">Waiting for GNSS fix...</div>

<div class="status-box">
  <div><span class="label">Serial:</span> <span id="serial_status">{escape(snapshot['serial'])}</span></div>
  <div><span class="label">NTRIP:</span> <span id="ntrip_status">{escape(snapshot['ntrip'])}</span></div>
  <div><span class="label">Message:</span> <span id="msg_status">{escape(snapshot['message'])}</span></div>
  <div><span class="label">Fix:</span> <span id="fix_text">{escape(snapshot['fix_text'])}</span></div>
  <div><span class="label">Latitude (WGS84):</span> <span id="lat_text">{escape(snapshot['lat'])}</span></div>
  <div><span class="label">Longitude (WGS84):</span> <span id="lon_text">{escape(snapshot['lon'])}</span></div>
  <div><span class="label">Map Lat (GCJ/China):</span> <span id="map_lat_text">{escape(snapshot['map_lat'])}</span></div>
  <div><span class="label">Map Lon (GCJ/China):</span> <span id="map_lon_text">{escape(snapshot['map_lon'])}</span></div>
  <div><span class="label">Last GGA time:</span> <span id="gga_time">{escape(snapshot['last_gga_time'])}</span></div>
  <div><span class="label">RTK Event:</span> <span id="event_text">{escape(snapshot['event'])}</span></div>
  <div><span class="label">Position offset (cm):</span> <span id="offset_text">{escape(snapshot['offset_cm'])}</span></div>
  <div>
    <span class="label">Bing Map:</span>
    <a id="bing_link" href="#" target="_blank" style="opacity:0.5;pointer-events:none;">No coordinate yet</a>
  </div>
</div>

<div class="stream-row">
  <div class="stream-col">
    <h3>RTCM Stream (from caster, hex)</h3>
    <div id="rtcm_box" class="stream-box"></div>
  </div>
  <div class="stream-col">
    <h3>Live NMEA Stream (from receiver)</h3>
    <div id="nmea_box" class="stream-box"></div>
  </div>
</div>
</div>
</div>

<script>
function updateStatus() {{
  fetch('/api/status').then(r => r.json()).then(s => {{
    document.getElementById('serial_status').textContent = s.serial;
    document.getElementById('ntrip_status').textContent = s.ntrip;
    document.getElementById('msg_status').textContent = s.message;
    document.getElementById('lat_text').textContent = s.lat;
    document.getElementById('lon_text').textContent = s.lon;
    document.getElementById('map_lat_text').textContent = s.map_lat || '';
    document.getElementById('map_lon_text').textContent = s.map_lon || '';
    document.getElementById('gga_time').textContent = s.last_gga_time;
    document.getElementById('offset_text').textContent = s.offset_cm || '';

    const fixSpan = document.getElementById('fix_text');
    fixSpan.textContent = s.fix_text;
    fixSpan.className = '';

    const banner = document.getElementById('rtk_banner');
    banner.className = 'banner-none';
    banner.textContent = 'Waiting for GNSS fix...';

    const ev = document.getElementById('event_text');
    ev.textContent = s.event;
    ev.className = '';

    if (s.fix_text === 'RTK Fixed') {{
      fixSpan.className = 'fix-rtk';
      banner.className = 'banner-fixed';
      banner.textContent = 'RTK FIXED (cm-level)';
      ev.className = 'event-ok';
    }} else if (s.fix_text === 'RTK Float') {{
      fixSpan.className = 'fix-float';
      banner.className = 'banner-float';
      banner.textContent = 'RTK FLOAT (decimeter-level)';
      ev.className = 'event-float';
    }} else if (s.fix_text === 'DGPS') {{
      fixSpan.className = 'fix-dgps';
      banner.className = 'banner-dgps';
      banner.textContent = 'DGPS (no RTK yet)';
    }}

    const bingLink = document.getElementById('bing_link');
    const useLat = s.map_lat || s.lat;
    const useLon = s.map_lon || s.lon;
    if (useLat && useLon) {{
      const coord = useLat + ',' + useLon;
      const url = 'https://www.bing.com/maps?q=' + encodeURIComponent(coord);
      bingLink.href = url;
      bingLink.textContent = coord + ' (Open in Bing)';
      bingLink.style.opacity = '1';
      bingLink.style.pointerEvents = 'auto';
    }} else {{
      bingLink.href = '#';
      bingLink.textContent = 'No coordinate yet';
      bingLink.style.opacity = '0.5';
      bingLink.style.pointerEvents = 'none';
    }}
  }});
}}

function updateNmea() {{
  fetch('/api/nmea').then(r => r.text()).then(t => {{
    const box = document.getElementById('nmea_box');
    box.textContent = t;
    box.scrollTop = box.scrollHeight;
  }});
}}

function updateRtcm() {{
  fetch('/api/rtcm').then(r => r.text()).then(t => {{
    const box = document.getElementById('rtcm_box');
    box.textContent = t;
    box.scrollTop = box.scrollHeight;
  }});
}}

function updateSourceTable() {{
  const host = document.querySelector('input[name="caster_host"]').value;
  const port = document.querySelector('input[name="caster_port"]').value;
  const user = document.querySelector('input[name="username"]').value;
  const pwd  = document.querySelector('input[name="password"]').value;
  if (!host || !port) {{ alert('Please fill caster host and port first.'); return; }}

  fetch('/api/update_sourcetable', {{
    method: 'POST',
    headers: {{'Content-Type': 'application/json'}},
    body: JSON.stringify({{ caster_host: host, caster_port: port, username: user, password: pwd }})
  }})
  .then(r => r.json())
  .then(data => {{
    const sel = document.getElementById('mountpoint_select');
    sel.innerHTML = '';
    if (!data.ok) {{
      alert('Update NTRIP source table failed: ' + (data.error || 'unknown error'));
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = '-- No STR lines found --';
      sel.appendChild(opt);
      return;
    }}
    const firstOpt = document.createElement('option');
    firstOpt.value = '';
    firstOpt.textContent = '-- Select mount point --';
    sel.appendChild(firstOpt);
    data.mountpoints.forEach(mp => {{
      const opt = document.createElement('option');
      opt.value = mp;
      opt.textContent = mp;
      sel.appendChild(opt);
    }});
  }})
  .catch(err => {{ alert('Update NTRIP source table error: ' + err); }});
}}

setInterval(updateStatus, 1000);
setInterval(updateNmea, 1000);
setInterval(updateRtcm, 1000);
updateStatus();
updateNmea();
updateRtcm();
</script>
</body>
</html>
"""


class RTKRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_text(self, text, status_code=200, content_type="text/plain; charset=utf-8"):
        data = text.encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj, status_code=200):
        self.send_text(json.dumps(obj, ensure_ascii=False), status_code, "application/json; charset=utf-8")

    def redirect(self, location):
        self.send_response(302)
        self.send_header("Location", location)
        self.end_headers()

    def read_form(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length).decode("utf-8", errors="ignore")
        return parse_qs(body, keep_blank_values=True)

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length).decode("utf-8", errors="ignore")
        if not body:
            return {}
        try:
            return json.loads(body)
        except Exception:
            return {}

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/":
            self.send_text(build_index_html(), content_type="text/html; charset=utf-8")
            return

        if path == "/api/status":
            self.send_json(get_status_snapshot())
            return

        if path == "/api/nmea":
            with state_lock:
                text = "\n".join(nmea_buffer)
            self.send_text(text)
            return

        if path == "/api/rtcm":
            with state_lock:
                text = "\n".join(rtcm_buffer)
            self.send_text(text)
            return

        self.send_text("Not Found", 404)

    def do_POST(self):
        global sourcetable_mountpoints

        path = urlparse(self.path).path

        if path == "/start":
            fields = self.read_form()
            update_config_from_fields(fields)
            start_rtk_from_current_config()
            self.redirect("/")
            return

        if path == "/stop":
            self.read_form()
            stop_rtk()
            self.redirect("/")
            return

        if path == "/save_config":
            fields = self.read_form()
            update_config_from_fields(fields)
            save_config_to_file()
            with state_lock:
                status["message"] = "Config saved to file"
            self.redirect("/")
            return

        if path == "/load_config":
            self.read_form()
            load_config_from_file()
            with state_lock:
                status["message"] = "Config loaded from file"
            self.redirect("/")
            return

        if path == "/send_cmd":
            fields = self.read_form()
            cmd = (first_field(fields, "cmd") or "").strip()
            if not cmd:
                with state_lock:
                    status["message"] = "No command to send"
                self.redirect("/")
                return

            to_send = cmd
            if not to_send.endswith("\r") and not to_send.endswith("\n"):
                to_send += "\r\n"

            with serial_lock:
                if serial_port is None:
                    with state_lock:
                        status["message"] = "Serial not open, cannot send command"
                else:
                    try:
                        serial_port.write(to_send.encode("ascii", errors="ignore"))
                        with state_lock:
                            status["message"] = f"Sent command: {cmd}"
                    except Exception as e:
                        with state_lock:
                            status["message"] = f"Send command error: {e}"
            self.redirect("/")
            return

        if path == "/api/update_sourcetable":
            data = self.read_json()
            host = (data.get("caster_host") or "").strip()
            port = str(data.get("caster_port") or "").strip()
            user = (data.get("username") or "").strip()
            password = (data.get("password") or "").strip()

            if not host or not port:
                self.send_json({"ok": False, "error": "Host/port not set", "mountpoints": []})
                return

            try:
                mountpoints = fetch_ntrip_sourcetable(host, port, user=user, password=password)
                with state_lock:
                    sourcetable_mountpoints = mountpoints
                if not mountpoints:
                    self.send_json({"ok": False, "error": "No STR lines found in sourcetable", "mountpoints": []})
                    return
                self.send_json({"ok": True, "mountpoints": mountpoints})
                return
            except Exception as e:
                self.send_json({"ok": False, "error": str(e), "mountpoints": []})
                return

        self.send_text("Not Found", 404)


def main():
    if AUTO_START_ON_BOOT:
        try:
            save_config_to_file()
        except Exception:
            pass
        start_rtk_from_current_config()

    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), RTKRequestHandler)
    print(f"RTK web server started on port {HTTP_PORT}")
    print_urls()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_rtk()
        server.server_close()
        print("Server stopped")


if __name__ == "__main__":
    main()