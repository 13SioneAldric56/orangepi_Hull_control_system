#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
# RTK-Base.py

import os
import json
import time
import html
import socket
import fcntl
import struct
import threading
import urllib.parse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

PORT = 12345
APP_NAME = "web_rtk_server"
CONFIG_FILE = "rtk_config.json"

DEFAULT_CONFIG = {
    "serial_port": "/dev/ttyUSB0",
    "baud_rate": 115200,

    "caster_host": "164.90.243.252",
    "caster_port": 2101,
    "username": "lb1020",
    "password": "776spe",
    "mountpoint": "MP24981",

    "send_gga": False,
    "gga_interval_sec": 10.0,

    "init_cmd": "$POLCFGRTCM,1,1,1",
    "save_cmd": "$POLCFGSAVE",

    "custom_cmd": "",

    "socket_timeout_sec": 15,
    "reconnect_sec": 3.0,
}

cfg_lock = threading.Lock()
run_lock = threading.Lock()
serial_lock = threading.Lock()
status_lock = threading.Lock()
logs_lock = threading.Lock()

serial_obj = None
sock_obj = None
uploader_thread = None
stop_event = threading.Event()

status = {
    "running": False,
    "serial": "Disconnected",
    "ntrip": "Disconnected",
    "message": "",
    "bytes_up": 0,
    "last_up_ts": 0,
    "last_err": "",
    "last_cmd": "",
}

logs = deque(maxlen=400)

last_gga = ""
last_gga_ts = 0.0


def get_iface_ip(ifname):
    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(
            fcntl.ioctl(
                s.fileno(),
                0x8915,
                struct.pack("256s", ifname[:15].encode("utf-8"))
            )[20:24]
        )
    except Exception:
        return None
    finally:
        if s:
            s.close()


def print_urls():
    seen = set()

    print(f"localhost: \n http://127.0.0.1:{PORT}")
    seen.add("127.0.0.1")

    try:
        for ifname in os.listdir("/sys/class/net"):
            if ifname == "lo":
                continue
            ip = get_iface_ip(ifname)
            if ip and ip not in seen:
                print(f"{ifname}: \n http://{ip}:{PORT}")
                seen.add(ip)
    except Exception:
        pass


def log(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    with logs_lock:
        logs.appendleft(line)
    print(line, flush=True)


def load_config():
    cfg = DEFAULT_CONFIG.copy()
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                on_disk = json.load(f)
            if isinstance(on_disk, dict):
                cfg.update(on_disk)
        except Exception as e:
            log(f"Config load failed: {e}")
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        log(f"Config saved to {CONFIG_FILE}")
    except Exception as e:
        log(f"Config save failed: {e}")


def set_status(**kwargs):
    with status_lock:
        status.update(kwargs)


def open_serial(port, baud):
    return serial.Serial(
        port=port,
        baudrate=int(baud),
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1,
        write_timeout=2,
    )


def normalize_cmd_lines(text):
    if not text:
        return []
    lines = []
    for raw in text.replace("\r\n", "\n").replace("\r", "\n").split("\n"):
        line = raw.strip()
        if line:
            lines.append(line)
    return lines


def serial_send_lines(lines):
    global serial_obj
    if not lines:
        return
    if serial_obj is None:
        raise RuntimeError("Serial not connected")
    with serial_lock:
        for line in lines:
            out = line.strip()
            if not out.endswith("\n"):
                out += "\r\n"
            serial_obj.write(out.encode("ascii", errors="ignore"))
            serial_obj.flush()
            set_status(last_cmd=line)
            log(f"Serial sent: {line}")


def ntrip_source_handshake(sock, cfg):
    mount = (cfg.get("mountpoint") or "").strip()
    if mount.startswith("/"):
        mount = mount[1:]
    password = (cfg.get("password") or "").strip()

    req = (
        f"SOURCE {password} /{mount}\r\n"
        f"Source-Agent: NTRIP {APP_NAME}\r\n"
        f"\r\n"
    ).encode("ascii", errors="ignore")

    sock.sendall(req)
    sock.settimeout(cfg.get("socket_timeout_sec", 15))

    try:
        resp = sock.recv(4096)
    except socket.timeout:
        resp = b""
    except Exception:
        resp = b""

    text = resp.decode("latin1", errors="ignore").strip()
    if text:
        log(f"Caster response: {text[:200]}")
        if ("401" in text) or ("403" in text) or ("ERROR" in text.upper()):
            raise RuntimeError(f"Caster rejected: {text}")
    else:
        log("Caster response: (no banner)")
    return True


def build_ntrip_gga_line(gga_sentence):
    line = gga_sentence.strip()
    if not line.startswith("$"):
        return b""
    if not line.endswith("\r\n"):
        line += "\r\n"
    return line.encode("ascii", errors="ignore")


def extract_gga_from_stream(buf):
    global last_gga, last_gga_ts
    try:
        s = buf.decode("latin1", errors="ignore")
    except Exception:
        return

    for key in ("$GPGGA", "$GNGGA", "$GLGGA", "$GAGGA"):
        idx = s.find(key)
        if idx >= 0:
            end = s.find("\n", idx)
            if end > idx:
                line = s[idx:end].strip()
                if line:
                    last_gga = line
                    last_gga_ts = time.time()
            break


def close_all():
    global serial_obj, sock_obj

    try:
        if serial_obj:
            serial_obj.close()
    except Exception:
        pass
    serial_obj = None

    try:
        if sock_obj:
            sock_obj.close()
    except Exception:
        pass
    sock_obj = None


def uploader_loop():
    global serial_obj, sock_obj

    log("Uploader thread started")
    set_status(running=True, last_err="")

    bytes_up = 0
    last_up_ts = 0
    last_gga_sent = 0.0
    init_sent = False

    while not stop_event.is_set():
        with cfg_lock:
            cfg = load_config()

        if serial_obj is None:
            try:
                serial_obj = open_serial(cfg["serial_port"], cfg["baud_rate"])
                set_status(
                    serial="Connected",
                    message=f"Serial {cfg['serial_port']} @ {cfg['baud_rate']}"
                )
                log(f"Serial connected: {cfg['serial_port']} @ {cfg['baud_rate']}")
                init_sent = False
            except Exception as e:
                set_status(
                    serial="Disconnected",
                    ntrip="Disconnected",
                    last_err=str(e),
                    message="Serial open failed"
                )
                log(f"Serial open failed: {e}")
                time.sleep(float(cfg.get("reconnect_sec", 3)))
                continue

        if not init_sent:
            try:
                init_lines = normalize_cmd_lines(cfg.get("init_cmd", ""))
                save_lines = normalize_cmd_lines(cfg.get("save_cmd", ""))
                serial_send_lines(init_lines)
                serial_send_lines(save_lines)
                init_sent = True
                log("RTK Base init commands sent OK")
            except Exception as e:
                set_status(last_err=str(e), message="Init command send failed")
                log(f"Init command send failed: {e}")
                time.sleep(float(cfg.get("reconnect_sec", 3)))
                continue

        if sock_obj is None:
            try:
                sock_obj = socket.create_connection(
                    (cfg["caster_host"], int(cfg["caster_port"])),
                    timeout=cfg.get("socket_timeout_sec", 15),
                )
                sock_obj.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                ntrip_source_handshake(sock_obj, cfg)
                set_status(
                    ntrip="Connected",
                    message=f"NTRIP {cfg['caster_host']}:{cfg['caster_port']} / {cfg['mountpoint']}",
                )
                log(f"NTRIP connected: {cfg['caster_host']}:{cfg['caster_port']} mount={cfg['mountpoint']}")
            except Exception as e:
                try:
                    if sock_obj:
                        sock_obj.close()
                except Exception:
                    pass
                sock_obj = None
                set_status(ntrip="Disconnected", last_err=str(e), message="Caster connect failed")
                log(f"Caster connect/handshake failed: {e}")
                time.sleep(float(cfg.get("reconnect_sec", 3)))
                continue

        try:
            chunk = serial_obj.read(4096)
            if chunk:
                if cfg.get("send_gga", False):
                    extract_gga_from_stream(chunk)

                sock_obj.sendall(chunk)
                bytes_up += len(chunk)
                last_up_ts = int(time.time())
                set_status(bytes_up=bytes_up, last_up_ts=last_up_ts)

            if cfg.get("send_gga", False):
                now = time.time()
                if now - last_gga_sent >= float(cfg.get("gga_interval_sec", 10)):
                    if last_gga:
                        gga_bytes = build_ntrip_gga_line(last_gga)
                        if gga_bytes:
                            sock_obj.sendall(gga_bytes)
                            last_gga_sent = now
                            log(f"Sent GGA: {last_gga[:80]}")

        except Exception as e:
            set_status(ntrip="Disconnected", last_err=str(e), message="Upload error, reconnecting...")
            log(f"Upload error: {e}")
            try:
                if sock_obj:
                    sock_obj.close()
            except Exception:
                pass
            sock_obj = None
            time.sleep(float(cfg.get("reconnect_sec", 3)))
            continue

    log("Uploader thread stopping...")
    close_all()
    set_status(running=False, serial="Disconnected", ntrip="Disconnected", message="Stopped")
    log("Uploader thread stopped")


def esc(v):
    return html.escape("" if v is None else str(v), quote=True)


def render_page(cfg):
    serial_port = esc(cfg.get("serial_port", ""))
    baud_rate = esc(cfg.get("baud_rate", ""))
    caster_host = esc(cfg.get("caster_host", ""))
    caster_port = esc(cfg.get("caster_port", ""))
    username = esc(cfg.get("username", ""))
    password = esc(cfg.get("password", ""))
    mountpoint = esc(cfg.get("mountpoint", ""))
    send_gga = "true" if cfg.get("send_gga") else "false"
    gga_interval_sec = esc(cfg.get("gga_interval_sec", ""))
    reconnect_sec = esc(cfg.get("reconnect_sec", ""))
    init_cmd = esc(cfg.get("init_cmd", ""))
    save_cmd = esc(cfg.get("save_cmd", ""))
    custom_cmd = esc(cfg.get("custom_cmd", ""))

    return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>RTK Base → NTRIP Caster Uploader</title>
  <style>
    :root{{
      --glass: rgba(255,255,255,0.72);
      --stroke: rgba(0,0,0,0.10);
      --text: #0b2a44;
      --muted: rgba(11,42,68,0.70);
      --field: rgba(255,255,255,0.88);
    }}
    body{{
      margin:0;
      min-height:100vh;
      font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial;
      color: var(--text);
      background: linear-gradient(180deg,#cfe9ff 0%,#eaf6ff 60%,#ffffff 100%);
      padding: 26px;
      box-sizing: border-box;
    }}
    .wrap{{
      max-width: 1200px; margin: 0 auto;
    }}
    .card{{
      background: var(--glass);
      border: 1px solid var(--stroke);
      border-radius: 18px;
      padding: 18px;
      backdrop-filter: blur(6px);
      -webkit-backdrop-filter: blur(6px);
      box-shadow: 0 12px 32px rgba(0,60,120,0.15);
    }}
    h1{{
      margin: 0 0 14px 0;
      font-size: 26px;
      letter-spacing: 0.2px;
      font-weight: 800;
    }}
    h3{{
      margin: 0 0 10px 0;
      font-size: 16px;
      font-weight: 800;
      color: #0b2a44;
    }}
    .grid-main{{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 18px;
      align-items: start;
    }}
    .left .grid{{
      display: grid;
      grid-template-columns: 1fr;
      gap: 14px;
    }}
    .right{{
      display:flex;
      flex-direction:column;
      gap: 12px;
    }}
    label{{
      display:block;
      font-size: 13px;
      color: var(--muted);
      margin: 0 0 8px 0;
      font-weight: 650;
    }}
    input, textarea{{
      width: 100%;
      box-sizing: border-box;
      padding: 12px 14px;
      border-radius: 14px;
      border: 1px solid rgba(0,0,0,0.12);
      background: var(--field);
      color: var(--text);
      outline: none;
      font-size: 14px;
    }}
    textarea{{min-height: 92px; resize: vertical; line-height: 1.35;}}
    .btns{{
      margin-top: 14px;
      display: flex;
      flex-wrap: nowrap;
      justify-content: space-between;
      width: 100%;
      gap: 12px;
    }}
    .btns button{{
      white-space: nowrap;
      flex-shrink: 0;
    }}
    button{{
      padding: 11px 16px;
      border-radius: 14px;
      border: 1px solid rgba(0,0,0,0.12);
      background: rgba(255,255,255,0.85);
      color: var(--text);
      cursor: pointer;
      font-size: 14px;
      font-weight: 700;
    }}
    button.primary{{
      background: rgba(39,94,196,0.88);
      border-color: rgba(39,94,196,0.30);
      color: white;
    }}
    button.warn{{
      background: rgba(190,65,65,0.88);
      border-color: rgba(190,65,65,0.30);
      color: white;
    }}
    .stat{{
      padding: 12px 14px;
      border-radius: 14px;
      background: rgba(255,255,255,0.78);
      border: 1px solid rgba(0,0,0,0.10);
      font-size: 13px;
      color: rgba(11,42,68,0.92);
    }}
    .mono{{font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;}}
    .logs{{
      max-height: 420px;
      overflow: auto;
      white-space: pre-wrap;
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255,255,255,0.72);
      border: 1px solid rgba(0,0,0,0.10);
      font-size: 12px;
      color: rgba(11,42,68,0.92);
    }}
    .tag{{
      display:inline-block;
      padding: 4px 10px;
      border-radius: 999px;
      border: 1px solid rgba(0,0,0,0.12);
      margin-right: 8px;
      background: rgba(255,255,255,0.55);
      font-weight: 700;
    }}
    .smallnote{{font-size:12px; color: rgba(11,42,68,0.72); margin-top:8px;}}
  </style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>RTK Base → NTRIP Caster Uploader</h1>

    <div class="grid-main">
      <div class="left">
        <form method="post" action="/save">
          <div class="grid">
            <div>
              <label>Serial device (serial_port)</label>
              <input name="serial_port" value="{serial_port}"/>
            </div>
            <div>
              <label>Baud rate (baud_rate)</label>
              <input name="baud_rate" value="{baud_rate}"/>
            </div>

            <div>
              <label>Caster host / IP (caster_host)</label>
              <input name="caster_host" value="{caster_host}"/>
            </div>
            <div>
              <label>Caster port (caster_port)</label>
              <input name="caster_port" value="{caster_port}"/>
            </div>

            <div>
              <label>Username (username)</label>
              <input name="username" value="{username}"/>
            </div>
            <div>
              <label>Password (password)</label>
              <input name="password" value="{password}"/>
            </div>

            <div>
              <label>Mountpoint (mountpoint)</label>
              <input name="mountpoint" value="{mountpoint}"/>
            </div>
            <div>
              <label>Send GGA (send_gga: true/false)</label>
              <input name="send_gga" value="{send_gga}"/>
            </div>

            <div>
              <label>GGA interval (seconds)</label>
              <input name="gga_interval_sec" value="{gga_interval_sec}"/>
            </div>
            <div>
              <label>Reconnect interval (seconds)</label>
              <input name="reconnect_sec" value="{reconnect_sec}"/>
            </div>
          </div>

          <div class="btns">
            <button class="primary" type="submit">Save configuration</button>
            <button type="button" onclick="fetch('/start',{{method:'POST'}}).then(()=>setTimeout(poll,400));">Start upload</button>
            <button class="warn" type="button" onclick="fetch('/stop',{{method:'POST'}}).then(()=>setTimeout(poll,400));">Stop</button>
            <button type="button" onclick="sendCustom()">Send custom command</button>
            <button type="button" onclick="poll()">Refresh status</button>
          </div>
        </form>
      </div>

      <div class="right">
        <h3>Commands, Status & Upload Log</h3>

        <div>
          <label>RTK Base init command (auto send on Start)</label>
          <input readonly value="{init_cmd}"/>
          <div class="smallnote mono">Default: $POLCFGRTCM,1,1,1</div>
        </div>

        <div>
          <label>RTK Base save command (auto send on Start)</label>
          <input readonly value="{save_cmd}"/>
          <div class="smallnote mono">Default: $POLCFGSAVE</div>
        </div>

        <div>
          <label>Custom command (send now; supports multiple lines)</label>
          <textarea id="custom_cmd" placeholder="$POLCFGRTCM,1,1,1&#10;$POLCFGSAVE">{custom_cmd}</textarea>
          <div class="smallnote mono">Example: $POLCFGSAVE</div>
        </div>

        <div class="stat mono" id="stat">Loading...</div>
        <div class="logs mono" id="logs"></div>
      </div>
    </div>
  </div>
</div>

<script>
async function poll(){{
  const s = await fetch('/status').then(r=>r.json());
  const l = await fetch('/logs').then(r=>r.json());
  const upTime = s.last_up_ts ? new Date(s.last_up_ts*1000).toLocaleString() : '-';
  document.getElementById('stat').innerHTML =
    '<span class="tag">running: ' + s.running + '</span>' +
    '<span class="tag">serial: ' + s.serial + '</span>' +
    '<span class="tag">ntrip: ' + s.ntrip + '</span><br/>' +
    'bytes_up: ' + s.bytes_up + ' | last_up: ' + upTime + '<br/>' +
    'message: ' + (s.message||'') + '<br/>' +
    'last_cmd: ' + (s.last_cmd||'') + '<br/>' +
    'last_err: ' + (s.last_err||'');
  document.getElementById('logs').textContent = l.lines.join('\\n');
}}

async function sendCustom(){{
  const ta = document.getElementById('custom_cmd');
  const cmd = ta ? ta.value : '';
  await fetch('/send_cmd', {{
    method:'POST',
    headers: {{'Content-Type':'application/json'}},
    body: JSON.stringify({{cmd: cmd}})
  }});
  setTimeout(poll, 350);
}}

poll();
setInterval(poll, 1500);
</script>
</body>
</html>"""

def parse_post_body(handler):
    length = int(handler.headers.get("Content-Length", "0") or "0")
    raw = handler.rfile.read(length) if length > 0 else b""
    ctype = handler.headers.get("Content-Type", "")

    if "application/json" in ctype:
        try:
            return json.loads(raw.decode("utf-8", errors="ignore"))
        except Exception:
            return {}

    if "application/x-www-form-urlencoded" in ctype:
        parsed = urllib.parse.parse_qs(raw.decode("utf-8", errors="ignore"), keep_blank_values=True)
        return {k: v[0] if isinstance(v, list) and v else "" for k, v in parsed.items()}

    return {"_raw": raw.decode("utf-8", errors="ignore")}

def start_uploader():
    global uploader_thread
    with run_lock:
        if uploader_thread and uploader_thread.is_alive():
            set_status(message="Already running")
            return
        stop_event.clear()
        uploader_thread = threading.Thread(target=uploader_loop, daemon=True)
        uploader_thread.start()
        set_status(message="Starting...")

def stop_uploader():
    with run_lock:
        stop_event.set()
        set_status(message="Stopping...")

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_text(self, code, text, content_type="text/plain; charset=utf-8"):
        data = text.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def redirect(self, location):
        self.send_response(303)
        self.send_header("Location", location)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    def do_GET(self):
        if self.path == "/":
            cfg = load_config()
            page = render_page(cfg)
            self.send_text(200, page, "text/html; charset=utf-8")
            return

        if self.path == "/status":
            with status_lock:
                out = dict(status)
            self.send_json(out)
            return

        if self.path == "/logs":
            with logs_lock:
                out = {"lines": list(logs)}
            self.send_json(out)
            return

        self.send_text(404, "Not Found")

    def do_POST(self):
        if self.path == "/save":
            form = parse_post_body(self)
            cfg = load_config()

            def get(name, default=""):
                return str(form.get(name, default)).strip()

            try:
                cfg["serial_port"] = get("serial_port", cfg["serial_port"])
                cfg["baud_rate"] = int(get("baud_rate", str(cfg["baud_rate"])) or cfg["baud_rate"])
                cfg["caster_host"] = get("caster_host", cfg["caster_host"])
                cfg["caster_port"] = int(get("caster_port", str(cfg["caster_port"])) or cfg["caster_port"])
                cfg["username"] = get("username", cfg["username"])
                cfg["password"] = get("password", cfg["password"])
                cfg["mountpoint"] = get("mountpoint", cfg["mountpoint"])

                send_gga = get("send_gga", "false").lower()
                cfg["send_gga"] = True if send_gga in ("1", "true", "yes", "y", "on") else False

                cfg["gga_interval_sec"] = float(get("gga_interval_sec", str(cfg["gga_interval_sec"])) or cfg["gga_interval_sec"])
                cfg["reconnect_sec"] = float(get("reconnect_sec", str(cfg["reconnect_sec"])) or cfg["reconnect_sec"])
                cfg["custom_cmd"] = cfg.get("custom_cmd", "")

                with cfg_lock:
                    save_config(cfg)

                self.redirect("/")
            except Exception as e:
                log(f"Save config failed: {e}")
                set_status(last_err=str(e), message="Save config failed")
                self.redirect("/")
            return

        if self.path == "/send_cmd":
            data = parse_post_body(self)
            cmd_text = (data.get("cmd") or "").strip()

            cfg = load_config()
            cfg["custom_cmd"] = cmd_text
            with cfg_lock:
                save_config(cfg)

            lines = normalize_cmd_lines(cmd_text)
            if not lines:
                set_status(message="No command to send")
                self.send_text(200, "OK")
                return

            try:
                if serial_obj is None:
                    raise RuntimeError("Serial not connected (start uploader first, or check port/baud)")
                serial_send_lines(lines)
                set_status(message="Custom command sent")
            except Exception as e:
                set_status(last_err=str(e), message="Custom command send failed")
                log(f"Custom command send failed: {e}")

            self.send_text(200, "OK")
            return

        if self.path == "/start":
            start_uploader()
            self.send_text(200, "OK")
            return

        if self.path == "/stop":
            stop_uploader()
            self.send_text(200, "OK")
            return

        self.send_text(404, "Not Found")


if __name__ == "__main__":
    cfg = load_config()
    save_config(cfg)
    log(f"Serving HTTP on 0.0.0.0:{PORT}")
    print_urls()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        close_all()
        server.server_close()
        log("Server stopped")