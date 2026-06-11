#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Compass-IST8310.py

import time
import math
import struct
import threading
import argparse
import atexit
import json
import socket
import fcntl

from smbus2 import SMBus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ===== IST8310 registers =====
REG_WAI     = 0x00
WAI_VALUE   = 0x10
REG_STAT1   = 0x02
DRDY_MASK   = 0x01
REG_DATAXL  = 0x03  # 0x03..0x08
REG_CNTL1   = 0x0A  # 0x01 single measurement
REG_CNTL2   = 0x0B  # bit0 soft reset
REG_AVGCNTL = 0x41  # set 0x24
REG_PDCNTL  = 0x42  # set 0xC0


def log(msg):
    print(msg, flush=True)


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=1666)

    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x0E)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--sleep", action="store_true")

    ap.add_argument("--swap-xy", action="store_true")
    ap.add_argument("--invert-x", action="store_true")
    ap.add_argument("--invert-y", action="store_true")
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--declination", type=float, default=0.0)
    ap.add_argument("--vec-alpha", type=float, default=0.35)

    ap.add_argument("--verbose", action="store_true")
    return ap.parse_args()


class IST8310:
    def __init__(self, bus_no=1, addr=0x0E, verbose=False):
        self.bus_no = bus_no
        self.addr = addr
        self.verbose = verbose
        self.bus = None

    def open(self):
        self.bus = SMBus(self.bus_no)
        self._check_wai_robust()
        self.soft_reset()

        self.write_u8(REG_CNTL1, 0x00)
        time.sleep(0.01)

        self.write_u8(REG_AVGCNTL, 0x24)
        self.write_u8(REG_PDCNTL,  0xC0)
        time.sleep(0.01)

        if self.verbose:
            wai = self.read_u8(REG_WAI)
            log(f"[IST8310] opened, addr=0x{self.addr:02X}, WAI=0x{wai:02X}")

    def close(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

    def read_u8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def write_u8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _check_wai_robust(self, tries=7, gap_s=0.01):
        vals = []
        for _ in range(tries):
            try:
                v = self.read_u8(REG_WAI)
                vals.append(v)
            except OSError:
                vals.append(None)
            time.sleep(gap_s)

        if vals.count(WAI_VALUE) == 0:
            last = next((v for v in reversed(vals) if v is not None), None)
            raise RuntimeError(f"WAI mismatch. samples={vals}, last={last}")

    def soft_reset(self):
        try:
            self.write_u8(REG_CNTL2, 0x01)
            time.sleep(0.01)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.1:
                v = self.read_u8(REG_CNTL2)
                if (v & 0x01) == 0:
                    return
                time.sleep(0.005)
        except OSError:
            if self.verbose:
                log("[WARN] soft_reset OSError, continue...")

    def read_xyz_raw(self, poll=True, timeout_s=0.05):
        self.write_u8(REG_CNTL1, 0x01)

        if poll:
            t0 = time.monotonic()
            while True:
                st = self.read_u8(REG_STAT1)
                if st & DRDY_MASK:
                    break
                if time.monotonic() - t0 > timeout_s:
                    raise TimeoutError("DRDY timeout")
                time.sleep(0.001)
        else:
            time.sleep(0.0065)

        data = self.bus.read_i2c_block_data(self.addr, REG_DATAXL, 6)
        x, y, z = struct.unpack("<hhh", bytes(data))
        return x, y, z


def wrap360(deg):
    return (deg % 360.0 + 360.0) % 360.0


def vec_lerp(prev, cur, alpha):
    if prev is None:
        return cur
    return (
        prev[0] + (cur[0] - prev[0]) * alpha,
        prev[1] + (cur[1] - prev[1]) * alpha,
        prev[2] + (cur[2] - prev[2]) * alpha,
    )


def compute_heading_deg_from_xy(x, y, swap_xy=False, invert_x=False, invert_y=False,
                                offset_deg=0.0, declination_deg=0.0):
    if swap_xy:
        x, y = y, x
    if invert_x:
        x = -x
    if invert_y:
        y = -y

    hdg = math.degrees(math.atan2(x, y))
    hdg = wrap360(hdg + offset_deg + declination_deg)
    return hdg


state_lock = threading.Lock()
cal_lock = threading.Lock()
stop_evt = threading.Event()

worker_th = None
dev = None

state = {
    "ok": False,
    "x": 0,
    "y": 0,
    "z": 0,
    "x_cal": 0.0,
    "y_cal": 0.0,
    "heading": 0.0,
    "error": "",
    "cal": {
        "min_x": None,
        "max_x": None,
        "min_y": None,
        "max_y": None,
        "ready": False
    }
}

cal = {
    "min_x": None,
    "max_x": None,
    "min_y": None,
    "max_y": None,
}


def cal_reset():
    with cal_lock:
        cal["min_x"] = None
        cal["max_x"] = None
        cal["min_y"] = None
        cal["max_y"] = None

    with state_lock:
        state["cal"]["min_x"] = None
        state["cal"]["max_x"] = None
        state["cal"]["min_y"] = None
        state["cal"]["max_y"] = None
        state["cal"]["ready"] = False

    log("[CAL] reset")


def cal_update(x, y):
    with cal_lock:
        if cal["min_x"] is None:
            cal["min_x"] = cal["max_x"] = x
            cal["min_y"] = cal["max_y"] = y
        else:
            cal["min_x"] = min(cal["min_x"], x)
            cal["max_x"] = max(cal["max_x"], x)
            cal["min_y"] = min(cal["min_y"], y)
            cal["max_y"] = max(cal["max_y"], y)


def cal_apply(x, y):
    with cal_lock:
        min_x = cal["min_x"]
        max_x = cal["max_x"]
        min_y = cal["min_y"]
        max_y = cal["max_y"]

    if min_x is None:
        return float(x), float(y), False, (None, None, None, None)

    cx = (min_x + max_x) / 2.0
    cy = (min_y + max_y) / 2.0
    rx = (max_x - min_x) / 2.0
    ry = (max_y - min_y) / 2.0

    if rx < 1e-6 or ry < 1e-6:
        return float(x - cx), float(y - cy), False, (min_x, max_x, min_y, max_y)

    x0 = (x - cx) / rx
    y0 = (y - cy) / ry

    ready = (rx > 50 and ry > 50)
    return float(x0), float(y0), ready, (min_x, max_x, min_y, max_y)


HTML = r"""
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>IST8310 Compass</title>
<style>
:root{ --bg:#0a0d12; --cyan:#37d6ff; --text:#e9f2ff; --muted:#8aa0b8; }
*{box-sizing:border-box;}
body{
  margin:0; min-height:100vh; display:flex; align-items:center; justify-content:center;
  background:
    radial-gradient(1200px 600px at 50% 15%, #162033 0%, transparent 60%),
    radial-gradient(900px 500px at 50% 100%, #0c1220 0%, transparent 70%),
    var(--bg);
  color:var(--text);
  font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial;
}
.card{
  width:min(860px, 92vw);
  display:grid; grid-template-columns: 420px 1fr; gap:22px;
  padding:22px; border-radius:18px;
  background: linear-gradient(180deg, rgba(255,255,255,0.06), rgba(255,255,255,0.02));
  border: 1px solid rgba(255,255,255,0.08);
  box-shadow: 0 20px 60px rgba(0,0,0,0.45);
}
@media (max-width: 780px){ .card{ grid-template-columns:1fr; } }
.compass{ width:380px; height:380px; margin:0 auto; display:grid; place-items:center; }
.dial{
  width:100%; height:100%; border-radius:999px; position:relative; overflow:hidden;
  background:
    radial-gradient(circle at 50% 50%, rgba(55,214,255,0.22) 0%, rgba(10,167,255,0.08) 45%, rgba(0,0,0,0) 70%),
    radial-gradient(circle at 50% 50%, #0c1220 0%, #070a10 72%, #05070b 100%);
  box-shadow: inset 0 0 0 18px rgba(0,0,0,0.55), inset 0 0 40px rgba(55,214,255,0.18), 0 25px 55px rgba(0,0,0,0.55);
}
.ticks{
  position:absolute; inset:14px; border-radius:999px; pointer-events:none;
  background: repeating-conic-gradient(from -90deg, rgba(55,214,255,0.65) 0deg 1deg, rgba(55,214,255,0.00) 1deg 6deg);
  mask: radial-gradient(circle at 50% 50%, transparent 0 62%, #000 64% 100%);
  opacity:0.85;
}
.glow{
  position:absolute; inset:-30px; border-radius:999px;
  background: radial-gradient(circle at 50% 50%, rgba(55,214,255,0.26) 0%, rgba(10,167,255,0.10) 45%, rgba(0,0,0,0) 70%);
  filter: blur(2px); pointer-events:none;
}
.mark{ position:absolute; font-weight:800; letter-spacing:1px; user-select:none; color: rgba(233,242,255,0.92); text-shadow: 0 0 10px rgba(55,214,255,0.35); }
.mark.n{ top:18px; left:50%; transform:translateX(-50%); font-size:22px; }
.mark.s{ bottom:18px; left:50%; transform:translateX(-50%); font-size:18px; opacity:.85;}
.mark.e{ right:18px; top:50%; transform:translateY(-50%); font-size:18px; opacity:.85;}
.mark.w{ left:18px; top:50%; transform:translateY(-50%); font-size:18px; opacity:.85;}

.needle{ position:absolute; inset:0; transform:rotate(0deg); transition: transform 60ms linear; }
.needle-head,.needle-tail{ position:absolute; left:50%; transform:translateX(-50%); width:16px; border-radius:999px; }
.needle-head{ top:70px; height:130px; background: linear-gradient(180deg, rgba(55,214,255,0.0), rgba(55,214,255,0.95) 22%, rgba(55,214,255,0.25)); box-shadow: 0 0 18px rgba(55,214,255,0.55); clip-path: polygon(50% 0%, 65% 10%, 65% 100%, 35% 100%, 35% 10%); }
.needle-tail{ bottom:95px; height:95px; background: linear-gradient(180deg, rgba(255,80,110,0.20), rgba(255,80,110,0.92) 60%, rgba(255,80,110,0.0)); box-shadow: 0 0 16px rgba(255,80,110,0.35); clip-path: polygon(50% 100%, 65% 90%, 65% 0%, 35% 0%, 35% 90%); }
.needle-center{ position:absolute; left:50%; top:50%; width:18px; height:18px; transform:translate(-50%,-50%); border-radius:999px; background: radial-gradient(circle at 30% 30%, rgba(255,255,255,0.9), rgba(55,214,255,0.45) 35%, rgba(0,0,0,0.2) 75%); box-shadow: 0 0 18px rgba(55,214,255,0.35); }
.cap{ position:absolute; left:50%; top:50%; width:64px; height:64px; transform:translate(-50%,-50%); border-radius:999px; background: radial-gradient(circle at 30% 30%, rgba(255,255,255,0.08), rgba(0,0,0,0.35) 55%, rgba(0,0,0,0.7)); box-shadow: inset 0 0 20px rgba(0,0,0,0.65); }

.info{ display:flex; flex-direction:column; justify-content:center; gap:12px; }
.row{ display:flex; align-items:baseline; justify-content:space-between; gap:10px; }
.big{ font-size:54px; font-weight:900; letter-spacing:-1px; }
.unit{ font-size:18px; margin-left:6px; color:var(--muted); font-weight:700; }
.sub{ font-size:18px; color:var(--muted); font-weight:700; }
.small{ font-size:14px; color: rgba(233,242,255,0.78); }
.err{ min-height:18px; color: rgba(255,120,140,0.95); font-size:13px; }
.btn{
  appearance:none; border:none; cursor:pointer;
  padding:10px 12px; border-radius:12px;
  background: rgba(55,214,255,0.14);
  color: rgba(233,242,255,0.92);
  border: 1px solid rgba(55,214,255,0.25);
}
.btn:active{ transform: translateY(1px); }
.pill{
  display:inline-block; padding:4px 8px; border-radius:999px;
  background: rgba(255,255,255,0.06);
  border: 1px solid rgba(255,255,255,0.10);
  color: rgba(233,242,255,0.80);
  font-size:12px;
}
</style>
</head>
<body>
  <div class="card">
    <div class="compass">
      <div class="dial">
        <div class="mark n">N</div>
        <div class="mark e">E</div>
        <div class="mark s">S</div>
        <div class="mark w">W</div>
        <div class="ticks"></div>
        <div class="glow"></div>
        <div class="needle" id="needle">
          <div class="needle-head"></div>
          <div class="needle-tail"></div>
          <div class="needle-center"></div>
        </div>
        <div class="cap"></div>
      </div>
    </div>

    <div class="info">
      <div class="row">
        <div class="big"><span id="deg">--.-</span><span class="unit">°</span></div>
        <div class="sub" id="dir">---</div>
      </div>

      <div class="row small">
        <div>RAW: <span id="raw">x=0 y=0 z=0</span></div>
        <div>Status: <span id="status">--</span></div>
      </div>

      <div class="row small">
        <div>CAL: <span class="pill" id="calstate">not ready</span></div>
        <button class="btn" onclick="resetCal()">Reset Cal</button>
      </div>

      <div class="row err" id="err"></div>

      <div class="row small" style="opacity:.75">
        <div>Tip: After startup, keep the board level and rotate it slowly for a full turn to calibrate.</div>
      </div>
    </div>
  </div>

<script>
function degToDir(deg){
  const dirs=["N","NNE","NE","ENE","E","ESE","SE","SSE","S","SSW","SW","WSW","W","WNW","NW","NNW"];
  const idx = Math.round(deg/22.5) % 16;
  return dirs[idx];
}

async function resetCal(){
  try{
    await fetch("/cal/reset", {method:"POST", cache:"no-store"});
  }catch(e){}
}

async function poll(){
  try{
    const r = await fetch("/heading", {cache:"no-store"});
    const d = await r.json();
    const hdg = d.heading ?? 0;

    document.getElementById("needle").style.transform = `rotate(${hdg}deg)`;
    document.getElementById("deg").textContent = hdg.toFixed(1);
    document.getElementById("dir").textContent = degToDir(hdg);

    document.getElementById("raw").textContent = `x=${d.x} y=${d.y} z=${d.z}`;
    document.getElementById("status").textContent = d.ok ? "OK" : "WARN";
    document.getElementById("err").textContent = d.error ? d.error : "";
    document.getElementById("calstate").textContent = d.cal_ready ? "ready" : "not ready";
  }catch(e){
    document.getElementById("status").textContent = "ERR";
    document.getElementById("err").textContent = String(e);
  }
}
setInterval(poll, 120);
poll();
</script>
</body>
</html>
"""


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


def print_urls(host, port):
    if host not in ("0.0.0.0", "::"):
        log(f"URL: http://{host}:{port}")
        return

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
            log(f"{ifname}: http://{ip}:{port}")
            found = True
    except Exception:
        pass

    log(f"localhost: http://127.0.0.1:{port}")
    if not found:
        log(f"open: http://127.0.0.1:{port}")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_bytes(self, code, data, content_type):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, obj, code=200):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_bytes(code, data, "application/json; charset=utf-8")

    def send_html(self, html_text, code=200):
        data = html_text.encode("utf-8")
        self.send_bytes(code, data, "text/html; charset=utf-8")

    def send_text(self, text, code=200):
        data = text.encode("utf-8")
        self.send_bytes(code, data, "text/plain; charset=utf-8")

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            self.send_html(HTML)
            return

        if self.path == "/heading":
            with state_lock:
                out = {
                    "ok": bool(state["ok"]),
                    "heading": float(state["heading"]),
                    "x": int(state["x"]),
                    "y": int(state["y"]),
                    "z": int(state["z"]),
                    "error": state["error"],
                    "cal_ready": bool(state["cal"]["ready"]),
                }
            self.send_json(out)
            return

        if self.path == "/favicon.ico":
            self.send_bytes(204, b"", "image/x-icon")
            return

        self.send_text("Not Found", 404)

    def do_POST(self):
        if self.path == "/cal/reset":
            cal_reset()
            self.send_json({"ok": True})
            return

        self.send_text("Not Found", 404)


def worker_loop(args):
    global dev

    period = 1.0 / max(args.hz, 0.1)
    v_prev = None
    last_init_ok = False

    while not stop_evt.is_set():
        if dev is None:
            try:
                dev = IST8310(bus_no=args.bus, addr=args.addr, verbose=args.verbose)
                dev.open()
                last_init_ok = True
                log(f"[IST8310] sensor ready on I2C bus={args.bus}, addr=0x{args.addr:02X}")
                with state_lock:
                    state["ok"] = True
                    state["error"] = ""
            except Exception as e:
                last_init_ok = False
                with state_lock:
                    state["ok"] = False
                    state["error"] = f"init failed: {e}"
                log(f"[ERR] init failed: {e}")
                if dev:
                    try:
                        dev.close()
                    except Exception:
                        pass
                dev = None
                time.sleep(1.0)
                continue

        try:
            x, y, z = dev.read_xyz_raw(poll=(not args.sleep))

            cal_update(x, y)
            x_cal, y_cal, ready, mm = cal_apply(x, y)

            v_prev = vec_lerp(v_prev, (x_cal, y_cal, float(z)), args.vec_alpha)
            xs, ys, _ = v_prev

            heading = compute_heading_deg_from_xy(
                xs, ys,
                swap_xy=args.swap_xy,
                invert_x=args.invert_x,
                invert_y=args.invert_y,
                offset_deg=args.offset,
                declination_deg=args.declination
            )

            with state_lock:
                state["ok"] = True
                state["x"] = int(x)
                state["y"] = int(y)
                state["z"] = int(z)
                state["x_cal"] = float(xs)
                state["y_cal"] = float(ys)
                state["heading"] = float(heading)
                state["error"] = ""
                state["cal"]["ready"] = bool(ready)
                state["cal"]["min_x"], state["cal"]["max_x"], state["cal"]["min_y"], state["cal"]["max_y"] = mm

        except Exception as e:
            with state_lock:
                state["ok"] = False
                state["error"] = f"read error: {e}"

            log(f"[ERR] read error: {e}")

            try:
                dev.soft_reset()
                dev.write_u8(REG_CNTL1, 0x00)
                time.sleep(0.01)
                dev.write_u8(REG_AVGCNTL, 0x24)
                dev.write_u8(REG_PDCNTL, 0xC0)
                time.sleep(0.01)
                if args.verbose:
                    log("[IST8310] recovery done")
            except Exception as e2:
                with state_lock:
                    state["error"] = f"recovery failed: {e2}"
                log(f"[ERR] recovery failed: {e2}")
                try:
                    dev.close()
                except Exception:
                    pass
                dev = None
                time.sleep(0.3)

        time.sleep(period)

    if dev:
        try:
            dev.close()
        except Exception:
            pass
        dev = None

    if last_init_ok:
        log("[IST8310] worker stopped")


def shutdown():
    global worker_th, dev
    stop_evt.set()

    if worker_th and worker_th.is_alive():
        worker_th.join(timeout=1.0)

    if dev:
        try:
            dev.close()
        except Exception:
            pass
        dev = None


atexit.register(shutdown)


def main():
    global worker_th

    args = parse_args()
    cal_reset()

    log("Starting Compass-IST8310...")
    log(f"I2C bus={args.bus}, addr=0x{args.addr:02X}, hz={args.hz}, host={args.host}, port={args.port}")

    worker_th = threading.Thread(target=worker_loop, args=(args,), daemon=True)
    worker_th.start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)

    log(f"Serving HTTP on {args.host}:{args.port}")
    print_urls(args.host, args.port)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("KeyboardInterrupt, exiting...")
    finally:
        stop_evt.set()
        try:
            server.server_close()
        except Exception:
            pass
        shutdown()
        log("Server stopped")


if __name__ == "__main__":
    main()