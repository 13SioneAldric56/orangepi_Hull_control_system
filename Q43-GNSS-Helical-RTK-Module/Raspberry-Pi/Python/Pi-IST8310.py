#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Pi-IST8310.py

import time
import struct
import csv
import argparse
from smbus2 import SMBus

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

UT_PER_LSB = 0.3  # 常见用法；如你要严格换算可后续再校准

class IST8310:
    def __init__(self, bus_no=1, addr=0x0E, verbose=True):
        self.bus_no = bus_no
        self.addr = addr
        self.verbose = verbose
        self.bus = None

    def open(self):
        self.bus = SMBus(self.bus_no)
        self._check_wai_robust()

        self.soft_reset()

        # standby
        self.write_u8(REG_CNTL1, 0x00)
        time.sleep(0.01)

        # set recommended registers (verified in your i2cdump: 0x41=0x24, 0x42=0xC0)
        self.write_u8(REG_AVGCNTL, 0x24)
        self.write_u8(REG_PDCNTL,  0xC0)
        time.sleep(0.01)

        if self.verbose:
            wai = self.read_u8(REG_WAI)
            print(f"[IST8310] : addr=0x{self.addr:02X}, WAI=0x{wai:02X}")

    def close(self):
        if self.bus:
            self.bus.close()
            self.bus = None

    def read_u8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def write_u8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _check_wai_robust(self, tries=7, gap_s=0.01):
        """
        多次读取 WAI，避免偶发返回异常值（你之前看到 0x01）。
        """
        vals = []
        for _ in range(tries):
            try:
                v = self.read_u8(REG_WAI)
                vals.append(v)
            except OSError:
                vals.append(None)
            time.sleep(gap_s)

        good = vals.count(WAI_VALUE)
        if self.verbose:
            printable = [("None" if v is None else f"0x{v:02X}") for v in vals]
            print(f"[IST8310] WAI samples: {printable} (0x10 count={good})")

        if good == 0:
            last = next((v for v in reversed(vals) if v is not None), None)
            raise RuntimeError(f"WAI read failed/mismatch. samples={vals}, last={last}")

    def soft_reset(self):
        # CNTL2 bit0 = 1 triggers reset, then self-clears
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
            # if something weird, ignore; we'll still try to run
            if self.verbose:
                print("[WARN] soft_reset OSError, continue...")

    def read_xyz_raw(self, poll=True, timeout_s=0.05):
        """
        单次测量：
        1) CNTL1=0x01
        2) 等 DRDY 或 sleep>6ms
        3) 读 6 字节
        """
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

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x0E)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--sleep", action="store_true", help="use fixed sleep >6ms instead of DRDY polling")
    ap.add_argument("--csv", type=str, default="")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    period = 1.0 / max(args.hz, 0.1)
    poll = not args.sleep
    verbose = not args.quiet

    dev = IST8310(bus_no=args.bus, addr=args.addr, verbose=verbose)

    csv_f = None
    writer = None
    if args.csv:
        csv_f = open(args.csv, "w", newline="")
        writer = csv.writer(csv_f)
        writer.writerow(["t", "x_raw", "y_raw", "z_raw", "x_uT", "y_uT", "z_uT"])

    t0 = time.monotonic()

    try:
        dev.open()
        print("[IST8310] running... Ctrl+C to stop")

        while True:
            t = time.monotonic() - t0
            try:
                x, y, z = dev.read_xyz_raw(poll=poll)
            except (OSError, TimeoutError) as e:
                # 自动恢复：重置并继续跑
                print(f"[WARN] read error: {e}. reset & retry...")
                try:
                    dev.soft_reset()
                    dev.write_u8(REG_CNTL1, 0x00)
                    time.sleep(0.01)
                    dev.write_u8(REG_AVGCNTL, 0x24)
                    dev.write_u8(REG_PDCNTL,  0xC0)
                    time.sleep(0.01)
                except Exception as e2:
                    print(f"[WARN] recovery failed: {e2}")
                time.sleep(0.05)
                continue

            xu, yu, zu = x * UT_PER_LSB, y * UT_PER_LSB, z * UT_PER_LSB

            if verbose:
                print(f"t={t:7.3f}s RAW:{x:6d},{y:6d},{z:6d}   uT:{xu:7.1f},{yu:7.1f},{zu:7.1f}")

            if writer:
                writer.writerow([f"{t:.6f}", x, y, z, f"{xu:.3f}", f"{yu:.3f}", f"{zu:.3f}"])

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n[IST8310] stop.")
    finally:
        dev.close()
        if csv_f:
            csv_f.close()

if __name__ == "__main__":
    main()
