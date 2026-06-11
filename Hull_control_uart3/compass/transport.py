"""串口传输层。"""

from __future__ import annotations

import time
from typing import Optional

import serial

from . import protocol as proto


class SerialTransport:
  def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
    self.port = port
    self.baudrate = baudrate
    self.timeout = timeout
    self._serial: Optional[serial.Serial] = None
    self._buffer = bytearray()

  def connect(self) -> bool:
    try:
      self._serial = serial.Serial(
        port=self.port,
        baudrate=self.baudrate,
        timeout=self.timeout,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
      )
      self.clear_buffer()
      return True
    except serial.SerialException:
      return False

  def disconnect(self) -> None:
    if self._serial and self._serial.is_open:
      self._serial.close()
    self._serial = None
    self._buffer.clear()

  @property
  def is_connected(self) -> bool:
    return self._serial is not None and self._serial.is_open

  def clear_buffer(self) -> None:
    if self._serial and self._serial.is_open:
      self._serial.reset_input_buffer()
    self._buffer.clear()

  def write(self, data: bytes) -> bool:
    if not self.is_connected:
      return False
    try:
      self._serial.write(data)
      return True
    except serial.SerialException:
      return False

  def read_frame(self, timeout: float | None = None) -> Optional[bytes]:
    if not self.is_connected:
      return None
    timeout = self.timeout if timeout is None else timeout
    start = time.time()
    while time.time() - start < timeout:
      if self._serial.in_waiting > 0:
        self._buffer.extend(self._serial.read(self._serial.in_waiting))
        frame = proto.find_frame(self._buffer)
        if frame:
          return frame
      time.sleep(0.001)
    return None
