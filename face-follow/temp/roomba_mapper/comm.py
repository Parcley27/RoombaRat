"""
comm.py – Non-blocking TCP communication with the ESP32 bridge.
"""
import socket
import select
import time
from typing import Optional


class RoombaComm:

    def __init__(self, ip: str, port: int):
        self._ip   = ip
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._buf  = ""
        self.connected = False

    # ── Connection management ─────────────────────────────────────────────

    def connect(self, timeout: float = 5.0) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((self._ip, self._port))
            s.setblocking(False)
            self._sock = s
            self.connected = True
            print(f"[comm] Connected to {self._ip}:{self._port}")
            return True
        except OSError as exc:
            print(f"[comm] Connection failed: {exc}")
            self.connected = False
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self.connected = False

    # ── Outgoing commands ─────────────────────────────────────────────────

    def send_drive(self, left_mms: int, right_mms: int):
        self._send(f"DRIVE {int(left_mms)} {int(right_mms)}\n")

    def send_stop(self):
        self._send("STOP\n")

    def send_safe(self):
        self._send("SAFE\n")

    def _send(self, text: str):
        if not self.connected or self._sock is None:
            return
        try:
            self._sock.sendall(text.encode())
        except OSError as exc:
            print(f"[comm] Send error: {exc}")
            self.connected = False

    # ── Incoming sensor data ──────────────────────────────────────────────

    def read_sensors(self) -> Optional[dict]:
        """
        Drain the receive buffer and return the *latest* complete sensor
        packet, or None if no complete packet is available yet.

        Sensor line format (from ESP32):
          S <bumpsDrops> <wall> <cliffL> <cliffFL> <cliffFR> <cliffR>
            <overcurrent> <leftEnc> <rightEnc>
        """
        if not self.connected or self._sock is None:
            return None

        # Non-blocking read
        try:
            ready, _, _ = select.select([self._sock], [], [], 0)
            if ready:
                chunk = self._sock.recv(4096)
                if not chunk:
                    print("[comm] Connection closed by ESP32")
                    self.connected = False
                    return None
                self._buf += chunk.decode("ascii", errors="ignore")
        except BlockingIOError:
            pass
        except OSError as exc:
            print(f"[comm] Recv error: {exc}")
            self.connected = False
            return None

        # Parse all complete lines, keep the last sensor packet
        latest = None
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if not line.startswith("S "):
                continue
            parts = line.split()
            if len(parts) != 10:
                continue
            try:
                latest = {
                    "bumps_drops": int(parts[1]),
                    "wall":        int(parts[2]),
                    "cliff_l":     int(parts[3]),
                    "cliff_fl":    int(parts[4]),
                    "cliff_fr":    int(parts[5]),
                    "cliff_r":     int(parts[6]),
                    "overcurrent": int(parts[7]),
                    "left_enc":    int(parts[8]),
                    "right_enc":   int(parts[9]),
                }
            except ValueError:
                pass

        return latest
