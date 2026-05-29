# main.py – MicroPython ESP32 Roomba OI bridge
#
# In Thonny: File → Save copy → MicroPython device → main.py
#
# Wiring:
#   GPIO 4  → Roomba Mini-DIN pin 5 (BRC)
#   GPIO 25 ← Roomba Mini-DIN pin 4 (Roomba TXD → ESP32 RX)
#   GPIO 26 → Roomba Mini-DIN pin 3 (Roomba RXD ← ESP32 TX)
#   GND     → Roomba Mini-DIN pin 6 or 7

import network
import socket
import time
import machine

# ── WiFi ──────────────────────────────────────────────────────────────────────
WIFI_SSID = "The wifi"
WIFI_PASS = "6394701868"
TCP_PORT  = 8080

# ── Pin assignments ───────────────────────────────────────────────────────────
BRC_PIN = 4
RX_PIN  = 26
TX_PIN  = 25

# ── Roomba OI opcodes ─────────────────────────────────────────────────────────
OI_START        = 128
OI_SAFE         = 131
OI_FULL         = 132
OI_DRIVE_DIRECT = 145
OI_QUERY_LIST   = 149

SONGS = {
    'beep':  [(72, 16), (72, 16)],                          # two short C5 blips
    'alert': [(80, 8), (76, 8), (80, 8)],                   # ascending chirp
    'happy': [(60, 16), (64, 16), (67, 16), (72, 32)],      # C major arpeggio
}

def read_battery():
    # Request packet 25 (charge) and 26 (capacity) via Query List opcode 149
    _send([149, 2, 25, 26])
    time.sleep_ms(50)
    data = uart.read(4)
    if data is None or len(data) < 4:
        print("Battery: no response from Roomba")
        return
    charge   = (data[0] << 8) | data[1]   # mAh
    capacity = (data[2] << 8) | data[3]   # mAh
    if capacity > 0:
        pct = charge * 100 // capacity
        print(f"Battery: {charge}/{capacity} mAh  ({pct}%)")
    else:
        print(f"Battery charge: {charge} mAh (capacity unknown)")

def _send(data):
    uart.write(bytes(data))

def define_song(slot, notes):
    payload = [140, slot, len(notes)]
    for note, dur in notes:
        payload += [note, dur]
    _send(payload)

def play_song(slot):
    _send([141, slot])

# ── Sensor query (9 packets → 11 bytes response) ──────────────────────────────
QUERY_PACKETS  = bytes([7, 8, 9, 10, 11, 12, 14, 43, 44])
RESPONSE_BYTES = 11

# ── Hardware init ─────────────────────────────────────────────────────────────
brc  = machine.Pin(BRC_PIN, machine.Pin.OUT, value=1)
uart = machine.UART(1, baudrate=115200, bits=8, parity=None, stop=1,
                    rx=RX_PIN, tx=TX_PIN, rxbuf=256)

roomba_ready = False   # set True only after Roomba confirms OI is active

# ── Roomba helpers ────────────────────────────────────────────────────────────

def flush_uart():
    if uart.any():
        uart.read(uart.any())

def wake_roomba():
    """BRC low-pulse sequence to wake Roomba from sleep."""
    brc.value(1); time.sleep_ms(200)
    brc.value(0); time.sleep_ms(700)
    brc.value(1); time.sleep_ms(500)

def send_oi(data):
    uart.write(bytes(data) if not isinstance(data, bytes) else data)

def drive(left_mms, right_mms):
    if not roomba_ready:
        return
    left_mms  = max(-500, min(500, left_mms))
    right_mms = max(-500, min(500, right_mms))
    send_oi([
        OI_DRIVE_DIRECT,
        (right_mms >> 8) & 0xFF, right_mms & 0xFF,
        (left_mms  >> 8) & 0xFF, left_mms  & 0xFF,
    ])

def query_sensors():
    flush_uart()
    send_oi(bytes([OI_QUERY_LIST, len(QUERY_PACKETS)]) + QUERY_PACKETS)
    deadline = time.ticks_add(time.ticks_ms(), 50)
    while uart.any() < RESPONSE_BYTES:
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            return None
        time.sleep_us(300)
    d = uart.read(RESPONSE_BYTES)
    if d is None or len(d) < RESPONSE_BYTES:
        return None
    return (
        d[0], d[1],
        d[2], d[3], d[4], d[5],
        d[6],
        (d[7] << 8) | d[8],
        (d[9] << 8) | d[10],
    )

# ── Roomba initialisation with diagnostics ────────────────────────────────────

def init_roomba():
    global roomba_ready
    roomba_ready = False

    print("")
    print("=== Roomba Init ===")

    # Step 1 – BRC wake pulse
    print("[1] BRC wake pulse...")
    flush_uart()
    wake_roomba()

    # Step 2 – OI START (enters Passive mode from any state)
    print("[2] Sending OI START...")
    flush_uart()
    send_oi([OI_START])
    time.sleep_ms(300)

    # Step 3 – Request Safe mode
    print("[3] Sending SAFE mode...")
    send_oi([OI_SAFE])
    time.sleep_ms(300)

    # Step 4 – Verify by querying the bumper byte (packet 7)
    print("[4] Verifying Roomba responds to sensor query...")
    for attempt in range(1, 4):
        flush_uart()
        send_oi([OI_QUERY_LIST, 1, 7])   # single packet: bumps & drops
        time.sleep_ms(60)
        n = uart.any()
        if n >= 1:
            resp = uart.read(n)
            print("    Got {} byte(s) back: {}  <-- Roomba is alive!".format(n, resp))
            roomba_ready = True
            break
        print("    Attempt {}/3: no response yet".format(attempt))
        time.sleep_ms(300)

    if roomba_ready:
        print("[OK] Roomba OI active. Safe mode on.")
        define_song(0, SONGS['happy'])
        play_song(0)
        read_battery()
    else:
        print("")
        print("!!! Roomba did NOT respond !!!")
        print("    Possible causes:")
        print("    1. Roomba is OFF - press the CLEAN button to turn it on,")
        print("       then press Ctrl-D in Thonny to reboot the ESP32.")
        print("    2. Wiring: GPIO25->pin4(RoombaRX), GPIO26->pin3(RoombaRX)")
        print("       confirm with a multimeter on the Mini-DIN connector.")
        print("    3. Baud rate mismatch - Roomba 600 boots at 115200 baud.")
        print("    The server will still start but drive commands are blocked.")
    print("===================")
    print("")
    return roomba_ready

# ── Command handler ───────────────────────────────────────────────────────────

def handle_command(raw):
    global roomba_ready
    line = raw.strip()
    if line.startswith("DRIVE "):
        parts = line.split()
        if len(parts) == 3:
            try:
                drive(int(parts[1]), int(parts[2]))
            except ValueError:
                pass
    elif line == "STOP":
        drive(0, 0)
    elif line == "SAFE":
        send_oi([OI_SAFE])
    elif line == "FULL":
        send_oi([OI_FULL])
    elif line == "REINIT":
        # Laptop can request a re-init if it notices no sensor data
        init_roomba()

# ── WiFi ──────────────────────────────────────────────────────────────────────

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        print("WiFi already up:", wlan.ifconfig()[0])
        return
    wlan.connect(WIFI_SSID, WIFI_PASS)
    print("Connecting to WiFi", end="")
    for _ in range(30):
        if wlan.isconnected():
            break
        time.sleep(0.5)
        print(".", end="")
    if not wlan.isconnected():
        raise OSError("WiFi failed - check SSID/password")
    print("\nConnected! IP:", wlan.ifconfig()[0])
    print(">>> Put this IP in roomba_mapper/config.py as ESP32_IP <<<")

# ── TCP server ────────────────────────────────────────────────────────────────

def run_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", TCP_PORT))
    srv.listen(1)
    srv.setblocking(False)
    print("TCP server ready on port", TCP_PORT)

    conn   = None
    rx_buf = b""

    SENSOR_PERIOD_MS = 50
    SAFE_PERIOD_MS   = 5000
    # How many consecutive sensor timeouts before we print a warning
    TIMEOUT_WARN     = 40

    t_sensor      = time.ticks_ms()
    t_safe        = time.ticks_ms()
    timeout_count = 0

    while True:

        # Accept new connection
        if conn is None:
            try:
                conn, addr = srv.accept()
                conn.setblocking(False)
                rx_buf = b""
                timeout_count = 0
                print("Laptop connected from", addr[0])
                if not roomba_ready:
                    print("WARNING: Roomba not ready - sending REINIT hint to laptop")
            except OSError:
                pass

        # Read commands
        if conn is not None:
            try:
                chunk = conn.recv(256)
                if chunk:
                    rx_buf += chunk
                    while b"\n" in rx_buf:
                        line, rx_buf = rx_buf.split(b"\n", 1)
                        handle_command(line.decode("ascii", "ignore"))
                else:
                    raise OSError("closed")
            except OSError as exc:
                if exc.args[0] not in (11,):   # 11 = EAGAIN
                    print("Client disconnected:", exc)
                    conn.close()
                    conn = None
                    drive(0, 0)

        now = time.ticks_ms()

        # Re-assert Safe mode periodically (cliff drops Roomba to Passive)
        if time.ticks_diff(now, t_safe) >= SAFE_PERIOD_MS:
            t_safe = now
            if roomba_ready:
                send_oi([OI_SAFE])

        # Stream sensors at 20 Hz
        if time.ticks_diff(now, t_sensor) >= SENSOR_PERIOD_MS:
            t_sensor = now
            s = query_sensors()
            if s is not None:
                timeout_count = 0
                if conn is not None:
                    msg = "S {} {} {} {} {} {} {} {} {}\n".format(*s)
                    try:
                        conn.send(msg.encode())
                    except OSError:
                        conn.close()
                        conn = None
                        drive(0, 0)
            else:
                timeout_count += 1
                if timeout_count == TIMEOUT_WARN:
                    print("WARNING: {} sensor timeouts in a row - Roomba not talking.".format(TIMEOUT_WARN))
                    print("  Press CLEAN on the Roomba, then type in Thonny shell:")
                    print("  import main; main.init_roomba()")
                elif timeout_count % 100 == 0:
                    print("Still no sensor data ({} timeouts)...".format(timeout_count))

# ── Entry point ───────────────────────────────────────────────────────────────

connect_wifi()
init_roomba()
run_server()
