import network
import socket
import time
from machine import UART, Pin

AP_SSID     = 'Roomba'
AP_PASSWORD = 'roomba123'
CMD_PORT    = 9000
WATCHDOG_MS = 1000   # stop Roomba if no command received for this long

BRC_PIN = 4
UART_TX = 25
UART_RX = 26

brc  = Pin(BRC_PIN, Pin.OUT)
uart = UART(1, baudrate=115200, tx=UART_TX, rx=UART_RX)

def _send(data):
    uart.write(bytes(data))

def wake():
    brc.value(1)
    time.sleep_ms(100)
    brc.value(0)
    time.sleep_ms(500)
    brc.value(1)
    time.sleep_ms(100)

def start():
    _send([128])
    time.sleep_ms(200)
    _send([132])
    time.sleep_ms(200)

def drive(velocity, radius=-32768):
    v = velocity & 0xFFFF
    r = radius   & 0xFFFF
    _send([137, v >> 8, v & 0xFF, r >> 8, r & 0xFF])

def define_song(slot, notes):
    payload = [140, slot, len(notes)]
    for note, dur in notes:
        payload += [note, dur]
    _send(payload)

def play_song(slot):
    _send([141, slot])

def beep(slot, notes):
    define_song(slot, notes)
    time.sleep_ms(100)
    play_song(slot)

BOOT_SONG    = [(60, 16), (67, 24)]
CONNECT_SONG = [(72, 8), (76, 8), (79, 12)]

def start_ap():
    # Disable station mode to avoid interference
    sta = network.WLAN(network.STA_IF)
    sta.active(False)

    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(ssid=AP_SSID, password=AP_PASSWORD,
              authmode=network.AUTH_WPA_WPA2_PSK)
    # ESP32 AP is always reachable at 192.168.4.1
    print("AP started — connect Mac to WiFi:", AP_SSID)
    print("ESP32 IP: 192.168.4.1")
    return ap

# --- boot ---
ap = start_ap()

print("Waking Roomba...")
wake()
time.sleep_ms(1000)
start()
time.sleep_ms(500)
beep(0, BOOT_SONG)
print("Ready.")

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.bind(('0.0.0.0', CMD_PORT))
udp.setblocking(False)

last_cmd_ms = time.ticks_ms()
stopped     = False

while True:
    now_ms = time.ticks_ms()

    # Receive drive command
    try:
        data, _ = udp.recvfrom(64)
        line = data.decode().strip()
        last_cmd_ms = now_ms
        stopped = False
        if line == 'C':
            beep(1, CONNECT_SONG)
        else:
            parts = line.split()
            if len(parts) == 2:
                drive(int(parts[0]), int(parts[1]))
    except OSError:
        pass

    # Watchdog — stop if Mac goes silent for >1 s
    if not stopped and time.ticks_diff(now_ms, last_cmd_ms) > WATCHDOG_MS:
        drive(0)
        stopped = True
        print("Watchdog: no command, stopped.")

    time.sleep_ms(5)
