import select
import sys
import time
from machine import UART, Pin

# Communicates with the Mac over USB serial (sys.stdin / sys.stdout).
# No WiFi needed — plug the USB-C cable in and run the Mac script.
# UART1 (GPIO25/26) is still dedicated to the Roomba OI.

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

# --- boot ---
print("Waking Roomba...")
wake()
time.sleep_ms(1000)
start()
time.sleep_ms(500)
beep(0, BOOT_SONG)
print("Ready. Waiting for commands over USB serial.")

rx_buf = b""

while True:
    # Non-blocking read from USB serial (Mac → ESP32)
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        chunk = sys.stdin.buffer.read(64)
        if chunk:
            rx_buf += chunk
            while b"\n" in rx_buf:
                line, rx_buf = rx_buf.split(b"\n", 1)
                line = line.strip().decode("ascii", "ignore")
                if line == "C":
                    start()
                    time.sleep_ms(100)
                    beep(1, CONNECT_SONG)
                    print("Mac connected.")
                else:
                    parts = line.split()
                    if len(parts) == 2:
                        try:
                            v, r_val = int(parts[0]), int(parts[1])
                            drive(v, r_val)
                        except ValueError:
                            pass

    time.sleep_ms(5)
