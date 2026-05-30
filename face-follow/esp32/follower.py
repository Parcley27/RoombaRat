import select
import sys
import time
from machine import UART, Pin

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
    time.sleep_ms(300)
    _send([132])
    time.sleep_ms(300)

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
ALERT_SONG   = [(84, 6), (84, 6), (84, 12)]

print("Waking Roomba (make sure CLEAN was pressed to power it on)...")
wake()
time.sleep_ms(2000)
start()
beep(0, BOOT_SONG)
print("Ready. Waiting for commands over USB serial.")

WATCHDOG_MS = 5000

rx_buf       = b""
last_cmd_ms  = time.ticks_ms()
watchdog_stopped = False

while True:
    now_ms = time.ticks_ms()

    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        chunk = sys.stdin.buffer.read(64)
        if chunk:
            rx_buf += chunk
            last_cmd_ms      = now_ms
            watchdog_stopped = False
            while b"\n" in rx_buf:
                line, rx_buf = rx_buf.split(b"\n", 1)
                line = line.strip().decode("ascii", "ignore")
                if line == "C":
                    start()
                    time.sleep_ms(100)
                    beep(1, CONNECT_SONG)
                    print("Mac connected.")
                elif line == "B":
                    beep(2, ALERT_SONG)
                    print("ALERT beep")
                else:
                    parts = line.split()
                    if len(parts) == 2:
                        try:
                            v, r_val = int(parts[0]), int(parts[1])
                            drive(v, r_val)
                        except ValueError:
                            pass

    if not watchdog_stopped and time.ticks_diff(now_ms, last_cmd_ms) > WATCHDOG_MS:
        start()        # re-enter Full mode in case OI dropped to Passive
        drive(0)       # then send stop
        watchdog_stopped = True
        print("Watchdog: Mac silent, Roomba stopped.")

    time.sleep_ms(5)
