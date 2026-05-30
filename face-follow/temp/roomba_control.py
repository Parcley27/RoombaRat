from machine import UART, Pin
import time

BRC_PIN = 4
UART_TX = 25
UART_RX = 26

brc  = Pin(BRC_PIN, Pin.OUT)
uart = UART(1, baudrate=115200, tx=UART_TX, rx=UART_RX)

def _send(data):
    uart.write(bytes(data))

def flush():
    while uart.any():
        uart.read(uart.any())

def oi_mode():
    """Return OI mode byte: 0=off 1=passive 2=safe 3=full, or -1 on timeout."""
    flush()
    _send([149, 1, 35])
    time.sleep_ms(50)
    d = uart.read(1)
    return d[0] if d else -1

def wake():
    brc.value(1)
    time.sleep_ms(100)
    brc.value(0)
    time.sleep_ms(500)
    brc.value(1)
    time.sleep_ms(100)

def start():
    """Send START + FULL, retrying until the Roomba confirms Full mode."""
    for attempt in range(5):
        flush()
        _send([128])
        time.sleep_ms(300)
        _send([132])
        time.sleep_ms(500)
        mode = oi_mode()
        print("OI mode after attempt {}: {}".format(attempt + 1, mode))
        if mode == 3:
            print("Roomba in Full mode.")
            return
        _send([131])
        time.sleep_ms(300)
        _send([132])
        time.sleep_ms(500)
        mode = oi_mode()
        if mode in (2, 3):
            print("Roomba in mode {}.".format(mode))
            return
        time.sleep_ms(500)
    print("WARNING: could not confirm Full mode — commands may be ignored.")

# Motors opcode 138: bits 0=side brush, 1=vacuum, 2=main brush
def set_clean_motors(side_brush=False, vacuum=False, main_brush=False):
    mask = 0
    if side_brush:  mask |= 0b001
    if vacuum:      mask |= 0b010
    if main_brush:  mask |= 0b100
    _send([138, mask])

def drive(velocity, radius=0x8000):
    v = velocity & 0xFFFF
    r = radius   & 0xFFFF
    _send([137, v >> 8, v & 0xFF, r >> 8, r & 0xFF])

def stop_all():
    drive(0)
    set_clean_motors()

def run_for(ms, action):
    action()
    time.sleep_ms(ms)
    stop_all()

# Sound: opcode 140 defines a song, 141 plays it.
# Note duration unit = 1/64 second (64 = 1s). Notes are MIDI numbers (31-127).
SONGS = {
    'beep':  [(72, 16), (72, 16)],                          # two short C5 blips
    'alert': [(80, 8), (76, 8), (80, 8)],                   # ascending chirp
    'happy': [(60, 16), (64, 16), (67, 16), (72, 32)],      # C major arpeggio
}

def define_song(slot, notes):
    payload = [140, slot, len(notes)]
    for note, dur in notes:
        payload += [note, dur]
    _send(payload)

def play_song(slot):
    _send([141, slot])

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

CHARGING_STATES = {
    0: 'Not charging',
    1: 'Reconditioning',
    2: 'Full charging',
    3: 'Trickle charging',
    4: 'Waiting',
    5: 'Charging fault',
}

def monitor_charging():
    print("Monitoring charge state — press Ctrl-C to stop.")
    last = None
    while True:
        flush()
        _send([149, 1, 21])   # packet 21 = Charging State
        time.sleep_ms(50)
        d = uart.read(1)
        if d:
            state = CHARGING_STATES.get(d[0], 'Unknown ({})'.format(d[0]))
            charging = d[0] in (1, 2, 3)
            label = 'CHARGING' if charging else 'NOT CHARGING'
            msg = '[{}]  {}'.format(label, state)
        else:
            msg = '[NO RESPONSE]  Roomba not responding'
        if msg != last:
            print(msg)
            last = msg
        time.sleep_ms(500)


def sound_menu():
    print("  Sounds:")
    names = list(SONGS.keys())
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name}")
    choice = input("  > ").strip()
    try:
        name = names[int(choice) - 1]
    except (ValueError, IndexError):
        print("  Invalid.")
        return
    define_song(0, SONGS[name])
    time.sleep_ms(50)
    play_song(0)
    print(f"  Playing '{name}'...")

def menu():
    print("\n=== Roomba Motor Control ===")
    print("1. Side brush")
    print("2. Vacuum")
    print("3. Wheels")
    print("4. Sound")
    print("5. Drive square (0.5m x 0.5m)")
    print("6. Monitor charging (Ctrl-C to stop)")
    print("q. Quit")
    return input("Select: ").strip()

def wheels_menu():
    print("  Direction: (f)orward / (b)ackward")
    d = input("  > ").strip().lower()
    speed = 200
    if d == 'b':
        speed = -200
    run_for(2000, lambda: drive(speed))

def drive_square():
    SPEED      = 200   # mm/s forward
    SPIN_SPEED = 150   # mm/s for in-place turn
    SIDE_MM    = 500

    # time to travel one side
    drive_ms = int(SIDE_MM / SPEED * 1000)

    # 90-degree spin: arc = 0.25 * pi * wheelbase (235mm on 600 series)
    # ~183mm per wheel at SPIN_SPEED — tune TURN_MS if corners overshoot/undershoot
    TURN_MS = 1220

    print("Driving 0.5m x 0.5m square...")
    for side in range(4):
        drive(SPEED)
        time.sleep_ms(drive_ms)
        drive(0)
        time.sleep_ms(150)
        drive(SPIN_SPEED, 0xFFFF)  # spin clockwise 90 degrees
        time.sleep_ms(TURN_MS)
        drive(0)
        time.sleep_ms(150)
    stop_all()
    print("Square complete.")

# --- init ---
print("Waking Roomba...")
wake()
time.sleep_ms(500)
start()
read_battery()
print("Ready.")

while True:
    choice = menu()

    if choice == '1':
        print("Running side brush for 2s...")
        run_for(2000, lambda: set_clean_motors(side_brush=True))

    elif choice == '2':
        print("Running vacuum + main brush for 2s...")
        run_for(2000, lambda: set_clean_motors(vacuum=True, main_brush=True))

    elif choice == '3':
        wheels_menu()

    elif choice == '4':
        sound_menu()

    elif choice == '5':
        drive_square()

    elif choice == '6':
        try:
            monitor_charging()
        except KeyboardInterrupt:
            print("\nStopped monitoring.")

    elif choice == 'q':
        stop_all()
        print("Stopped. Bye.")
        break

    else:
        print("Invalid choice.")
