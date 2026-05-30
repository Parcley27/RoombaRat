import serial
import RPi.GPIO as GPIO
import time
import sys

BRC_PIN     = 17            # GPIO17, physical pin 11
SERIAL_PORT = '/dev/ttyAMA0'
BAUD        = 115200

GPIO.setmode(GPIO.BCM)
GPIO.setup(BRC_PIN, GPIO.OUT)

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

def _send(data):
    ser.write(bytes(data))

def wake():
    GPIO.output(BRC_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(BRC_PIN, GPIO.LOW)
    time.sleep(0.5)
    GPIO.output(BRC_PIN, GPIO.HIGH)
    time.sleep(0.1)

def start():
    _send([128])
    time.sleep(0.2)
    _send([132])
    time.sleep(0.2)

def set_clean_motors(side_brush=False, vacuum=False):
    mask = 0
    if side_brush: mask |= 0b001
    if vacuum:     mask |= 0b010
    _send([138, mask])

def drive(velocity, radius=0x8000):
    v = velocity & 0xFFFF
    r = radius   & 0xFFFF
    _send([137, v >> 8, v & 0xFF, r >> 8, r & 0xFF])

def stop_all():
    drive(0)
    set_clean_motors()

def run_for(secs, action):
    action()
    time.sleep(secs)
    stop_all()

SONGS = {
    'beep':  [(72, 16), (72, 16)],
    'alert': [(80, 8), (76, 8), (80, 8)],
    'happy': [(60, 16), (64, 16), (67, 16), (72, 32)],
}

def define_song(slot, notes):
    payload = [140, slot, len(notes)]
    for note, dur in notes:
        payload += [note, dur]
    _send(payload)

def play_song(slot):
    _send([141, slot])

def wheels_menu():
    print("  Direction: (f)orward / (b)ackward")
    d = input("  > ").strip().lower()
    speed = 200 if d != 'b' else -200
    run_for(2, lambda: drive(speed))

def sound_menu():
    names = list(SONGS.keys())
    print("  Sounds:")
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name}")
    choice = input("  > ").strip()
    try:
        name = names[int(choice) - 1]
    except (ValueError, IndexError):
        print("  Invalid.")
        return
    define_song(0, SONGS[name])
    time.sleep(0.05)
    play_song(0)
    print(f"  Playing '{name}'...")

def menu():
    print("\n=== Roomba Motor Control ===")
    print("1. Side brush")
    print("2. Vacuum")
    print("3. Wheels")
    print("4. Sound")
    print("q. Quit")
    return input("Select: ").strip()

# --- init ---
try:
    print("Waking Roomba...")
    wake()
    time.sleep(0.5)
    start()
    print("Ready.")

    while True:
        choice = menu()

        if choice == '1':
            print("Running side brush for 2s...")
            run_for(2, lambda: set_clean_motors(side_brush=True))

        elif choice == '2':
            print("Running vacuum for 2s...")
            run_for(2, lambda: set_clean_motors(vacuum=True))

        elif choice == '3':
            wheels_menu()

        elif choice == '4':
            sound_menu()

        elif choice == 'q':
            stop_all()
            print("Stopped. Bye.")
            break

        else:
            print("Invalid choice.")

except KeyboardInterrupt:
    pass

finally:
    stop_all()
    ser.close()
    GPIO.cleanup()
