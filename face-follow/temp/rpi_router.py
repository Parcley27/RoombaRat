import serial
import RPi.GPIO as GPIO
import time
import threading
from flask import Flask, request, jsonify

# --- config ---
BRC_PIN     = 17
SERIAL_PORT = '/dev/ttyAMA0'
BAUD        = 115200
HOST        = '0.0.0.0'
PORT        = 5000

# Roomba OI will auto-stop if no command received within this interval (Safe mode watchdog)
WATCHDOG_INTERVAL = 3.0  # seconds

app = Flask(__name__)
serial_lock = threading.Lock()
last_command_time = time.time()

GPIO.setmode(GPIO.BCM)
GPIO.setup(BRC_PIN, GPIO.OUT)
ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

# --- Roomba low-level ---

def _send(data):
    with serial_lock:
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
    _send([131])  # Safe mode — watchdog auto-stops on serial timeout
    time.sleep(0.2)

def drive(velocity, radius=0x8000):
    v = velocity & 0xFFFF
    r = radius   & 0xFFFF
    _send([137, v >> 8, v & 0xFF, r >> 8, r & 0xFF])

def set_clean_motors(side_brush=False, vacuum=False):
    mask = 0
    if side_brush: mask |= 0b001
    if vacuum:     mask |= 0b010
    _send([138, mask])

def stop_all():
    drive(0)
    set_clean_motors()

# --- watchdog ---
# Roomba Safe mode stops on its own after ~5s with no serial activity,
# but we send an explicit stop sooner to be safe.

def watchdog_loop():
    global last_command_time
    while True:
        time.sleep(0.5)
        if time.time() - last_command_time > WATCHDOG_INTERVAL:
            stop_all()

threading.Thread(target=watchdog_loop, daemon=True).start()

def touch():
    global last_command_time
    last_command_time = time.time()

# --- API routes ---

@app.route('/drive', methods=['POST'])
def api_drive():
    """
    Body: { "velocity": int (-500 to 500 mm/s),
            "radius":   int (-2000 to 2000 mm) }  -- optional, default straight
    """
    data = request.get_json(force=True)
    velocity = int(data.get('velocity', 0))
    radius   = int(data.get('radius', 0x8000))
    velocity = max(-500, min(500, velocity))
    drive(velocity, radius)
    touch()
    return jsonify(ok=True, velocity=velocity, radius=radius)

@app.route('/stop', methods=['POST'])
def api_stop():
    stop_all()
    touch()
    return jsonify(ok=True)

@app.route('/motors', methods=['POST'])
def api_motors():
    """
    Body: { "side_brush": bool, "vacuum": bool }
    """
    data = request.get_json(force=True)
    side_brush = bool(data.get('side_brush', False))
    vacuum     = bool(data.get('vacuum',     False))
    set_clean_motors(side_brush=side_brush, vacuum=vacuum)
    touch()
    return jsonify(ok=True, side_brush=side_brush, vacuum=vacuum)

@app.route('/status', methods=['GET'])
def api_status():
    return jsonify(ok=True, uptime=time.time())

# --- main ---

if __name__ == '__main__':
    try:
        print("Waking Roomba...")
        wake()
        time.sleep(0.5)
        start()
        print(f"Roomba ready. Listening on {HOST}:{PORT}")
        app.run(host=HOST, port=PORT, threaded=True)
    finally:
        stop_all()
        ser.close()
        GPIO.cleanup()
