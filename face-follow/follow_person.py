import cv2
import os
import sys
import time
import argparse
import serial
import serial.tools.list_ports
from dotenv import load_dotenv

load_dotenv(override=True)

# --- tuning ---
CAMERA_INDEX = 0
FACE_TIMEOUT = 0.5
CMD_INTERVAL = 0.10

TARGET_AREA_MIN = 0.03
TARGET_AREA_MAX = 0.07
SPIN_THRESHOLD  = 0.40

FORWARD_SPEED = 180
SEARCH_SPEED  = 60
BACKUP_SPEED  = -80

STRAIGHT = -32768
CW_SPIN  = -1
CCW_SPIN =  1

CONFIRM_FRAMES = 2

SERIAL_PORT = os.getenv('SERIAL_PORT', '')   # e.g. /dev/cu.usbserial-0001
BAUD_RATE   = 115200

face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)


def find_serial_port():
    """Return first USB-serial port that looks like an ESP32."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        if any(k in desc for k in ('cp210', 'ch340', 'uart', 'esp32', 'usb serial')):
            return p.device
    return None


def detect_largest_face(frame):
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=9, minSize=(60, 60)
    )
    if len(faces) == 0:
        return None
    return tuple(max(faces, key=lambda b: b[2] * b[3]))


def compute_drive(horiz_err, area_ratio):
    if area_ratio > TARGET_AREA_MAX:
        return BACKUP_SPEED, STRAIGHT
    if abs(horiz_err) >= SPIN_THRESHOLD:
        return 100, (CW_SPIN if horiz_err > 0 else CCW_SPIN)
    t      = min(abs(horiz_err) / SPIN_THRESHOLD, 1.0)
    r_mag  = int(800 * (1 - t) + 200 * t)
    radius = -r_mag if horiz_err > 0 else r_mag
    speed  = FORWARD_SPEED if area_ratio < TARGET_AREA_MIN else 100
    return speed, radius


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',   default=None, help='Serial port (overrides SERIAL_PORT in .env)')
    parser.add_argument('--camera', type=int, default=CAMERA_INDEX)
    args = parser.parse_args()

    port = args.port or SERIAL_PORT or find_serial_port()
    if not port:
        print("Could not find ESP32 serial port.")
        print("Plug in the USB-C cable, then either:")
        print("  set SERIAL_PORT=/dev/cu.usbserial-XXXX in .env")
        print("  or pass --port /dev/cu.usbserial-XXXX")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        sys.exit(1)

    print(f"Opening serial port {port}...")
    ser = serial.Serial(port, BAUD_RATE, timeout=0)
    ser.write(b'C\n')
    print("Sent connect — listen for Roomba beep.")
    time.sleep(0.5)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("Could not open webcam.")
        sys.exit(1)

    frame_w    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h    = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_area = frame_w * frame_h

    last_drive     = None
    last_send_time = 0.0
    last_status    = ''
    last_face_time = 0.0
    last_box       = None
    face_streak    = 0

    def send_drive(v, r):
        nonlocal last_drive, last_send_time
        now = time.time()
        if (v, r) != last_drive or now - last_send_time >= CMD_INTERVAL:
            try:
                ser.write(f"{v} {r}\n".encode())
                last_drive     = (v, r)
                last_send_time = now
            except serial.SerialException as e:
                print(f"Serial error: {e}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            now = time.time()
            box = detect_largest_face(frame)

            if box is not None:
                face_streak = min(face_streak + 1, CONFIRM_FRAMES)
            else:
                face_streak = 0

            if box is not None and face_streak >= CONFIRM_FRAMES:
                last_face_time = now
                last_box       = box

            face_active = (now - last_face_time) < FACE_TIMEOUT

            if not face_active:
                send_drive(SEARCH_SPEED, CCW_SPIN)
                last_box = None
                label  = "Searching..."
                colour = (0, 165, 255)
            else:
                x, y, w, h = last_box
                cx         = x + w // 2
                area_ratio = (w * h) / frame_area
                horiz_err  = (cx - frame_w / 2) / (frame_w / 2)

                v, r = compute_drive(horiz_err, area_ratio)
                send_drive(v, r)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y + h // 2), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"area={area_ratio:.3f}  err={horiz_err:+.2f}  v={v} r={r}",
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                if area_ratio > TARGET_AREA_MAX:
                    label = f"BACK UP  area={area_ratio:.3f}"
                elif r in (CW_SPIN, CCW_SPIN):
                    label = f"SPIN {'RIGHT' if r == CW_SPIN else 'LEFT'}  err={horiz_err:+.2f}"
                elif r == STRAIGHT:
                    label = f"FORWARD  area={area_ratio:.3f}"
                else:
                    label = f"CURVE {'RIGHT' if r < 0 else 'LEFT'}  err={horiz_err:+.2f}"

                colour = (0, 255, 0)

            v_log, r_log = last_drive if last_drive else (0, 0)
            status = f"{label}  [v={v_log} r={r_log}]"
            if status != last_status:
                print(status)
                last_status = status

            cv2.putText(frame, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, colour, 2)
            cv2.imshow("Roomba Face Follower", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        try:
            ser.write(b'0 -32768\n')
        except serial.SerialException:
            pass
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == '__main__':
    main()
