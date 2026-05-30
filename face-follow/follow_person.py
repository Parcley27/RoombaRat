import cv2
import socket
import time
import sys
import argparse

# --- tuning ---
CAMERA_INDEX = 0
FACE_TIMEOUT = 0.5    # seconds to coast on last known position before searching
CMD_INTERVAL = 0.10   # min seconds between sending repeated identical commands

TARGET_AREA_MIN = 0.03   # drive forward below this (face too small = too far)
TARGET_AREA_MAX = 0.07   # back up above this (face too large = too close)
TURN_THRESHOLD = 0.08   # horizontal dead zone — no correction below this
SPIN_THRESHOLD = 0.40   # in-place spin above this, curved drive below

FORWARD_SPEED = 180    # mm/s base forward speed
SEARCH_SPEED = 60     # mm/s spin speed when no face detected
BACKUP_SPEED = -80    # mm/s when too close

STRAIGHT = -32768
CW_SPIN = -1
CCW_SPIN = 1

CMD_PORT = 9000

face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)


def discover_esp32():
    # esp32 ip
    return '192.168.4.1'


def detect_largest_face(frame):
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
    )
    if len(faces) == 0:
        return None
    return tuple(max(faces, key=lambda b: b[2] * b[3]))


def compute_drive(horiz_err, area_ratio):
    if area_ratio > TARGET_AREA_MAX:
        return BACKUP_SPEED, STRAIGHT

    too_far  = area_ratio < TARGET_AREA_MIN
    centered = abs(horiz_err) < TURN_THRESHOLD
    big_err  = abs(horiz_err) >= SPIN_THRESHOLD

    if centered:
        return (FORWARD_SPEED if too_far else 0), STRAIGHT

    if big_err:
        return 70, (CW_SPIN if horiz_err > 0 else CCW_SPIN)

    t      = (abs(horiz_err) - TURN_THRESHOLD) / (SPIN_THRESHOLD - TURN_THRESHOLD)
    r_mag  = int(800 * (1 - t) + 200 * t)
    radius = -r_mag if horiz_err > 0 else r_mag
    speed  = int(FORWARD_SPEED * (1 - t * 0.5)) if too_far else int(60 * (1 - t * 0.3))
    return speed, radius


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',   default=None, help='ESP32 IP (skip auto-discovery)')
    parser.add_argument('--camera', type=int, default=CAMERA_INDEX)
    args = parser.parse_args()

    esp32_ip = args.host or discover_esp32()
    if esp32_ip is None:
        print("Roomba not found. Make sure the ESP32 is on the same WiFi network.")
        print("Or specify its IP with:  --host <ip>")
        sys.exit(1)
    print(f"Roomba found at {esp32_ip}")

    cmd_sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp32_addr = (esp32_ip, CMD_PORT)

    cmd_sock.sendto(b'C\n', esp32_addr)
    print("Connected — listen for Roomba connect sound.")
    time.sleep(0.5)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("Could not open webcam.")
        sys.exit(1)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_area = frame_w * frame_h

    last_drive = None
    last_send_time = 0.0
    last_status = ''
    last_face_time = 0.0
    last_box = None

    def send_drive(v, r):
        nonlocal last_drive, last_send_time
        now = time.time()
        if (v, r) != last_drive or now - last_send_time >= CMD_INTERVAL:
            try:
                cmd_sock.sendto(f"{v} {r}\n".encode(), esp32_addr)
                last_drive     = (v, r)
                last_send_time = now
            except OSError as e:
                print(f"Network error ({e}) — waiting for ESP32 to reconnect...")

    # Startup test — confirms drive commands reach the Roomba
    print("Startup test: forward 2s...")
    cmd_sock.sendto(b'200 -32768\n', esp32_addr)
    time.sleep(2)
    cmd_sock.sendto(b'0 -32768\n', esp32_addr)
    print("Test done. Starting face tracking...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            now = time.time()
            box = detect_largest_face(frame)

            if box is not None:
                last_face_time = now
                last_box = box

            face_active = (now - last_face_time) < FACE_TIMEOUT

            if not face_active:
                send_drive(SEARCH_SPEED, CCW_SPIN)
                last_box = None
                label = "Searching..."
                colour = (0, 165, 255)
            else:
                x, y, w, h = last_box
                cx = x + w // 2
                area_ratio = (w * h) / frame_area
                horiz_err = (cx - frame_w / 2) / (frame_w / 2)

                v, r = compute_drive(horiz_err, area_ratio)
                send_drive(v, r)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y + h // 2), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"area={area_ratio:.3f}  err={horiz_err:+.2f}  v={v} r={r}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                if area_ratio > TARGET_AREA_MAX:
                    label = f"BACK UP  area={area_ratio:.3f}"
                elif v == 0 and r == STRAIGHT:
                    label = f"HOLD  err={horiz_err:+.2f}"
                elif r in (CW_SPIN, CCW_SPIN) and v < 100:
                    label = f"SPIN {'RIGHT' if r == CW_SPIN else 'LEFT'}  err={horiz_err:+.2f}"
                elif r == STRAIGHT:
                    label = f"FORWARD  area={area_ratio:.3f}"
                else:
                    label = f"CURVE {'RIGHT' if r < 0 else 'LEFT'}  err={horiz_err:+.2f}"

                colour = (0, 255, 0) if box is not None else (0, 165, 255)

            v_log, r_log = last_drive if last_drive else (0, 0)
            status = f"{label}  [v={v_log} r={r_log}]"
            if status != last_status:
                print(status)
                last_status = status

            cv2.putText(frame, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, colour, 2)
            cv2.imshow("Roomba Face Follower", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        try:
            cmd_sock.sendto(b'0 0\n', esp32_addr)
        except OSError:
            pass
        cmd_sock.close()
        cap.release()
        cv2.destroyAllWindows()
        print("Stopped.")


if __name__ == '__main__':
    main()
