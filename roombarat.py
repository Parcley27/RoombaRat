import cv2
import os
import socket
import sys
import time
from datetime import datetime
from dotenv import load_dotenv

load_dotenv(override=True)

from RekognitionController import check_for_phone
from BoxController import upload_to_box
from EmailController import send_alert_email

# setup
ESP32_IP = os.getenv('ESP32_IP', '192.168.4.1')
CMD_PORT = 9000
CMD_INTERVAL = 0.10

CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
CAMERA_ROTATE = int(os.getenv('CAMERA_ROTATE', '0'))

CONFIRM_FRAMES = 2
FACE_TIMEOUT = 0.5

TARGET_AREA_MIN = 0.03
AT_FACE_AREA = 0.06
SPIN_THRESHOLD = 0.40

FORWARD_SPEED = 180
SEARCH_SPEED = 60
SPIN_RIGHT_SPD = 80

ALERT_COOLDOWN = 30.0
SPIN_AWAY_SECS = 2.5
INSPECT_FRAMES = 3

STRAIGHT = -32768
CW_SPIN = -1
CCW_SPIN = 1

# inspector states
SEARCHING  = 'SEARCHING'
FOLLOWING  = 'FOLLOWING'
INSPECTING = 'INSPECTING'

face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

def detect_largest_face(frame):
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=9, minSize=(60, 60)
    )
    if len(faces) == 0:
        return None
    return tuple(max(faces, key=lambda b: b[2] * b[3]))


def compute_drive(horiz_err, area_ratio):
    """returns velocity and radius for Roomba OI DRIVE opcode."""
    if abs(horiz_err) >= SPIN_THRESHOLD:
        return SEARCH_SPEED, (CW_SPIN if horiz_err > 0 else CCW_SPIN)

    t = min(abs(horiz_err) / SPIN_THRESHOLD, 1.0)
    r_mag = int(800 * (1 - t) + 200 * t)
    radius = -r_mag if horiz_err > 0 else r_mag

    speed = FORWARD_SPEED if area_ratio < TARGET_AREA_MIN else 100
    return speed, radius


def rotate_frame(frame, deg):
    if deg == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if deg == -90:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if deg == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def main():
    # connect to esp32
    sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp_addr = (ESP32_IP, CMD_PORT)

    sock.sendto(b'C\n', esp_addr)
    print(f"Sent connect to ESP32 at {ESP32_IP}:{CMD_PORT} — listen for Roomba beep.")
    time.sleep(0.5)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Could not open camera {CAMERA_INDEX}. Try setting CAMERA_INDEX in .env")
        sys.exit(1)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_area = frame_w * frame_h

    state = SEARCHING
    face_streak = 0
    last_face_time = 0.0
    last_box = None
    last_drive = None
    last_send_t = 0.0
    last_alert_t = 0.0
    inspect_frames = 0
    spin_until = 0.0

    def send_drive(v, r):
        nonlocal last_drive, last_send_t
        now = time.time()
        if (v, r) != last_drive or now - last_send_t >= CMD_INTERVAL:
            try:
                sock.sendto(f"{v} {r}\n".encode(), esp_addr)
                last_drive = (v, r)
                last_send_t = now
            except OSError as e:
                print(f"Network error: {e}")

    print("RoombaRat patrol started. Press 'q' to quit.")
    print(f"State: {state}")

    try:
        while True:
            ret, raw_frame = cap.read()
            if not ret:
                print("Warning: dropped frame")
                continue

            frame = rotate_frame(raw_frame, CAMERA_ROTATE)
            now   = time.time()

            # facial rec
            box = detect_largest_face(frame)

            if box is not None:
                face_streak = min(face_streak + 1, CONFIRM_FRAMES)
            else:
                face_streak = 0

            if box is not None and face_streak >= CONFIRM_FRAMES:
                last_face_time = now
                last_box       = box

            face_active = (now - last_face_time) < FACE_TIMEOUT

            if state == SEARCHING:
                send_drive(SEARCH_SPEED, CCW_SPIN)
                label  = "SEARCHING — spinning CCW"
                colour = (0, 165, 255)

                if face_active:
                    state = FOLLOWING
                    print(f"Face confirmed → FOLLOWING")

            elif state == FOLLOWING:
                if not face_active:
                    state = SEARCHING
                    last_box = None
                    print("Face lost → SEARCHING")
                    continue

                x, y, w, h = last_box
                cx         = x + w // 2
                area_ratio = (w * h) / frame_area
                horiz_err  = (cx - frame_w / 2) / (frame_w / 2)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, y + h // 2), 5, (0, 255, 0), -1)

                if area_ratio >= AT_FACE_AREA:
                    # face is close enoguh for phone rec
                    send_drive(0, STRAIGHT)
                    state          = INSPECTING
                    inspect_frames = 0
                    print(f"At face (area={area_ratio:.3f}) → INSPECTING")
                    label  = f"AT FACE — stopping"
                    colour = (0, 255, 255)
                else:
                    v, r = compute_drive(horiz_err, area_ratio)
                    send_drive(v, r)
                    label  = f"FOLLOWING  area={area_ratio:.3f}  err={horiz_err:+.2f}"
                    colour = (0, 255, 0)

            elif state == INSPECTING:
                send_drive(0, STRAIGHT)

                inspect_frames += 1
                label  = f"INSPECTING ({inspect_frames}/{INSPECT_FRAMES})"
                colour = (0, 255, 255)

                if inspect_frames >= INSPECT_FRAMES:

                    ret2, snap = cap.read()
                    if ret2:
                        snap = rotate_frame(snap, CAMERA_ROTATE)
                    else:
                        snap = frame

                    print("Calling Rekognition...")
                    try:
                        phone_found = check_for_phone(snap)
                    except Exception as e:
                        print(f"Rekognition error: {e}")
                        phone_found = False

                    if phone_found and (now - last_alert_t) >= ALERT_COOLDOWN:
                        last_alert_t = now
                        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                        filename = f"phone_caught_{timestamp}.jpg"
                        _, buf = cv2.imencode('.jpg', snap)

                        print(f"PHONE DETECTED — uploading to Box...")
                        try:
                            box_url = upload_to_box(buf.tobytes(), filename)
                            send_alert_email(filename, box_url)
                            print(f"Alert sent! Box: {box_url}")
                        except Exception as e:
                            print(f"Alert failed: {e}")

                        label  = "PHONE CAUGHT!"
                        colour = (0, 0, 255)
                        cv2.putText(frame, label, (10, 28),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
                        cv2.imshow("RoombaRat", frame)
                        cv2.waitKey(1000)

                    elif phone_found:
                        print("Phone found but still in cooldown — skipping alert.")
                    else:
                        print("No phone — moving to next person.")

                    spin_until = time.time() + SPIN_AWAY_SECS
                    state = SEARCHING
                    last_box = None
                    face_streak = 0

                    while time.time() < spin_until:
                        send_drive(SPIN_RIGHT_SPD, CW_SPIN)
                        ret_s, fr_s = cap.read()
                        if ret_s:
                            fr_s = rotate_frame(fr_s, CAMERA_ROTATE)
                            cv2.putText(fr_s, "Moving to next person...",
                                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.72, (0, 165, 255), 2)
                            cv2.imshow("RoombaRat", fr_s)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            raise KeyboardInterrupt
                    print("Spin-away done → SEARCHING")
                    continue

            cv2.putText(frame, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, colour, 2)
            v_log, r_log = last_drive if last_drive else (0, 0)
            cv2.putText(frame, f"v={v_log} r={r_log}  state={state}",
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            cv2.imshow("RoombaRat", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.sendto(b'0 -32768\n', esp_addr)
        except OSError:
            pass
        sock.close()
        cap.release()
        cv2.destroyAllWindows()
        print("RoombaRat stopped.")


if __name__ == '__main__':
    main()
