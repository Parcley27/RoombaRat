"""
RoombaRat patrol — merges the two original programs into one behaviour:

  SEARCH    spin until a face is found
  APPROACH  follow that face until it fills ~5% of the frame (or stops moving);
            while a face is in view, periodically ask Rekognition if a phone is
            present — if so, beep the Roomba and fire the Box upload + email alert
  ARRIVED   reached / face went still -> brief stop
  TURN_AWAY spin in the OPPOSITE direction for a moment to leave that face, then
            drop back to SEARCH and lock onto whoever shows up next

Driving reuses the face-follow path (UDP -> ESP32 -> Roomba). Phone detection /
Box / email reuse main.py's pipeline. Run with --dry-run (no robot) and
--mock-phone (press 'p' to fake a phone) to exercise the whole state machine on a
bare laptop with just a webcam.
"""

import cv2
import socket
import time
import sys
import argparse
from collections import deque
from datetime import datetime

# --- face-follow tuning (from follow_person.py) ---
CAMERA_INDEX   = 0
FACE_TIMEOUT   = 0.5    # seconds to coast on last known position before searching
CMD_INTERVAL   = 0.10   # min seconds between sending repeated identical commands

TARGET_AREA_MIN = 0.03  # drive forward below this (face too small = too far)
TARGET_AREA_MAX = 0.07  # back up above this (face too large = too close)
SPIN_THRESHOLD  = 0.40  # in-place spin above this, curved drive below

FORWARD_SPEED = 180     # mm/s base forward speed
SEARCH_SPEED  = 60      # mm/s spin speed when searching
BACKUP_SPEED  = -80     # mm/s when too close

STRAIGHT = -32768
CW_SPIN  = -1
CCW_SPIN = 1

CMD_PORT       = 9000
CONFIRM_FRAMES = 2      # consecutive face frames before we trust it

# --- patrol behaviour ---
ARRIVE_AREA    = 0.05   # face this fraction of frame => "reached" this person
STILL_TIME     = 1.5    # face must hold still this long to count as "done"
STILL_PX       = 25     # max centre wander (px) over STILL_TIME to count as still
ARRIVE_PAUSE   = 0.6    # seconds to sit still on arrival
TURN_AWAY_TIME = 1.3    # seconds to spin away before re-acquiring a new face

# --- phone detection cadence (from main.py) ---
CHECK_INTERVAL = 2.0    # seconds between Rekognition calls (only while a face is up)
COOLDOWN       = 30.0   # seconds between phone alerts

# Alert pipeline (Box upload + SES email) is optional — without it (or with
# --no-phone) this runs as a pure face patroller. Imported lazily so --dry-run
# works with no AWS/Box creds.
ALERT_AVAILABLE = True
_alert_import_error = None
try:
    from dotenv import load_dotenv
    load_dotenv(override=True)
    from BoxController import upload_to_box
    from EmailController import send_alert_email
except Exception as e:           # boto3 / boxsdk / creds missing, etc.
    ALERT_AVAILABLE = False
    _alert_import_error = e

# Phone detector backends. 'rekognition' = AWS cloud (paid, ~2s); 'yolo' = local
# (free, every-frame). Loaded on demand so you only pay the import cost of the
# one you use. Each returns check_for_phone(frame) -> bool.
DETECTOR_INTERVAL = {'rekognition': CHECK_INTERVAL, 'yolo': 0.10}


def load_detector(name):
    if name == 'rekognition':
        from RekognitionController import check_for_phone
        return check_for_phone
    if name == 'yolo':
        from YoloController import check_for_phone
        return check_for_phone
    raise ValueError(f"unknown detector: {name}")

face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)


class State:
    SEARCH    = 'SEARCH'
    APPROACH  = 'APPROACH'
    ARRIVED   = 'ARRIVED'
    TURN_AWAY = 'TURN_AWAY'


def detect_largest_face(frame):
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60)
    )
    if len(faces) == 0:
        return None
    return tuple(max(faces, key=lambda b: b[2] * b[3]))


def compute_drive(horiz_err, area_ratio):
    """Same proportional follower as follow_person.py."""
    if area_ratio > TARGET_AREA_MAX:
        return BACKUP_SPEED, STRAIGHT

    if abs(horiz_err) >= SPIN_THRESHOLD:
        return 100, (CW_SPIN if horiz_err > 0 else CCW_SPIN)

    t      = min(abs(horiz_err) / SPIN_THRESHOLD, 1.0)
    r_mag  = int(800 * (1 - t) + 200 * t)        # 800mm gentle -> 200mm sharp
    radius = -r_mag if horiz_err > 0 else r_mag

    speed = FORWARD_SPEED if area_ratio < TARGET_AREA_MIN else 100
    return speed, radius


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',   default=None, help='ESP32 IP (default 192.168.4.1)')
    parser.add_argument('--camera', type=int, default=CAMERA_INDEX)
    parser.add_argument('--dry-run', action='store_true',
                        help='do not send to the robot — print commands instead')
    parser.add_argument('--no-phone', action='store_true',
                        help='disable phone detection / Box / email entirely')
    parser.add_argument('--mock-phone', action='store_true',
                        help="press 'p' to simulate a phone detection (no detector needed)")
    parser.add_argument('--detector', choices=['rekognition', 'yolo'],
                        default='yolo',
                        help="phone backend: yolo=local/fast (default), "
                             "rekognition=AWS cloud. Toggle live with 'd'.")
    args = parser.parse_args()

    # Resolve the phone detector backend (unless mocking or disabled).
    detector_name = args.detector
    check_fn      = None
    if not args.no_phone and not args.mock_phone:
        try:
            check_fn = load_detector(detector_name)
        except Exception as e:
            print(f"Could not load '{detector_name}' detector: {e}")

    phone_enabled = args.mock_phone or (check_fn is not None and not args.no_phone)
    check_interval = DETECTOR_INTERVAL.get(detector_name, CHECK_INTERVAL)
    if phone_enabled and not args.mock_phone:
        print(f"Phone detector: {detector_name} (every {check_interval:.2f}s)")

    esp32_ip   = args.host or '192.168.4.1'
    cmd_sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp32_addr = (esp32_ip, CMD_PORT)

    if args.dry_run:
        print("[dry-run] robot commands will be printed, not sent.")
    else:
        print(f"Roomba at {esp32_ip} — sending connect.")
        cmd_sock.sendto(b'C\n', esp32_addr)
        time.sleep(0.5)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Could not open camera {args.camera}.")
        sys.exit(1)

    frame_w    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h    = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_area = frame_w * frame_h

    last_drive     = None
    last_send_time = 0.0

    def send(payload, label):
        nonlocal last_drive, last_send_time
        now = time.time()
        if payload != last_drive or now - last_send_time >= CMD_INTERVAL:
            last_drive, last_send_time = payload, now
            if args.dry_run:
                return
            try:
                cmd_sock.sendto(payload, esp32_addr)
            except OSError as e:
                print(f"Network error ({e}) — waiting for ESP32...")

    def send_drive(v, r):
        send(f"{v} {r}\n".encode(), 'drive')

    def send_beep():
        # 'B' is handled by the patched ESP32 firmware (follower.py).
        if args.dry_run:
            print("[dry-run] BEEP")
            return
        try:
            cmd_sock.sendto(b'B\n', esp32_addr)
        except OSError:
            pass

    def trigger_alert(frame):
        send_beep()
        stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"phone_caught_{stamp}.jpg"
        if not (ALERT_AVAILABLE and not args.no_phone):
            print(f"[mock] phone caught — would beep + upload {filename} + email")
            return
        try:
            _, buf  = cv2.imencode('.jpg', frame)
            box_url = upload_to_box(buf.tobytes(), filename)
            send_alert_email(filename, box_url)
            print(f"PHONE CAUGHT — alert sent. Box: {box_url}")
        except Exception as e:
            print(f"Alert failed: {e}")

    # --- state ---
    state          = State.SEARCH
    search_dir     = CCW_SPIN
    last_face_time = 0.0
    last_box       = None
    face_streak    = 0
    centres        = deque()        # (t, cx, cy) history while approaching
    arrived_time   = 0.0
    turn_until     = 0.0
    last_check        = 0.0
    last_alert        = 0.0
    phone_override    = False       # set by 'p' in --mock-phone mode
    phone_alert_until = 0.0         # show the "DETECTED" banner until this time
    last_status       = ''

    def is_still():
        """True if the face centre barely moved over the last STILL_TIME."""
        if len(centres) < 2 or (centres[-1][0] - centres[0][0]) < STILL_TIME:
            return False
        xs = [c[1] for c in centres]
        ys = [c[2] for c in centres]
        return (max(xs) - min(xs) < STILL_PX) and (max(ys) - min(ys) < STILL_PX)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            now = time.time()

            box = detect_largest_face(frame)
            if box is not None:
                face_streak = min(face_streak + 1, CONFIRM_FRAMES)
            else:
                face_streak = 0
            if box is not None and face_streak >= CONFIRM_FRAMES:
                last_face_time, last_box = now, box
            face_active = (now - last_face_time) < FACE_TIMEOUT

            # always show the raw detection + its size, in every state, so you can
            # see exactly when the cascade drops the face (e.g. too close/cropped)
            if box is not None:
                bx, by, bw, bh = box
                bar  = (bw * bh) / frame_area
                bcol = (0, 255, 0) if bar >= ARRIVE_AREA else (0, 255, 255)
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), bcol, 2)
                cv2.putText(frame, f"face {bar * 100:.1f}%", (bx, max(by - 8, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bcol, 1)

            label = state

            # ---------------- state machine ----------------
            if state == State.SEARCH:
                send_drive(SEARCH_SPEED, search_dir)
                if face_active:
                    state   = State.APPROACH
                    centres = deque()
                    label   = "FACE FOUND"

            elif state == State.APPROACH:
                if not face_active:
                    state = State.SEARCH
                    label = "lost face -> search"
                else:
                    x, y, w, h = last_box
                    cx         = x + w // 2
                    area_ratio = (w * h) / frame_area
                    horiz_err  = (cx - frame_w / 2) / (frame_w / 2)

                    v, r = compute_drive(horiz_err, area_ratio)
                    send_drive(v, r)

                    centres.append((now, cx, y + h // 2))
                    while centres and now - centres[0][0] > STILL_TIME:
                        centres.popleft()

                    label = f"APPROACH area={area_ratio:.3f} err={horiz_err:+.2f}"

                    # phone check — only while a face is up, throttled + cooled down
                    if phone_enabled and now - last_check >= check_interval:
                        last_check = now
                        try:
                            if args.mock_phone:
                                hit, phone_override = phone_override, False
                            else:
                                hit = check_fn(frame)
                        except Exception as e:
                            hit = False
                            print(f"{detector_name} error: {e}")
                        if hit:
                            phone_alert_until = now + 3.0   # flag it on screen
                            if now - last_alert >= COOLDOWN:
                                last_alert = now
                                trigger_alert(frame)        # beep + Box + email

                    if area_ratio >= ARRIVE_AREA or is_still():
                        state        = State.ARRIVED
                        arrived_time = now
                        label        = "ARRIVED"

            elif state == State.ARRIVED:
                send_drive(0, STRAIGHT)
                if now - arrived_time >= ARRIVE_PAUSE:
                    search_dir = -search_dir          # leave in a new direction
                    turn_until = now + TURN_AWAY_TIME
                    state      = State.TURN_AWAY

            elif state == State.TURN_AWAY:
                send_drive(SEARCH_SPEED, search_dir)
                label = "TURN AWAY"
                if now >= turn_until:
                    last_face_time = 0.0              # don't re-lock the one we left
                    state          = State.SEARCH

            # ---------------- HUD / logging ----------------
            colour = (0, 255, 0) if state == State.APPROACH else (0, 165, 255)
            cv2.putText(frame, f"{state}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, colour, 2)
            cv2.putText(frame, label, (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # phone status — always-visible top-right indicator
            if phone_enabled:
                fw = frame.shape[1]
                if now < phone_alert_until:
                    ptxt, pcol = "PHONE: DETECTED", (0, 0, 255)
                elif args.mock_phone and phone_override:
                    ptxt, pcol = "PHONE: queued", (0, 165, 255)
                else:
                    ptxt, pcol = "PHONE: clear", (0, 200, 0)
                (tw, _), _ = cv2.getTextSize(ptxt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.putText(frame, ptxt, (fw - tw - 10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, pcol, 2)
                # big banner while a phone is flagged
                if now < phone_alert_until:
                    cv2.putText(frame, "PHONE DETECTED!", (10, 95),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                src = "mock ('p')" if args.mock_phone else detector_name
                cv2.putText(frame, f"detector: {src}  ('d' to switch)",
                            (10, frame.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.imshow("RoombaRat Patrol", frame)

            if label != last_status:
                print(label)
                last_status = label

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('p') and args.mock_phone:
                phone_override = True
                print("[mock] phone queued for next check")
            if key == ord('d') and not args.mock_phone and not args.no_phone:
                # live-switch detector backend (lazy-load the other one)
                new_name = 'yolo' if detector_name == 'rekognition' else 'rekognition'
                try:
                    check_fn       = load_detector(new_name)
                    detector_name  = new_name
                    check_interval = DETECTOR_INTERVAL.get(new_name, CHECK_INTERVAL)
                    phone_enabled  = not args.no_phone
                    last_check     = 0.0
                    print(f"Switched detector -> {detector_name} "
                          f"(every {check_interval:.2f}s)")
                except Exception as e:
                    print(f"Could not switch to {new_name}: {e}")

    finally:
        if not args.dry_run:
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
