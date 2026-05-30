# Roomba Face Follower

The Roomba autonomously tracks and follows a person using face detection on the Mac webcam, with an ESP32 acting as a **USB-serial bridge** to the Roomba's Open Interface.

> **Connection:** the Mac talks to the ESP32 over a **USB cable** (USB serial), and the ESP32 talks to the Roomba over UART. There is no WiFi — earlier versions used an ESP32 WiFi access point, but the project now connects by serial.

---

## Hardware

| Component | Role |
|---|---|
| iRobot Roomba 600 series | Mobile platform |
| ESP32 | USB-serial link to Mac + Roomba OI (UART) bridge |
| Mac (with webcam) | Face detection + control logic |

### ESP32 → Roomba wiring (7-pin Mini-DIN on top of robot)

| ESP32 GPIO | Roomba pin | Signal |
|---|---|---|
| GPIO 4 | Pin 5 | BRC (wake) |
| GPIO 25 | Pin 3 | TX → Roomba RXD |
| GPIO 26 | Pin 4 | RX ← Roomba TXD |
| GND | Pin 6 or 7 | Ground |

> Roomba logic is 3.3 V — connect directly to ESP32, no level shifter needed. Do not connect Roomba Vpwr (pins 1–2) to the ESP32.

**Mac ↔ ESP32:** a USB cable carrying data (not a power-only cable). This is both the power source and the command link for the ESP32.

---

## How it works

### `esp32/follower.py` (runs on ESP32)

1. **Communicates with the Mac over USB serial** (`sys.stdin` / `sys.stdout`) — no WiFi. UART1 (GPIO 25/26) stays dedicated to the Roomba.

2. **Wakes the Roomba** by toggling the BRC pin, then brings up the Open Interface (OI) with a **robust retry**: it sends Start + Full and reads back the OI mode (Query List, packet 35), retrying — and falling back through Safe → Full — until the Roomba confirms **Full mode** (mode byte `3`):
   - Opcode `128` — Start (enter OI Passive mode)
   - Opcode `131` — Safe mode (stepping stone)
   - Opcode `132` — Full mode (enables all actuator commands)

3. **Plays a boot sound** via the Roomba's speaker to confirm UART is working.

4. **Reads newline-terminated commands** from USB serial:
   - `"v r\n"` — Roomba Drive opcode 137: velocity in mm/s, radius in mm
     - Special radius values: `-32768` = straight, `-1` = clockwise spin, `1` = counter-clockwise spin
   - `"C\n"` — (re)confirms Full mode and plays a connect sound
   - `"B\n"` — plays an alert sound (used by the patrol script when a phone is caught)

> **No watchdog:** if the Mac goes silent the Roomba simply holds its last command, so make sure the Mac script always sends a stop (`0 -32768`) on exit (both scripts here do).

---

### `follow_person.py` (runs on Mac)

1. **Finds the ESP32 serial port** automatically (matches descriptions like `cp210`, `ch340`, `uart`, `esp32`, `usb serial`), or uses `SERIAL_PORT` from `.env`, or `--port`. Opens it at **115200 baud** and sends `"C\n"`.

2. **Opens the webcam** and runs OpenCV's Haar cascade face detector (`haarcascade_frontalface_default.xml`) on every frame.

3. **Computes drive parameters** from the detected face bounding box:

   | Measurement | How computed |
   |---|---|
   | `area_ratio` | `(face_w × face_h) / frame_area` — proxy for distance |
   | `horiz_err` | `(face_center_x − frame_center_x) / (frame_width / 2)` — range −1.0 to +1.0 |

4. **Control logic** (`compute_drive`):

   | Condition | Action |
   |---|---|
   | `area > 0.07` (too close) | Back up straight at −80 mm/s |
   | `\|err\| ≥ 0.40` (large error) | In-place spin at 100 mm/s |
   | `\|err\| < 0.40`, face far (`area < 0.03`) | Curve forward at 180 mm/s — radius shrinks 800 mm → 200 mm as error grows |
   | `\|err\| < 0.40`, at distance | Curve forward at a slow 100 mm/s crawl (never fully stops, so it stays engaged) |
   | No face for > 500 ms | Slow counter-clockwise search spin at 60 mm/s |

   The proportional curved drive (instead of binary left/right) prevents oscillation — the robot arcs smoothly toward the face while continuing forward.

5. **Face timeout (500 ms)**: if detection drops for a frame or two, the last known position is used. Only after 500 ms of no face does it enter search mode. A face must also be seen for `CONFIRM_FRAMES` (2) consecutive frames before it's trusted.

6. **Sends `"v r\n"` over serial** at most every 100 ms, and sends `"0 -32768\n"` (stop) on exit.

> **Beyond following:** [`roomba_patrol.py`](../roomba_patrol.py) (repo root) builds on this — it follows faces and additionally runs phone detection (local YOLO or AWS Rekognition), beeps the Roomba, and fires a Box upload + email alert when a phone is caught.

---

## Setup

### One-time ESP32 setup
1. Open `esp32/follower.py` in Thonny
2. **File → Save as → MicroPython device**, name it `main.py`

### Every run
1. **Connect the ESP32 to the Mac with a USB (data) cable** — this powers it and carries commands.
2. Press the Roomba's **CLEAN** button to power it on (BRC alone can't turn it on from fully off).
3. Power/reset the ESP32 so `main.py` runs. Wait for the **boot beep** (two notes) — confirms UART is up and the Roomba reached Full mode.
4. Install dependencies if needed: `pip install -r ../requirements.txt`
5. Set the serial port: put `SERIAL_PORT=COM5` (Windows) or `SERIAL_PORT=/dev/cu.usbserial-XXXX` (Mac) in `.env`, or leave it blank to auto-detect, or pass `--port`.
6. Run: `python follow_person.py`
7. Listen for the **connect beep** (three rising notes) — confirms the serial link is live.
8. The camera window opens. Stand in front of the camera and the Roomba will follow.
9. Press **Q** in the camera window to stop.

---

## Tuning

All constants are at the top of `follow_person.py`:

| Constant | Default | Effect |
|---|---|---|
| `TARGET_AREA_MIN` | `0.03` | How far the robot chases before easing off the throttle |
| `TARGET_AREA_MAX` | `0.07` | How close before backing up |
| `SPIN_THRESHOLD` | `0.40` | Error above this triggers in-place spin |
| `FORWARD_SPEED` | `180` | Max forward speed mm/s |
| `SEARCH_SPEED` | `60` | Search-spin speed mm/s when no face is found |
| `BACKUP_SPEED` | `-80` | Reverse speed mm/s when too close |
| `FACE_TIMEOUT` | `0.5` | Seconds to coast after losing a face |
| `CONFIRM_FRAMES` | `2` | Consecutive frames a face must persist before it's trusted |
