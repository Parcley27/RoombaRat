# Roomba Face Follower

The Roomba autonomously tracks and follows a person using face detection on a Mac webcam and an ESP32 as a wireless bridge to the Roomba's serial interface.

---

## Hardware

| Component | Role |
|---|---|
| iRobot Roomba 600 series | Mobile platform |
| ESP32 | WiFi AP + Roomba serial bridge |
| Mac (with webcam) | Face detection + control logic |

### ESP32 → Roomba wiring (7-pin Mini-DIN on top of robot)

| ESP32 GPIO | Roomba pin | Signal |
|---|---|---|
| GPIO 4 | Pin 5 | BRC (wake) |
| GPIO 25 | Pin 3 | TX → Roomba RXD |
| GPIO 26 | Pin 4 | RX ← Roomba TXD |
| GND | Pin 6 or 7 | Ground |

> Roomba logic is 3.3 V — connect directly to ESP32, no level shifter needed. Do not connect Roomba Vpwr (pins 1–2) to the ESP32.

---

## How it works

### `follower.py` (runs on ESP32)

1. **Creates a WiFi access point** named `Roomba` (password `roomba123`). This bypasses managed networks with client isolation — the Mac connects directly to the ESP32 with no infrastructure in between.

2. **Wakes the Roomba** by toggling the BRC pin, then sends Roomba Open Interface (OI) opcodes over UART:
   - Opcode `128` — Start (enter OI Passive mode)
   - Opcode `132` — Full mode (enables all actuator commands)

3. **Plays a boot sound** via Roomba's speaker to confirm UART is working.

4. **Listens on UDP port 9000** for drive commands from the Mac. Commands are newline-terminated text:
   - `"v r\n"` — Roomba Drive opcode 137: velocity in mm/s, radius in mm
     - Special radius values: `-32768` = straight, `-1` = clockwise spin, `1` = counter-clockwise spin
   - `"C\n"` — plays a connect confirmation sound

5. **Watchdog**: if no command arrives for 1 second (e.g. Mac crashes), the Roomba stops automatically.

---

### `follow_person.py` (runs on Mac)

1. **Connects to the ESP32** at fixed IP `192.168.4.1` (the ESP32 AP's default gateway).

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
   | Centered (`err < 0.08`) + too far | Forward at 180 mm/s |
   | Centered + right distance | Stop |
   | Small error (0.08 – 0.40) | Curve forward — radius shrinks 800 mm → 200 mm as error grows |
   | Large error (`err > 0.40`) | In-place spin at 70 mm/s |
   | No face for > 500 ms | Slow counter-clockwise search spin at 60 mm/s |

   The proportional curved drive (instead of binary left/right) prevents oscillation — the robot arcs smoothly toward the face while continuing forward.

5. **Face timeout (500 ms)**: if detection drops for a frame or two, the last known position is used. Only after 500 ms of no face does it enter search mode.

6. **Sends `"v r\n"` over UDP** to `192.168.4.1:9000` at most every 100 ms.

---

## Setup

### One-time ESP32 setup
1. Open `follower.py` in Thonny
2. **File → Save as → MicroPython device**, name it `main.py`
3. Disconnect USB — the ESP32 only needs power from here on (power bank, USB charger, etc.)

### Every run
1. Power the ESP32
2. Wait for the **boot beep** (two notes) from the Roomba — confirms WiFi AP is up and Roomba is in Full mode
3. On your Mac: connect to **`Roomba`** WiFi (password: `roomba123`)
4. Install dependency if needed: `pip install opencv-python`
5. Run: `python follow_person.py`
6. Listen for the **connect beep** (three rising notes) — confirms UDP link is live
7. The camera window opens. Stand in front of the camera and the Roomba will follow.
8. Press **Q** in the camera window to stop.

---

## Tuning

All constants are at the top of `follow_person.py`:

| Constant | Default | Effect |
|---|---|---|
| `TARGET_AREA_MIN` | `0.03` | How far the robot chases before stopping |
| `TARGET_AREA_MAX` | `0.07` | How close before backing up |
| `TURN_THRESHOLD` | `0.08` | Horizontal dead zone (smaller = more reactive) |
| `SPIN_THRESHOLD` | `0.40` | Error above this triggers in-place spin |
| `FORWARD_SPEED` | `180` | Max forward speed mm/s |
| `FACE_TIMEOUT` | `0.5` | Seconds to coast after losing face |
