# Roomba Phone-Pursuit System — Architecture

## 1. Overview

An iRobot **Series 600** Roomba autonomously roams, detects cell phones via its onboard camera (an iPhone mounted on top, fed to a Mac), drives **toward** any phone it sees while **beeping**, logs each sighting to **Box**, and **emails the principal**.

The Mac is the brain. It captures video, runs detection, makes steering decisions, and talks to the Roomba over serial. The Roomba's onboard **Open Interface (OI)** handles movement and sound. The iPhone is the camera (via Continuity Camera / network stream). AWS Rekognition is the detector of record.

---

## 2. System Diagram

```
                          ┌─────────────────────────────┐
                          │           MAC (brain)        │
                          │                              │
  ┌──────────┐  video     │  ┌────────────────────────┐  │
  │ iPhone   │──────────▶ │  │ B: Capture + Detect     │  │
  │ (camera) │ Continuity │  │   OpenCV → Rekognition  │  │
  │ on Roomba│  Camera    │  └───────────┬────────────┘  │
  └──────────┘            │              │ detection JSON │
                          │              ▼                │
                          │  ┌────────────────────────┐  │
                          │  │ D: Control Loop /        │  │
                          │  │    Orchestrator          │  │
                          │  │  steer() → log → notify  │  │
                          │  └───┬─────────┬────────┬───┘  │
                          │      │         │        │      │
                          │      ▼         ▼        ▼      │
                          │   A:OI     C:Box     D:Email   │
                          └──────┼─────────┼────────┼──────┘
                                 │ serial  │ HTTPS  │ SMTP/SES
                                 ▼         ▼        ▼
                          ┌──────────┐ ┌──────┐ ┌──────────┐
                          │  Roomba  │ │ Box  │ │ Principal│
                          │ (drive + │ │store │ │  inbox   │
                          │  beep)   │ └──────┘ └──────────┘
                          └──────────┘
```

---

## 3. Components

### 3.1 Camera (iPhone)
- iPhone mounted on the Roomba, feeding the Mac as a webcam via **Continuity Camera** (wireless, ~30 ft, same network) or an RTSP/IP-camera app if range is an issue.
- Accessed in Python via `cv2.VideoCapture(index)`. Index must be confirmed at runtime (the built-in FaceTime camera may occupy index 0).

### 3.2 Brain (Mac, Python)
- Runs all four software workstreams as cooperating modules.
- Single main control loop owned by Person D drives the cadence: capture → detect → steer → log → notify.

### 3.3 Roomba (Series 600, Open Interface)
- Connected to the Mac over the 7-pin Mini-DIN serial port (TXD pin 3, RXD pin 4, GND pins 6/7), accessed with `pyserial`.
- Movement via OI `DRIVE` (opcode 137). Sound via OI `SONG` (140) + `PLAY` (141) on the onboard speaker.
- Operates in **Safe mode** (131) by default so cliff/wheel-drop sensors auto-stop it; **Full mode** (132) only if pursuit requires ignoring those.

### 3.4 Detector (AWS Rekognition)
- `DetectLabels` with a confidence threshold (≥80%), filtering for `Mobile Phone` / `Cell Phone` / `Phone`.
- Returns **bounding boxes** — required for steering, not just a yes/no.

### 3.5 Data Store (Box)
- Box Developer app (JWT or OAuth2). Each sighting uploads the annotated frame and appends a row to a manifest (CSV/JSON) in a designated folder.

### 3.6 Notification (AWS SES or SMTP)
- Emails the principal on detection, throttled or batched into a digest to avoid spam.

---

## 4. Data Contract

Frozen on Day 1. All modules build against this so they can develop in parallel against mocks.

```json
{
  "detection_id": "uuid",
  "timestamp": "ISO8601",
  "image_path": "local or s3 path to saved frame",
  "phones_detected": 2,
  "detections": [
    {
      "confidence": 0.94,
      "bounding_box": { "Left": 0.0, "Top": 0.0, "Width": 0.0, "Height": 0.0 },
      "center_x": 0.0,
      "center_y": 0.0
    }
  ],
  "frame_width": 1280,
  "frame_height": 720
}
```

`bounding_box` uses Rekognition's normalized (0–1) coordinates. `center_x` is what the steering controller consumes.

---

## 5. Control Loop & Pursuit Logic

The Mac runs a continuous loop. When a phone is present, it performs **visual servoing** — steering based on where the phone sits in the frame.

```
loop:
    frame = capture()
    detection = detect(frame)            # Rekognition
    if detection.phones_detected > 0:
        box = highest_confidence_box(detection)
        cmd = steer_command(box, frame_width)
        roomba.drive(cmd)                # toward the phone
        roomba.beep()                    # beep each iteration while visible
        box_log(detection)               # log the sighting
        notify(detection)                # email (throttled)
    else:
        roomba.search()                  # rotate to look, or stop
```

### Steering rule (proportional)
- Phone **left** of center → turn left.
- Phone **right** of center → turn right.
- Phone **centered** (within ~15% of frame center) → drive forward.
- Bounding box **growing** (filling more frame) → getting close → slow / stop.

```python
def steer_command(box, frame_w):
    cx = (box['Left'] + box['Width'] / 2) * frame_w
    err = cx - frame_w / 2            # negative = phone is left
    if abs(err) < frame_w * 0.15:     # centered enough
        return ('forward', 150)       # mm/s
    return ('turn', -err)             # turn proportional to error
```

---

## 6. Open Interface Reference (Roomba)

| Action | Opcode | Notes |
|--------|--------|-------|
| Start  | 128 | Wakes OI, required first |
| Safe mode | 131 | Auto-stops on cliff/wheel-drop |
| Full mode | 132 | Ignores safety sensors |
| Drive | 137 | velocity (mm/s) + radius (mm); straight = radius 32768, spin = 1 (CCW) / 65535 (CW) |
| Song  | 140 | Define: song#, length, note/duration pairs (MIDI notes, e.g. 72 = C5) |
| Play  | 141 | Trigger a defined song# — the beep |

**Known gotcha:** the OI sleeps and Series 600 can drop out of mode. Keep it awake by re-sending Start or pulsing the BRC pin. Test this first — it's the most common failure.

---

## 7. Latency Strategy

Rekognition is a network round-trip (hundreds of ms per call), so steering off Rekognition alone is jerky and laggy.

- **Option A (simple, recommended for demo):** Accept slow pursuit. Keep Roomba speed low (~100–150 mm/s) so it doesn't overshoot between detections.
- **Option B (smooth):** Run a **local** detector (YOLO / OpenCV) for fast steering; call Rekognition only periodically for the official "log this" event.

The architecture supports both — local detection slots in as a fast inner loop, Rekognition as the slower system-of-record.

---

## 8. Team Workstreams (Parallel)

| Owner | Module | Deliverable | Builds against |
|-------|--------|-------------|----------------|
| **A** | Roomba OI driver (`pyserial`) | `drive()`, `turn()`, `stop()`, `beep()`, mode/keep-alive | Real hardware (independent) |
| **B** | Vision + Rekognition | frame in → detection JSON (with bounding box) out | Sample frames |
| **C** | Box logging | detection JSON → annotated frame + manifest row in Box | Mocked JSON |
| **D** | Orchestration + control loop | main loop, steering logic, email/SES, wiring A·B·C | Mock A/B/C functions |

**Critical path:** Person A (OI driver) and the frozen data contract. Everything else mocks against the contract.

---

## 9. Decisions to Lock Early

1. **Camera index** — confirm which `VideoCapture` index is the iPhone, not the built-in cam.
2. **Safe vs Full mode** — will cliff sensors interrupt pursuit?
3. **Local detector or not** — depends on how smooth pursuit needs to be (Section 7).
4. **Email cadence** — per-detection vs. batched digest.
5. **Phone-leaves-frame behavior** — stop, or rotate to search.

---

## 10. Privacy Note

The system captures images of phones (and likely nearby people) and stores them in Box plus emails them out. Confirm consent and data-handling expectations with the principal before deploying in any shared space.
