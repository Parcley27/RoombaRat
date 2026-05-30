# Roomba Phone-Pursuit System — Architecture

## 1. Overview

An iRobot **Series 600** Roomba autonomously roams, detects cell phones via its onboard camera (an iPhone mounted on top, fed to a Mac), drives **toward** any phone it sees while **beeping**, logs each sighting to **Box**, and **emails the principal**.

The Mac is the brain. It captures video, runs detection, makes steering decisions, and talks to the Roomba over serial. The Roomba's onboard **Open Interface (OI)** handles movement and sound. The iPhone is the camera (via Continuity Camera / network stream).

Recognition happens in **two layers**, each with a different job, engine, and cadence:

- **Layer 1 — Phone pursuit (local, every frame).** A local **YOLO** detector on the Mac finds phones and produces bounding boxes for steering. It runs at frame rate with no network round-trip, and it tracks whether the Roomba is *closing in* by watching the box grow.
- **Layer 2 — Person identity (cloud, once per catch).** When the Roomba has closed on a phone, **AWS Rekognition** runs face recognition on the person holding it and returns their **student ID → name**. This fires once at the "catch" moment, not every frame.

The two layers are deliberately decoupled: Layer 1 is the fast inner loop that does the driving; Layer 2 is the slow, authoritative "who was this?" event.

---

## 2. System Diagram

```
                          ┌──────────────────────────────────┐
                          │             MAC (brain)           │
                          │                                   │
  ┌──────────┐  video     │  ┌─────────────────────────────┐ │
  │ iPhone   │──────────▶ │  │ B1: Capture + LOCAL YOLO     │ │
  │ (camera) │ Continuity │  │   OpenCV → YOLO (phone box   │ │
  │ on Roomba│  Camera    │  │   + closing-in / range trend)│ │
  └──────────┘            │  └──────────────┬──────────────┘ │
                          │                 │ detection JSON  │
                          │                 ▼                 │
                          │  ┌─────────────────────────────┐ │
                          │  │ D: Control Loop /            │ │
                          │  │    Orchestrator              │ │
                          │  │  steer() → [catch?] → log    │ │
                          │  └──┬────────┬────────┬─────┬───┘ │
                          │     │        │ catch  │     │     │
                          │     │        ▼        │     │     │
                          │     │  ┌───────────┐  │     │     │
                          │     │  │ B2: Face  │  │     │     │
                          │     │  │ Rekognition│ │     │     │
                          │     │  └─────┬─────┘  │     │     │
                          │     ▼        │        ▼     ▼     │
                          │   A:OI       │     C:Box  D:Email │
                          └─────┼────────┼───────┼──────┼─────┘
                                │ serial  │ search │HTTPS │ SMTP/SES
                                ▼         ▼        ▼      ▼
                          ┌──────────┐ ┌─────────┐ ┌─────┐ ┌──────────┐
                          │  Roomba  │ │  Rekog  │ │ Box │ │ Principal│
                          │ (drive + │ │ Face    │ │store│ │  inbox   │
                          │  beep)   │ │ Collec- │ │+ros-│ └──────────┘
                          └──────────┘ │ tion    │ │ter  │
                                       │(by stu- │ └─────┘
                                       │ dent ID)│
                                       └─────────┘
```

Layer 1 (B1: YOLO) runs every frame and feeds steering. Layer 2 (B2: Rekognition
face search) fires only on a **catch** event and resolves the matched student ID
to a name via the roster stored in Box.

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

### 3.4 Layer 1 — Phone Detector (Local YOLO)
- A local **YOLO** model (e.g. Ultralytics YOLOv8n, COCO classes include `cell phone`) runs on the Mac via OpenCV/PyTorch. No network round-trip, so it can run at frame rate.
- Returns **bounding boxes** with confidence — required for steering, not just a yes/no. Same coordinate convention as before (normalized 0–1), so downstream steering/logging is unchanged.
- **Range / closing-in trend (monocular):** the detector tracks the phone box's normalized **area** across frames. Growing area → Roomba is getting *closer*; shrinking → *farther*; stable → steady. This is relative, not absolute distance, and needs no depth sensor. It drives both the "slow down" rule and the **catch** trigger (area ≥ threshold).
  - Absolute distance (in cm) is *not* attempted. It would require camera calibration plus the phone's known physical width (pinhole estimate) or a depth sensor — unnecessary for pursuit.

### 3.5 Layer 2 — Face Recognition (AWS Rekognition Collection)
- Fires **once per catch**, not every frame: when Layer 1 reports the phone box is large/close (Roomba has reached the holder), the orchestrator grabs a frame and calls Rekognition.
- Uses a Rekognition **Collection**, not `DetectLabels`:
  - **Enrollment (offline, one-time):** `IndexFaces` enrolls each student's photo into the collection with `ExternalImageId = student_id`. The collection stores face *vectors*, not images.
  - **At catch:** `SearchFacesByImage` matches the captured face against the collection and returns the best `ExternalImageId` (= student ID) plus a similarity score.
- Returns the **student ID**; the human-readable **name** is resolved from the roster in Box (§3.6). If no face is found or no match clears the similarity threshold, the catch is logged as `unidentified`.

### 3.6 Data Store (Box)
- Box Developer app (JWT or OAuth2). Each sighting/catch uploads the annotated frame and appends a row to a manifest (CSV/JSON) in a designated folder.
- **Also holds the roster:** a `student_id → name` mapping file (and optionally the enrollment source photos). This is what makes the Rekognition match human-readable. The Rekognition collection and the Box roster share the same `student_id` key. (Box stores the *roster and images*; the face *vectors* live in the AWS collection — they are not duplicated in Box.)

### 3.7 Notification (AWS SES or SMTP)
- Emails the principal on detection/catch, throttled or batched into a digest to avoid spam.
- When a catch has an identity, the email names the student (resolved via the Box roster); otherwise it reports an unidentified catch.

---

## 4. Data Contract

Frozen on Day 1. All modules build against this so they can develop in parallel against mocks.

```json
{
  "detection_id": "uuid",
  "timestamp": "ISO8601",
  "image_path": "local or s3 path to saved frame",
  "source": "yolo",
  "phones_detected": 2,
  "detections": [
    {
      "confidence": 0.94,
      "bounding_box": { "Left": 0.0, "Top": 0.0, "Width": 0.0, "Height": 0.0 },
      "center_x": 0.0,
      "center_y": 0.0,
      "box_area": 0.18,
      "range_trend": "closer"
    }
  ],
  "frame_width": 1280,
  "frame_height": 720,
  "identification": null
}
```

`bounding_box` uses normalized (0–1) coordinates (same convention YOLO and Rekognition share). `center_x` is what the steering controller consumes.

**Layer 1 additions (additive, non-breaking):**
- `source` — which detector produced this frame's boxes (`"yolo"`); lets logs distinguish layers.
- `box_area` — normalized box area, the closeness signal.
- `range_trend` — `"closer" | "farther" | "steady"` vs. the previous frame; drives slow-down and the catch trigger.

**Layer 2 — the `identification` block** is `null` on ordinary frames and populated **only on a catch event**:

```json
"identification": {
  "triggered_by": "catch",
  "face_detected": true,
  "match": {
    "student_id": "S10432",
    "name": "Jordan Lee",
    "similarity": 98.7,
    "face_id": "rekognition-uuid"
  }
}
```

`student_id` is Rekognition's matched `ExternalImageId`; `name` is resolved from the Box roster. On no face or no confident match, `match` is `null` and the event is logged as `unidentified`. Because the phone-detection shape is unchanged and `identification` is optional, **existing steering and logging consumers do not need to change** — they simply ignore the new fields until they care about them.

---

## 5. Control Loop & Pursuit Logic

The Mac runs a continuous loop. When a phone is present, it performs **visual servoing** — steering based on where the phone sits in the frame.

```
loop:
    frame = capture()
    detection = yolo_detect(frame)       # Layer 1: local YOLO, every frame
    if detection.phones_detected > 0:
        box = highest_confidence_box(detection)
        cmd = steer_command(box, frame_width)
        roomba.drive(cmd)                # toward the phone
        roomba.beep()                    # beep each iteration while visible

        if is_catch(box):                # close enough → reached the holder
            roomba.stop()
            ident = rekog_identify(frame)  # Layer 2: face search (once)
            detection.identification = ident
            notify(detection)            # email names the student if matched
        box_log(detection)               # log every frame / catch
    else:
        roomba.search()                  # rotate to look, or stop
```

### Catch trigger
`is_catch(box)` is true when the phone box is large/close enough that the Roomba
has effectively reached the holder — e.g. `box_area >= CATCH_AREA` and
`range_trend != "closer"` for a couple of frames (it has stopped gaining). This
debounce keeps Layer 2 from firing repeatedly on one approach, which matters
because each Rekognition call costs latency and money.

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

The two-layer split *is* the latency strategy:

- **Layer 1 (YOLO) is local**, so steering runs at frame rate with no network round-trip — smooth pursuit, no jerk.
- **Layer 2 (Rekognition) is a network round-trip** (hundreds of ms per call) and would be far too slow to run every frame. It doesn't need to: it fires **once per catch**, when the Roomba has already stopped. A few hundred ms there is invisible to the user and keeps API cost/quota low.

Keep Roomba speed moderate (~100–150 mm/s) so it doesn't overshoot, and debounce the catch (§5) so Layer 2 fires once per approach rather than repeatedly.

---

## 8. Team Workstreams (Parallel)

| Owner | Module | Deliverable | Builds against |
|-------|--------|-------------|----------------|
| **A** | Roomba OI driver (`pyserial`) | `drive()`, `turn()`, `stop()`, `beep()`, mode/keep-alive | Real hardware (independent) |
| **B1** | Layer 1 — local YOLO vision | frame in → detection JSON (box, `center_x`, `box_area`, `range_trend`) out | Sample frames |
| **B2** | Layer 2 — Rekognition face rec | enroll roster (`IndexFaces`); frame in → `identification` block out; student_id→name resolve | Sample faces + a test collection |
| **C** | Box logging + roster | detection JSON → annotated frame + manifest row; host `student_id → name` roster | Mocked JSON |
| **D** | Orchestration + control loop | main loop, steering, catch trigger, email/SES, wiring A·B1·B2·C | Mock A/B/C functions |

**Critical path:** Person A (OI driver) and the frozen data contract. Everything else mocks against the contract. B1 and B2 are independent — B1 gates pursuit, B2 only needs the catch frame, so they can be built in parallel and B2 can use a static test image until the loop is wired.

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
