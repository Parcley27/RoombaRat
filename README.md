<p align="center">
  <img src="branding/Full Logo.png" width="220" alt="RoombaRat logo" />
</p>

# RoombaRat

A Roomba with a phone on top that patrols classrooms and reports anyone caught on their phone or otherwise distracted. Built at Cascadia AI Hackathon 2026 by Pierce Nestibo-Oxley, Dale Dai, Jerry Hu, and Daniel Guo.

When the camera spots a phone or distraction, it uploads a screenshot to Box and emails the principal.

## How it works

The Roomba patrols the room autonomously. An iPhone mounted on top streams video via Apple Continuity Camera. Every 2 seconds, a frame is sent to AWS Rekognition. If a phone or distraction is detected above a confidence threshold, RoombaRat uploads the screenshot to Box and fires an alert email via AWS SES.

## Setup

### 1. Install dependencies

```bash
python -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure environment variables

```bash
cp .env.example .env
# fill in all values in .env
```

### 3. AWS (5 min)

1. In IAM, create a user with `AmazonRekognitionReadOnlyAccess` and `AmazonSESFullAccess`
2. Generate an access key and paste it into `.env`
3. In SES, verify both your sender and recipient email addresses (required in sandbox mode)

### 4. Box (5 min)

1. Go to developer.box.com > My Apps > Create New App > Custom App > User Authentication (OAuth 2.0)
2. Copy Client ID and Client Secret into `.env`
3. Under Developer Token, click Generate Developer Token and paste into `BOX_DEVELOPER_TOKEN` (tokens expire after 60 min)

### 5. Camera

With Apple Continuity Camera, the iPhone shows up as a system webcam. `CAMERA_INDEX=0` is usually the built-in camera, so try `1` or `2` if the feed looks wrong.

### 6. Run

```bash
python main.py
```

Press `q` to quit. A green "Monitoring..." label means it's working. "PHONE DETECTED!" (red) or "DISTRACTION DETECTED" (orange) triggers an upload and email.

## Files

| File | Purpose |
|---|---|
| `main.py` | Webcam loop: captures frames, drives detection and alerting |
| `RekognitionController.py` | Sends frame to AWS Rekognition, returns True if phone/distraction found |
| `BoxController.py` | Uploads JPEG to Box, returns shareable URL |
| `EmailController.py` | Sends alert email via AWS SES |

## Tuning

| Variable | Default | Effect |
|---|---|---|
| `CHECK_INTERVAL` | 2 s | How often to hit Rekognition. Lower = faster response, higher API cost. |
| `COOLDOWN` | 30 s | Minimum gap between alert emails. Prevents inbox flooding. |
| `MIN_CONFIDENCE` | — | Rekognition confidence threshold. Lower = more sensitive, more false positives. |
