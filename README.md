# RoombaRat

> A Roomba with a phone on top that patrols classrooms and snitches on anyone caught using their phone, or otherwise distracted.
> Built at Cascadia AI Hackathon 2026 by Pierce Nestibo-Oxley, Dale Dai, Jerry Hu, and Daniel Guo.

When the camera detects a phone or other distraction, it uploads a screenshot to Box and emails the principal.

---

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

### 3. AWS setup (5 min)

1. In IAM, create a user with these policies:
   - `AmazonRekognitionReadOnlyAccess`
   - `AmazonSESFullAccess`
2. Generate an access key and paste into `.env`
3. In SES → **Verified identities**, verify both your sender and recipient email addresses (required in sandbox mode)

### 4. Box setup (5 min)

1. Go to [developer.box.com](https://developer.box.com) → **My Apps** → **Create New App** → Custom App → User Authentication (OAuth 2.0)
2. Copy **Client ID** and **Client Secret** into `.env`
3. Under **Developer Token**, click **Generate Developer Token** — paste into `BOX_DEVELOPER_TOKEN` (tokens expire after 60 min, regenerate as needed)

### 5. Camera

With Apple Continuity Camera, the iPhone shows up as a system webcam. `CAMERA_INDEX=0` is usually the built-in, so try `1` or `2` if the feed looks wrong.

### 6. Run

```bash
python main.py
```

Press `q` to quit. A green "Monitoring..." label means it's working. Red "PHONE DETECTED!" triggers an upload + email. Samething but an orange "DISTRACTION DETECTED" for other stuff.

---

## File overview

| File | Purpose |
|---|---|
| `main.py` | Webcam loop — captures frames, drives detection + alerting |
| `RekognitionController.py` | Sends frame to AWS Rekognition, returns True if phone/distraction found |
| `BoxController.py` | Uploads JPEG to Box, returns shareable URL |
| `EmailController.py` | Sends alert email via AWS SES |

## Tuning

- **`CHECK_INTERVAL`** — how often to hit Rekognition (every 2 s by default). Lower = faster response but more API cost.
- **`COOLDOWN`** — minimum gap between alert emails (30 s). Prevents inbox flooding.
- **`MIN_CONFIDENCE`** — Rekognition confidence threshold. Lower = more sensitive but more false positives.
