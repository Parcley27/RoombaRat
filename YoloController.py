"""
Local phone detector — drop-in alternative to RekognitionController.

Runs a COCO-pretrained YOLO model on-device (no network, no per-call cost), so
detection happens every frame instead of every ~2s. COCO already has a
'cell phone' class (id 67), which is what we look for.

Exposes the same check_for_phone(frame) -> bool signature as
RekognitionController so roomba_patrol can swap between them freely.

Needs:  pip install ultralytics   (pulls in PyTorch)
The weights file (default yolov8n.pt, ~6MB) auto-downloads on first use.
"""

import os

PHONE_CLASSES = {'cell phone'}                       # COCO class name
MIN_CONFIDENCE = float(os.getenv('YOLO_MIN_CONFIDENCE', '0.35'))
WEIGHTS = os.getenv('YOLO_WEIGHTS', 'yolov8n.pt')    # n=nano (fastest on CPU)

_model = None


def _get_model():
    global _model
    if _model is None:
        from ultralytics import YOLO     # lazy — only import torch if YOLO is used
        _model = YOLO(WEIGHTS)
    return _model


def detect_phone(frame):
    """Return (hit, best_conf, box_or_None) for the highest-confidence phone."""
    model   = _get_model()
    results = model.predict(frame, conf=MIN_CONFIDENCE, verbose=False)

    best = None
    for r in results:
        names = r.names
        for b in r.boxes:
            if names[int(b.cls)] in PHONE_CLASSES:
                conf = float(b.conf)
                if best is None or conf > best[0]:
                    xyxy = [int(v) for v in b.xyxy[0].tolist()]
                    best = (conf, xyxy)

    if best is None:
        return False, 0.0, None
    return True, best[0], best[1]


def check_for_phone(frame):
    """Boolean, matching RekognitionController.check_for_phone."""
    hit, _, _ = detect_phone(frame)
    return hit


def warmup():
    """Force model load + download now (so the first real frame isn't slow)."""
    _get_model()
