"""
vision.py – Webcam-based obstacle detection.

The laptop is mounted so its webcam faces the front of the Roomba.
We correct for the 90° rotation, then detect obstacles via:
  1. Canny edge density in the lower (near-field) band  → very close obstacle
  2. Canny edge density in the middle band              → distant obstacle
  3. Floor-colour segmentation as a secondary check

Returns an estimated distance-to-obstacle so the mapping layer can mark
the occupancy grid accordingly.
"""
import cv2
import numpy as np
from config import (CAMERA_INDEX, CAMERA_ROTATE_DEG,
                    NEAR_ROWS, MID_ROW_START, MID_ROW_END,
                    EDGE_THRESH_CLOSE, EDGE_THRESH_FAR,
                    OBS_DIST_CLOSE_M, OBS_DIST_FAR_M)


def _rotation_code(deg: int):
    """Map rotation degrees to an OpenCV rotate code (or None)."""
    mapping = {90: cv2.ROTATE_90_CLOCKWISE,
               -90: cv2.ROTATE_90_COUNTERCLOCKWISE,
               270: cv2.ROTATE_90_COUNTERCLOCKWISE,
               180: cv2.ROTATE_180}
    return mapping.get(deg)


class CameraDetector:

    def __init__(self):
        self._cap = cv2.VideoCapture(CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        self._rot_code = _rotation_code(CAMERA_ROTATE_DEG)

        self._frame:    np.ndarray | None = None
        self._debug:    np.ndarray | None = None  # annotated frame for display

        # Adaptive floor-colour model (HSV range, updated each frame)
        self._floor_hsv_lo = np.array([0,   0,  80], dtype=np.uint8)
        self._floor_hsv_hi = np.array([180, 60, 255], dtype=np.uint8)

    # ── Public API ────────────────────────────────────────────────────────

    def update(self) -> tuple[bool, float | None]:
        """
        Capture a frame and run obstacle detection.
        Returns (obstacle_detected, estimated_distance_m | None).
        """
        ret, raw = self._cap.read()
        if not ret:
            return False, None

        # Correct orientation
        frame = cv2.rotate(raw, self._rot_code) if self._rot_code is not None else raw
        self._frame = frame
        h, w = frame.shape[:2]

        # ── Edge-based detection ──────────────────────────────────────────
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edges   = cv2.Canny(blurred, 40, 120)

        # Near band: bottom NEAR_ROWS rows, centre half of width
        near_band = edges[h - NEAR_ROWS: h,  w // 4: 3 * w // 4]
        near_cnt  = int(np.count_nonzero(near_band))

        # Mid band: middle fraction of the frame
        y0 = int(h * MID_ROW_START)
        y1 = int(h * MID_ROW_END)
        mid_band = edges[y0:y1, w // 4: 3 * w // 4]
        mid_cnt  = int(np.count_nonzero(mid_band))

        # ── Floor-colour segmentation (secondary) ─────────────────────────
        # Learn floor colour from bottom 15% of frame (assumed traversable).
        floor_sample = frame[int(h * 0.85):, :]
        self._update_floor_model(floor_sample)

        # Detect non-floor pixels in mid band
        hsv_mid = cv2.cvtColor(frame[y0:y1, w // 4: 3 * w // 4],
                                cv2.COLOR_BGR2HSV)
        floor_mask   = cv2.inRange(hsv_mid, self._floor_hsv_lo, self._floor_hsv_hi)
        non_floor_px = (w // 2) * (y1 - y0) - int(np.count_nonzero(floor_mask))
        colour_alert = non_floor_px > (w // 2) * (y1 - y0) * 0.40

        # ── Decision ──────────────────────────────────────────────────────
        obstacle, dist = False, None
        if near_cnt > EDGE_THRESH_CLOSE:
            obstacle, dist = True, OBS_DIST_CLOSE_M
        elif near_cnt > EDGE_THRESH_CLOSE // 2 or colour_alert:
            obstacle, dist = True, OBS_DIST_CLOSE_M * 1.5
        elif mid_cnt > EDGE_THRESH_FAR:
            obstacle, dist = True, OBS_DIST_FAR_M

        # ── Debug overlay ─────────────────────────────────────────────────
        dbg = frame.copy()
        cv2.rectangle(dbg, (w//4, h - NEAR_ROWS), (3*w//4, h),
                      (0, 0, 255) if near_cnt > EDGE_THRESH_CLOSE else (0, 255, 0), 2)
        cv2.rectangle(dbg, (w//4, y0), (3*w//4, y1),
                      (0, 165, 255) if mid_cnt > EDGE_THRESH_FAR else (255, 255, 0), 2)
        label = f"OBS {dist:.2f}m" if obstacle else "CLEAR"
        colour = (0, 0, 255) if obstacle else (0, 220, 0)
        cv2.putText(dbg, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)
        cv2.putText(dbg, f"near={near_cnt} mid={mid_cnt}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        self._debug = dbg

        return obstacle, dist

    def get_display_frame(self) -> np.ndarray | None:
        """Return annotated frame for the camera window."""
        return self._debug

    def get_raw_frame(self) -> np.ndarray | None:
        return self._frame

    def release(self):
        self._cap.release()

    # ── Internal ──────────────────────────────────────────────────────────

    def _update_floor_model(self, sample: np.ndarray):
        """Adapt floor HSV range from a region that should be floor."""
        if sample.size == 0:
            return
        hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
        lo  = np.percentile(hsv.reshape(-1, 3), 10, axis=0).astype(np.uint8)
        hi  = np.percentile(hsv.reshape(-1, 3), 90, axis=0).astype(np.uint8)
        # Widen range slightly and clamp
        lo = np.clip(lo.astype(int) - 20, 0,   255).astype(np.uint8)
        hi = np.clip(hi.astype(int) + 20, 0,   255).astype(np.uint8)
        lo[0], hi[0] = 0, 180   # keep full hue range (floor colour varies widely)
        self._floor_hsv_lo = lo
        self._floor_hsv_hi = hi
