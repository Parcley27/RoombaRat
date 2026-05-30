"""
config.py – All tunable parameters for the Roomba mapper.
Edit this file before running main.py.
"""
import math

# ─── Network ──────────────────────────────────────────────────────────────
# Set to the IP address printed by the ESP32 on its serial monitor.
ESP32_IP   = "10.0.0.109"
ESP32_PORT = 8080

# ─── Roomba 600-series physical constants ─────────────────────────────────
WHEEL_DIAMETER_MM = 72.0        # mm
WHEELBASE_MM      = 235.0       # mm  (track width between wheel centres)
COUNTS_PER_REV    = 508.8       # encoder counts per wheel revolution
MM_PER_COUNT      = (math.pi * WHEEL_DIAMETER_MM) / COUNTS_PER_REV  # ~0.445 mm

# ─── Occupancy grid ───────────────────────────────────────────────────────
GRID_RESOLUTION_M = 0.05        # metres per cell  (5 cm)
GRID_SIZE         = 300         # cells on each side  → 15 m × 15 m map

# ─── Exploration speeds (mm/s) ────────────────────────────────────────────
SPEED_FORWARD     = 200
SPEED_ROTATE      = 140
SPEED_BACKUP      = 100

# How many 50 ms ticks to spend backing up / rotating after a bump.
BACKUP_TICKS      = 8
ROTATE_TICKS_MIN  = 16
ROTATE_TICKS_MAX  = 32

# How many ticks of no measurable progress before triggering a recovery spin.
STUCK_TICKS       = 60          # 3 seconds at 20 Hz

# ─── Webcam / vision ──────────────────────────────────────────────────────
CAMERA_INDEX      = 0           # cv2.VideoCapture index
# Rotation to apply to every captured frame so the image is "robot-forward = up".
# 0 = none  |  90  = 90° clockwise  |  -90 = 90° CCW  |  180 = flip
CAMERA_ROTATE_DEG = -90         # Continuity Camera: rotate 90° CCW

VISION_ENABLED    = True

# Rows from the bottom of the (corrected) frame to scan for close obstacles.
NEAR_ROWS         = 100
# Rows in the middle band of the frame for distant obstacles.
MID_ROW_START     = 0.30        # fraction of height
MID_ROW_END       = 0.55        # fraction of height

# Canny edge pixel counts that trigger obstacle alerts.
EDGE_THRESH_CLOSE = 200
EDGE_THRESH_FAR   = 120

# Estimated real-world distance (m) when a close/far obstacle is detected.
OBS_DIST_CLOSE_M  = 0.25
OBS_DIST_FAR_M    = 0.60

# ─── Visualisation ────────────────────────────────────────────────────────
MAP_DISPLAY_PX    = 750         # map window side length in pixels
VIZ_FPS_TARGET    = 15
