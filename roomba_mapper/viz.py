"""
viz.py – LIDAR-style occupancy-grid visualiser.

Visual language mirrors what ROS / rviz produces:
  • Near-black   = unexplored / unknown
  • White        = confirmed free space
  • Black        = wall / obstacle
  • Cyan rays    = current simulated scan fan from robot
  • Green dots   = robot trail
  • Red arrow    = robot pose
  • Subtle grid  = metric scale reference

Key bindings (focus the Map window):
  Q / Esc  – quit
  S        – save map PNG
  R        – clear trail
  +/-      – zoom in / out
"""

import math
import time
import cv2
import numpy as np

from config import MAP_DISPLAY_PX, GRID_SIZE, GRID_RESOLUTION_M

# ── Colour palette (BGR) ──────────────────────────────────────────────────────
_C_UNKNOWN  = (28,  28,  28)    # near-black void
_C_FREE     = (240, 240, 240)   # white free space
_C_OCC      = (0,   0,   0)    # pure black walls
_C_TRAIL    = (60,  180, 60)    # green path
_C_SCAN_RAY = (80,  160, 140)   # muted cyan ray lines
_C_SCAN_HIT = (0,   230, 230)   # bright cyan endpoint dot
_C_ROBOT    = (0,   50,  235)   # blue-red robot marker
_C_GRID     = (48,  48,  48)    # subtle grid lines
_C_HUD_BG   = (15,  15,  15)    # HUD panel background
_C_HUD_TXT  = (0,   230, 120)   # green terminal text

# Scan fan parameters
_SCAN_RAYS       = 90           # rays per sweep (every 4°)
_SCAN_RANGE_M    = 3.5          # max ray length in metres
_SCAN_RANGE_CELLS = int(_SCAN_RANGE_M / GRID_RESOLUTION_M)


class MapVisualizer:

    def __init__(self):
        self._trail: list[tuple[float, float]] = []
        self._zoom  = 2          # pixels per grid cell (start at 2×)
        self._cx    = GRID_SIZE // 2   # viewport centre (grid coords)
        self._cy    = GRID_SIZE // 2

        cv2.namedWindow("Roomba Map",  cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Roomba Map",  MAP_DISPLAY_PX, MAP_DISPLAY_PX + 90)
        cv2.resizeWindow("Camera Feed", 480, 360)

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, grid, rx, ry, rtheta, state_str, cam_frame) -> bool:
        robot_gx, robot_gy = grid.world_to_grid(rx, ry)

        # Keep viewport centred on robot
        self._cx = robot_gx
        self._cy = robot_gy

        # ── Base map image ────────────────────────────────────────────────
        prob  = grid.probability()
        base  = np.full((GRID_SIZE, GRID_SIZE, 3), _C_UNKNOWN, dtype=np.uint8)
        base[prob < grid.THRESH_FREE]  = _C_FREE
        base[prob > grid.THRESH_OCC]   = _C_OCC

        # ── Metric grid overlay ───────────────────────────────────────────
        # Draw a line every 1 m (= every 20 cells at 5 cm resolution)
        cells_per_metre = int(1.0 / GRID_RESOLUTION_M)
        ox = grid.origin % cells_per_metre
        oy = grid.origin % cells_per_metre
        for gx in range(ox, GRID_SIZE, cells_per_metre):
            base[:, gx] = np.where(
                (base[:, gx] == list(_C_FREE)).all(axis=1, keepdims=True),
                _C_GRID, base[:, gx])
        for gy in range(oy, GRID_SIZE, cells_per_metre):
            base[gy, :] = np.where(
                (base[gy, :] == list(_C_FREE)).all(axis=1, keepdims=True),
                _C_GRID, base[gy, :])

        # ── Scan fan ─────────────────────────────────────────────────────
        base = self._draw_scan_fan(base, grid, robot_gx, robot_gy, rtheta)

        # ── Trail ─────────────────────────────────────────────────────────
        self._trail.append((rx, ry))
        for wx, wy in self._trail[-3000:]:
            gx, gy = grid.world_to_grid(wx, wy)
            if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
                base[gy, gx] = _C_TRAIL

        # ── Crop viewport ─────────────────────────────────────────────────
        half  = MAP_DISPLAY_PX // (2 * self._zoom)
        x0    = max(0, self._cx - half)
        y0    = max(0, self._cy - half)
        x1    = min(GRID_SIZE, x0 + MAP_DISPLAY_PX // self._zoom)
        y1    = min(GRID_SIZE, y0 + MAP_DISPLAY_PX // self._zoom)
        crop  = base[y0:y1, x0:x1]
        view  = cv2.resize(crop, (MAP_DISPLAY_PX, MAP_DISPLAY_PX),
                           interpolation=cv2.INTER_NEAREST)

        # ── Robot marker on the scaled view ───────────────────────────────
        scale  = MAP_DISPLAY_PX / (x1 - x0)
        px     = int((robot_gx - x0) * scale)
        py     = int((robot_gy - y0) * scale)
        arrow  = int(18 * self._zoom / 2)
        ex     = int(px + arrow * math.cos(rtheta))
        ey     = int(py - arrow * math.sin(rtheta))
        cv2.circle(view, (px, py), max(4, int(5 * self._zoom / 2)),
                   _C_ROBOT, -1)
        cv2.arrowedLine(view, (px, py), (ex, ey), _C_ROBOT, 2, tipLength=0.4)

        # ── Scale bar ─────────────────────────────────────────────────────
        bar_cells = cells_per_metre              # 1 m
        bar_px    = int(bar_cells * scale)
        bx, by    = 20, MAP_DISPLAY_PX - 22
        cv2.rectangle(view, (bx - 1, by - 1),
                      (bx + bar_px + 1, by + 7), (0, 0, 0), -1)
        cv2.rectangle(view, (bx, by), (bx + bar_px, by + 5),
                      (200, 200, 200), -1)
        cv2.putText(view, "1 m", (bx + bar_px + 5, by + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # ── HUD panel ─────────────────────────────────────────────────────
        hud_h  = 90
        canvas = np.zeros((MAP_DISPLAY_PX + hud_h, MAP_DISPLAY_PX, 3),
                          dtype=np.uint8)
        canvas[:MAP_DISPLAY_PX] = view
        canvas[MAP_DISPLAY_PX:] = _C_HUD_BG

        heading_deg = math.degrees(rtheta) % 360
        lines = [
            "Pos  x={:+.2f} m  y={:+.2f} m    Heading {:.1f} deg".format(
                rx, ry, heading_deg),
            "State: {:16s}  Coverage: {:.1f}%    Zoom: {}x".format(
                state_str, grid.coverage_pct(), self._zoom),
            "Q=quit   S=save map   R=clear trail   +/- = zoom",
        ]
        for i, txt in enumerate(lines):
            cv2.putText(canvas, txt,
                        (12, MAP_DISPLAY_PX + 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, _C_HUD_TXT, 1)

        cv2.imshow("Roomba Map", canvas)

        # ── Camera feed ───────────────────────────────────────────────────
        if cam_frame is not None:
            cv2.imshow("Camera Feed", cam_frame)

        # ── Key handling ──────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            return False
        if key in (ord('s'), ord('S')):
            self._save(grid)
        if key in (ord('r'), ord('R')):
            self._trail.clear()
        if key in (ord('+'), ord('=')):
            self._zoom = min(self._zoom + 1, 8)
        if key in (ord('-'), ord('_')):
            self._zoom = max(self._zoom - 1, 1)

        return True

    def save_final_map(self, grid, filename=None):
        self._save(grid, filename)

    def quit(self):
        cv2.destroyAllWindows()

    # ── Scan fan ──────────────────────────────────────────────────────────────

    def _draw_scan_fan(self, img, grid, robot_gx, robot_gy, rtheta):
        """
        Cast rays in a full 360° sweep from the robot's grid cell.
        Draw the ray in muted cyan up to the last free cell, then a bright
        dot at the first occupied cell (or range limit).
        """
        log = grid._log

        for i in range(_SCAN_RAYS):
            angle = rtheta + (2 * math.pi * i / _SCAN_RAYS)
            cos_a = math.cos(angle)
            sin_a = -math.sin(angle)   # screen y is flipped

            prev_gx, prev_gy = robot_gx, robot_gy

            for step in range(1, _SCAN_RANGE_CELLS + 1):
                gx = int(robot_gx + step * cos_a)
                gy = int(robot_gy + step * sin_a)

                if not (0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE):
                    break

                lo = float(log[gy, gx])

                if lo > 0.5:   # occupied — draw hit dot and stop
                    img[gy, gx] = _C_SCAN_HIT
                    break

                if lo < -0.1:  # confirmed free — tint ray pixel toward scan colour
                    img[gy, gx] = (
                        int(img[gy, gx, 0] * 0.65 + _C_SCAN_RAY[0] * 0.35),
                        int(img[gy, gx, 1] * 0.65 + _C_SCAN_RAY[1] * 0.35),
                        int(img[gy, gx, 2] * 0.65 + _C_SCAN_RAY[2] * 0.35),
                    )

                prev_gx, prev_gy = gx, gy

        return img

    # ── Save ──────────────────────────────────────────────────────────────────

    def _save(self, grid, filename=None):
        prob = grid.probability()
        img  = np.full((GRID_SIZE, GRID_SIZE, 3), _C_UNKNOWN, dtype=np.uint8)
        img[prob < grid.THRESH_FREE] = _C_FREE
        img[prob > grid.THRESH_OCC]  = _C_OCC
        if filename is None:
            filename = "map_{}.png".format(int(time.time()))
        cv2.imwrite(filename, img)
        print("[viz] Map saved →", filename)


