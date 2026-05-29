"""
grid.py – Log-odds occupancy grid with raycasting and frontier detection.

World → grid coordinate mapping:
  gx = round(wx / res) + origin          (x right)
  gy = origin - round(wy / res)          (y up in world → row decreases upward)

This keeps north (positive y) at the top of the rendered image without any flip.
"""
import math
import numpy as np
from config import GRID_SIZE, GRID_RESOLUTION_M


class OccupancyGrid:

    # Log-odds update magnitudes
    _LO_OCC  =  0.85
    _LO_FREE = -0.40
    _LO_MAX  =  3.5
    _LO_MIN  = -2.0

    # Probability thresholds
    THRESH_FREE = 0.35
    THRESH_OCC  = 0.65

    def __init__(self):
        self.size   = GRID_SIZE
        self.res    = GRID_RESOLUTION_M
        self.origin = GRID_SIZE // 2          # robot starts at grid centre
        self._log   = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    # ── Coordinate transforms ─────────────────────────────────────────────

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        gx = int(round(wx / self.res)) + self.origin
        gy = self.origin - int(round(wy / self.res))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        wx = (gx - self.origin) * self.res
        wy = (self.origin - gy) * self.res
        return wx, wy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size

    # ── Single-cell updates ───────────────────────────────────────────────

    def mark_free(self, wx: float, wy: float):
        gx, gy = self.world_to_grid(wx, wy)
        if self._in_bounds(gx, gy):
            self._log[gy, gx] = max(self._log[gy, gx] + self._LO_FREE, self._LO_MIN)

    def mark_occupied(self, wx: float, wy: float):
        gx, gy = self.world_to_grid(wx, wy)
        if self._in_bounds(gx, gy):
            self._log[gy, gx] = min(self._log[gy, gx] + self._LO_OCC, self._LO_MAX)

    # ── Raycasting ────────────────────────────────────────────────────────

    def raycast(self, rx: float, ry: float,
                end_x: float, end_y: float,
                hit: bool = False):
        """
        Mark all cells along the ray from (rx,ry) to (end_x,end_y) as free.
        If hit=True, also mark the endpoint as occupied.
        """
        gx0, gy0 = self.world_to_grid(rx, ry)
        gx1, gy1 = self.world_to_grid(end_x, end_y)
        cells = self._bresenham(gx0, gy0, gx1, gy1)

        for gx, gy in cells[:-1]:
            if self._in_bounds(gx, gy):
                self._log[gy, gx] = max(self._log[gy, gx] + self._LO_FREE, self._LO_MIN)

        if cells:
            gx, gy = cells[-1]
            if self._in_bounds(gx, gy):
                if hit:
                    self._log[gy, gx] = min(self._log[gy, gx] + self._LO_OCC, self._LO_MAX)
                else:
                    self._log[gy, gx] = max(self._log[gy, gx] + self._LO_FREE, self._LO_MIN)

    @staticmethod
    def _bresenham(x0, y0, x1, y1) -> list[tuple[int, int]]:
        cells = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x += sx
            if e2 <  dx:
                err += dx; y += sy
        return cells

    # ── Queries ───────────────────────────────────────────────────────────

    def probability(self) -> np.ndarray:
        """Return probability map (float32, shape (size, size)), 0.5 = unknown."""
        return 1.0 / (1.0 + np.exp(-self._log))

    def is_occupied(self, wx: float, wy: float) -> bool:
        gx, gy = self.world_to_grid(wx, wy)
        if not self._in_bounds(gx, gy):
            return True
        return float(self._log[gy, gx]) > 0.0 and \
               (1.0 / (1.0 + math.exp(-float(self._log[gy, gx])))) > self.THRESH_OCC

    def is_free(self, wx: float, wy: float) -> bool:
        gx, gy = self.world_to_grid(wx, wy)
        if not self._in_bounds(gx, gy):
            return False
        return (1.0 / (1.0 + math.exp(-float(self._log[gy, gx])))) < self.THRESH_FREE

    # ── Frontier extraction ───────────────────────────────────────────────

    def get_frontiers(self, min_cluster: int = 4) -> list[tuple[float, float]]:
        """
        Return world-coordinate centroids of frontier clusters.
        A frontier cell is unknown and adjacent to at least one free cell.
        """
        from scipy.ndimage import binary_dilation, label

        prob = self.probability()
        free_mask    = prob < self.THRESH_FREE
        unknown_mask = (prob >= self.THRESH_FREE) & (prob <= self.THRESH_OCC)

        # Dilate free mask by 1 cell to find unknown cells touching free space
        frontier_mask = binary_dilation(free_mask, iterations=1) & unknown_mask

        labeled, n = label(frontier_mask)
        centroids = []
        for i in range(1, n + 1):
            pts = np.argwhere(labeled == i)   # shape (N, 2): [row, col]
            if len(pts) < min_cluster:
                continue
            cy, cx = pts.mean(axis=0)
            wx, wy = self.grid_to_world(int(cx), int(cy))
            centroids.append((wx, wy))

        return centroids

    # ── Coverage stat ─────────────────────────────────────────────────────

    def coverage_pct(self) -> float:
        prob    = self.probability()
        known   = np.sum((prob < self.THRESH_FREE) | (prob > self.THRESH_OCC))
        return 100.0 * float(known) / (self.size * self.size)
