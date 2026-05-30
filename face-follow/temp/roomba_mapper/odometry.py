"""
odometry.py – Differential-drive odometry from Roomba wheel encoders.

Coordinate convention (standard math / ROS REP-103):
  x     – forward  (robot's initial heading)
  y     – left
  theta – yaw CCW from x-axis, radians

The robot starts at (0, 0) facing "north" (theta = π/2) so the display
has forward pointing upward.
"""
import math
from config import MM_PER_COUNT, WHEELBASE_MM


class Odometry:

    _ENC_MAX = 65535      # 16-bit unsigned rollover value
    _ENC_HALF = 32767     # used for signed-delta rollover detection

    def __init__(self, initial_theta: float = math.pi / 2):
        self.x     = 0.0
        self.y     = 0.0
        self.theta = initial_theta

        self._prev_left:  int | None = None
        self._prev_right: int | None = None

    # ── Public interface ──────────────────────────────────────────────────

    def update(self, left_enc: int, right_enc: int) -> tuple[float, float]:
        """
        Integrate new encoder counts and update pose.
        Returns (dc, dtheta): linear displacement (m) and rotation (rad)
        since the last call.
        """
        if self._prev_left is None:
            self._prev_left  = left_enc
            self._prev_right = right_enc
            return 0.0, 0.0

        dl_counts = self._signed_delta(left_enc,  self._prev_left)
        dr_counts = self._signed_delta(right_enc, self._prev_right)
        self._prev_left  = left_enc
        self._prev_right = right_enc

        dl = dl_counts * MM_PER_COUNT / 1000.0   # metres
        dr = dr_counts * MM_PER_COUNT / 1000.0

        dc     = (dl + dr) / 2.0
        dtheta = (dr - dl) / (WHEELBASE_MM / 1000.0)

        if abs(dtheta) < 1e-9:
            # Straight line – avoid division by near-zero radius
            self.x += dc * math.cos(self.theta)
            self.y += dc * math.sin(self.theta)
        else:
            # Arc of a circle
            r = dc / dtheta
            self.x += r * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += r * (math.cos(self.theta)           - math.cos(self.theta + dtheta))
            self.theta += dtheta

        # Keep theta in (-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        return dc, dtheta

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta

    # ── Helpers ───────────────────────────────────────────────────────────

    def _signed_delta(self, curr: int, prev: int) -> int:
        """Compute signed encoder delta accounting for 16-bit rollover."""
        d = curr - prev
        if d >  self._ENC_HALF:
            d -= self._ENC_MAX + 1
        elif d < -self._ENC_HALF:
            d += self._ENC_MAX + 1
        return d
