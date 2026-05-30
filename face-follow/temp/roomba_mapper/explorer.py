"""
explorer.py – BFS frontier exploration with waypoint path following.

Algorithm
---------
Every REPLAN_INTERVAL ticks (or when the waypoint list runs out):

  1. BFS #1 – Frontier search
     Expand outward from the robot through FREE cells only.
     The first UNKNOWN cell encountered is the nearest reachable frontier.

  2. BFS #2 – Path planning
     Find the shortest grid path from the robot to that frontier,
     allowing travel through both FREE and UNKNOWN cells
     (unknown = uncharted but not confirmed blocked).

  3. Decimate the path to waypoints spaced ~0.5 m apart.

  4. Follow waypoints with proportional heading control.
     Bumps / cliffs trigger backup → rotate → replan.
     "Stuck" detection (< 5 cm in 3 s) triggers an escape rotation
     toward the side with the most unknown space.
"""

from collections import deque
import math
import random

from config import (SPEED_FORWARD, SPEED_ROTATE, SPEED_BACKUP,
                    BACKUP_TICKS, ROTATE_TICKS_MIN, ROTATE_TICKS_MAX,
                    STUCK_TICKS, GRID_SIZE, GRID_RESOLUTION_M)

# ── Tuning ────────────────────────────────────────────────────────────────────
REPLAN_INTERVAL  = 60     # ticks between replans while navigating  (3 s @ 20 Hz)
REPLAN_RETRY     = 20     # ticks before retrying after a failed BFS (1 s)
WAYPOINT_TOL_M   = 0.28   # m  – waypoint "reached" radius
WAYPOINT_STEP    = 10     # grid cells between kept waypoints  (0.5 m @ 5 cm/cell)
BFS_LIMIT        = 25000  # max cells visited per BFS call (keeps latency < ~80 ms)


class ExplorationFSM:

    def __init__(self, grid):
        self._grid = grid
        self.state = "forward"

        # Recovery counters
        self._backup_left = 0
        self._rotate_left = 0
        self._rotate_dir  = 1

        # Navigation
        self._waypoints: list[tuple[float, float]] = []
        self._replan_cd  = 0        # countdown: 0 means "replan now"

        # Stuck detection
        self._stuck_ticks = 0
        self._last_x = 0.0
        self._last_y = 0.0

    # ── Main update (called at 20 Hz) ─────────────────────────────────────────

    def update(self,
               rx: float, ry: float, rtheta: float,
               bump_right: bool, bump_left: bool,
               cliff: bool,
               cam_obstacle: bool,
               cam_dist) -> tuple[int, int]:

        bump = bump_right or bump_left

        # ── BACKUP ────────────────────────────────────────────────────────────
        if self.state == "backup":
            self._backup_left -= 1
            if self._backup_left <= 0:
                self._rotate_dir  = -1 if bump_right else 1
                self._rotate_left = random.randint(ROTATE_TICKS_MIN, ROTATE_TICKS_MAX)
                self._waypoints   = []
                self._replan_cd   = 0   # replan as soon as rotation done
                self.state = "rotate"
            return (-SPEED_BACKUP, -SPEED_BACKUP)

        # ── ROTATE ────────────────────────────────────────────────────────────
        if self.state == "rotate":
            self._rotate_left -= 1
            if self._rotate_left <= 0:
                self.state = "forward"
            spd = SPEED_ROTATE * self._rotate_dir
            return (-spd, spd)

        # ── Triggers: bump / cliff / camera ───────────────────────────────────
        if bump or cliff:
            self.state = "backup"
            self._backup_left = BACKUP_TICKS
            return (-SPEED_BACKUP, -SPEED_BACKUP)

        if cam_obstacle and cam_dist is not None and cam_dist < 0.25:
            self.state = "backup"
            self._backup_left = max(BACKUP_TICKS // 2, 3)
            return (-SPEED_BACKUP, -SPEED_BACKUP)

        # ── Stuck detection ───────────────────────────────────────────────────
        self._stuck_ticks += 1
        if self._stuck_ticks >= STUCK_TICKS:
            self._stuck_ticks = 0
            dist_moved = math.hypot(rx - self._last_x, ry - self._last_y)
            if dist_moved < 0.05:
                # Rotate toward the side with the most unexplored space
                self._rotate_dir  = self._best_escape_dir(rx, ry, rtheta)
                self._rotate_left = ROTATE_TICKS_MAX
                self._waypoints   = []
                self._replan_cd   = 0
                self.state = "rotate"
                return (-SPEED_ROTATE * self._rotate_dir,
                         SPEED_ROTATE * self._rotate_dir)
            self._last_x, self._last_y = rx, ry

        # ── Replanning ────────────────────────────────────────────────────────
        if self._replan_cd <= 0:
            success = self._plan(rx, ry)
            self._replan_cd = REPLAN_INTERVAL if success else REPLAN_RETRY

        self._replan_cd -= 1

        # ── Consume reached waypoints ─────────────────────────────────────────
        while self._waypoints:
            tx, ty = self._waypoints[0]
            if math.hypot(tx - rx, ty - ry) < WAYPOINT_TOL_M:
                self._waypoints.pop(0)
            else:
                break

        # If we just finished the last waypoint, replan immediately next tick
        if not self._waypoints:
            self._replan_cd = 0

        # ── Steer toward next waypoint ────────────────────────────────────────
        if self._waypoints:
            return self._steer_to(rx, ry, rtheta, self._waypoints[0])

        # No path at all – creep forward to expose new cells for future BFS
        return (SPEED_FORWARD, SPEED_FORWARD)

    # ── BFS planning ─────────────────────────────────────────────────────────

    def _plan(self, rx: float, ry: float) -> bool:
        """
        Run BFS #1 (frontier search) then BFS #2 (path planning).
        Populates self._waypoints. Returns True on success.
        """
        grid   = self._grid
        prob   = grid.probability()   # computed once, reused by both BFS calls
        sgx, sgy = grid.world_to_grid(rx, ry)

        # ── BFS #1: find nearest reachable frontier ───────────────────────────
        frontier = self._bfs_frontier(sgx, sgy, prob, grid)
        if frontier is None:
            return False

        tgx, tgy = frontier

        # ── BFS #2: plan path to frontier ─────────────────────────────────────
        path = self._bfs_path(sgx, sgy, tgx, tgy, prob, grid)
        if not path:
            return False

        # ── Decimate to waypoints ─────────────────────────────────────────────
        wps = []
        for i in range(WAYPOINT_STEP, len(path), WAYPOINT_STEP):
            gx, gy = path[i]
            wps.append(grid.grid_to_world(gx, gy))

        # Always include the frontier cell itself as the final waypoint
        gx, gy = path[-1]
        last_wp = grid.grid_to_world(gx, gy)
        if not wps or wps[-1] != last_wp:
            wps.append(last_wp)

        self._waypoints = wps
        return True

    def _bfs_frontier(self, sgx, sgy, prob, grid) -> tuple[int, int] | None:
        """
        BFS #1 – expands through FREE cells from (sgx, sgy).
        Returns the first UNKNOWN cell encountered (= nearest frontier).
        Does NOT cross occupied or unknown cells while searching.
        """
        TF = grid.THRESH_FREE
        TO = grid.THRESH_OCC

        visited = set()
        visited.add((sgx, sgy))
        queue = deque([(sgx, sgy)])
        count = 0

        while queue and count < BFS_LIMIT:
            count += 1
            gx, gy = queue.popleft()

            for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                nx, ny = gx + dx, gy + dy
                if (nx, ny) in visited:
                    continue
                if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                    continue
                visited.add((nx, ny))

                p = float(prob[ny, nx])
                if p > TO:       # confirmed wall – don't enter
                    continue
                if p >= TF:      # unknown cell adjacent to free = frontier
                    return (nx, ny)
                # free – keep expanding
                queue.append((nx, ny))

        return None

    def _bfs_path(self, sgx, sgy, tgx, tgy, prob, grid) -> list[tuple[int, int]]:
        """
        BFS #2 – finds the shortest grid path from start to target.
        Travels through FREE and UNKNOWN cells; avoids OCCUPIED cells.
        Returns list of (gx, gy) from start to target, or [].
        """
        TO = grid.THRESH_OCC

        parent: dict[tuple[int,int], tuple[int,int] | None] = {(sgx, sgy): None}
        queue  = deque([(sgx, sgy)])
        count  = 0

        while queue and count < BFS_LIMIT:
            count += 1
            gx, gy = queue.popleft()

            if gx == tgx and gy == tgy:
                # Reconstruct path
                path, cur = [], (tgx, tgy)
                while cur is not None:
                    path.append(cur)
                    cur = parent[cur]
                path.reverse()
                return path

            for dx, dy in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
                nx, ny = gx + dx, gy + dy
                if (nx, ny) in parent:
                    continue
                if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                    continue
                if float(prob[ny, nx]) > TO:
                    continue
                parent[(nx, ny)] = (gx, gy)
                queue.append((nx, ny))

        return []

    # ── Navigation helpers ────────────────────────────────────────────────────

    def _steer_to(self, rx, ry, rtheta, target) -> tuple[int, int]:
        tx, ty  = target
        bearing = math.atan2(ty - ry, tx - rx)
        err     = bearing - rtheta
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        # Proportional steer; reduce forward speed on large heading errors
        k     = SPEED_ROTATE / (math.pi / 2)
        steer = max(-SPEED_ROTATE, min(SPEED_ROTATE, k * err))
        fwd   = int(SPEED_FORWARD * max(0.35, 1.0 - abs(err) / math.pi))

        left  = max(-500, min(500, fwd - int(steer)))
        right = max(-500, min(500, fwd + int(steer)))
        return left, right

    def _best_escape_dir(self, rx, ry, rtheta) -> int:
        """
        Return +1 (left) or -1 (right): whichever side has more unknown space
        in a 1.5 m side-sweep, so recovery rotations expose new territory.
        """
        grid = self._grid
        prob = grid.probability()
        TF, TO = grid.THRESH_FREE, grid.THRESH_OCC

        scores = {1: 0, -1: 0}
        for side in (1, -1):
            angle = rtheta + side * math.pi / 2
            for d in range(1, 30):
                wx = rx + d * GRID_RESOLUTION_M * math.cos(angle)
                wy = ry + d * GRID_RESOLUTION_M * math.sin(angle)
                gx, gy = grid.world_to_grid(wx, wy)
                if not (0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE):
                    break
                p = float(prob[gy, gx])
                if TF <= p <= TO:
                    scores[side] += 1
                elif p > TO:
                    break

        return max(scores, key=scores.get)
