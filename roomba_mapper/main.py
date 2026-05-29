"""
main.py – Roomba R600 autonomous room mapper.

Usage
-----
  python main.py

Before running:
  1. Flash esp32/roomba_controller.ino to the Freenove Wroover.
  2. Edit roomba_mapper/config.py → set ESP32_IP to the IP printed in the
     Arduino Serial Monitor.
  3. Install dependencies:  pip install -r requirements.txt

Map window key-bindings:
  Q / Esc  – stop robot and quit
  S        – save current map snapshot
  R        – clear the path trail
"""
import math
import signal
import sys
import time

# Ensure imports resolve regardless of working directory
import os
sys.path.insert(0, os.path.dirname(__file__))

from config  import (ESP32_IP, ESP32_PORT, VISION_ENABLED,
                     GRID_RESOLUTION_M, VIZ_FPS_TARGET,
                     SPEED_FORWARD)
from comm     import RoombaComm
from odometry import Odometry
from grid     import OccupancyGrid
from vision   import CameraDetector
from explorer import ExplorationFSM
from viz      import MapVisualizer


# ── Constants ─────────────────────────────────────────────────────────────

LOOP_HZ        = 20
LOOP_PERIOD    = 1.0 / LOOP_HZ

# How far ahead (m) to raycast "free" when no obstacle is detected by camera
FREE_RAYCAST_M = 1.2

# Roomba body radius (m) – cells within this radius are always free
ROBOT_RADIUS_M = 0.175


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("  Roomba Room Mapper – Exploration Mode")
    print("=" * 55)
    print(f"  Connecting to ESP32 at {ESP32_IP}:{ESP32_PORT} …")

    comm = RoombaComm(ESP32_IP, ESP32_PORT)
    retry = 0
    while not comm.connect():
        retry += 1
        if retry > 10:
            sys.exit("Could not connect to ESP32 after 10 tries. "
                     "Check config.py → ESP32_IP and verify the device is on.")
        print(f"  Retry {retry}/10 in 2 s …")
        time.sleep(2)

    odom     = Odometry()
    grid     = OccupancyGrid()
    explorer = ExplorationFSM(grid)
    viz      = MapVisualizer()

    vision: CameraDetector | None = None
    if VISION_ENABLED:
        try:
            vision = CameraDetector()
            print("  Camera initialised ✓")
        except Exception as exc:
            print(f"  Camera failed ({exc}) – running encoder-only mode")

    running = True

    def _shutdown(sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("\n  Exploration started. Press Q in the map window or Ctrl-C to stop.\n")

    prev_drive     = (0, 0)
    frame_dt       = 1.0 / VIZ_FPS_TARGET
    last_viz_time  = 0.0

    try:
        while running:
            t0 = time.monotonic()

            # ── 1. Read sensors ──────────────────────────────────────────
            sensor = comm.read_sensors()

            bump_right = bump_left = cliff = False
            rx, ry, rtheta = odom.pose

            if sensor:
                dc, _dtheta = odom.update(sensor["left_enc"],
                                           sensor["right_enc"])
                rx, ry, rtheta = odom.pose

                bd         = sensor["bumps_drops"]
                bump_right = bool(bd & 0x01)
                bump_left  = bool(bd & 0x02)
                cliff      = bool(sensor["cliff_fl"] or sensor["cliff_fr"]
                                  or sensor["cliff_l"] or sensor["cliff_r"])

                # Mark robot footprint as free
                grid.mark_free(rx, ry)

                # Mark cells just behind robot as free (confirms we passed through)
                if abs(dc) > 5e-4:
                    back_x = rx - dc * math.cos(rtheta)
                    back_y = ry - dc * math.sin(rtheta)
                    grid.mark_free(back_x, back_y)

                # Bump → obstacle immediately ahead
                if bump_right or bump_left:
                    obs_x = rx + (ROBOT_RADIUS_M + 0.05) * math.cos(rtheta)
                    obs_y = ry + (ROBOT_RADIUS_M + 0.05) * math.sin(rtheta)
                    grid.mark_occupied(obs_x, obs_y)
                    # Side bias
                    if bump_right:
                        ang = rtheta - math.pi / 6
                        grid.mark_occupied(rx + 0.20 * math.cos(ang),
                                           ry + 0.20 * math.sin(ang))
                    if bump_left:
                        ang = rtheta + math.pi / 6
                        grid.mark_occupied(rx + 0.20 * math.cos(ang),
                                           ry + 0.20 * math.sin(ang))

            # ── 2. Vision ─────────────────────────────────────────────────
            cam_obstacle = False
            cam_dist     = None
            cam_frame    = None

            if vision:
                cam_obstacle, cam_dist = vision.update()
                cam_frame = vision.get_display_frame()

                if cam_obstacle and cam_dist is not None:
                    obs_x = rx + cam_dist * math.cos(rtheta)
                    obs_y = ry + cam_dist * math.sin(rtheta)
                    grid.raycast(rx, ry, obs_x, obs_y, hit=True)
                else:
                    # Nothing detected → cast a free ray straight ahead
                    fx = rx + FREE_RAYCAST_M * math.cos(rtheta)
                    fy = ry + FREE_RAYCAST_M * math.sin(rtheta)
                    grid.raycast(rx, ry, fx, fy, hit=False)

            # ── 3. Exploration decision ───────────────────────────────────
            left_spd, right_spd = explorer.update(
                rx, ry, rtheta,
                bump_right, bump_left, cliff,
                cam_obstacle, cam_dist,
            )

            if (left_spd, right_spd) != prev_drive:
                comm.send_drive(left_spd, right_spd)
                prev_drive = (left_spd, right_spd)

            # ── 4. Visualise (throttled) ──────────────────────────────────
            now = time.monotonic()
            if now - last_viz_time >= frame_dt:
                last_viz_time = now
                if not viz.update(grid, rx, ry, rtheta,
                                  explorer.state, cam_frame):
                    running = False

            # ── 5. Reconnect if needed ────────────────────────────────────
            if not comm.connected:
                print("[main] Lost connection – reconnecting …")
                comm.connect()

            # ── 6. Sleep to hold loop rate ────────────────────────────────
            elapsed = time.monotonic() - t0
            wait    = LOOP_PERIOD - elapsed
            if wait > 0:
                time.sleep(wait)

    finally:
        print("\n[main] Stopping …")
        comm.send_stop()
        time.sleep(0.2)
        comm.disconnect()

        if vision:
            vision.release()

        rx, ry, rtheta = odom.pose
        print(f"[main] Final pose: ({rx:.3f}, {ry:.3f}) m  "
              f"heading {math.degrees(rtheta):.1f}°")
        print(f"[main] Coverage:   {grid.coverage_pct():.1f}%")

        viz.save_final_map(grid)
        viz.quit()
        print("[main] Map saved. Done.")


if __name__ == "__main__":
    main()
