#!/usr/bin/env python3
"""Send ONE waypoint via the XPRIZE PID-tuning endpoint and wait for arrival.

Uses requestSendGoToWPwithPIDXPRIZETuning, which overrides the distance-PID Kp gain
and the max horizontal acceleration limit per call (Ki/Kd and the yaw gains stay from
the active aircraft profile).

COLD START: those overrides only take effect when the waypoint loop starts cold. If a
loop is already running the bridge just hot-swaps the target and keeps the gains it
already captured. So this script calls requestAbortAll() first to cancel any running
loop (controlLoopEnabled=false, activeLoopIsWaypoint=false) — guaranteeing the new Kp /
accel are re-captured.

Arrival is tracked with the `arrived` flag, set from isWaypointReached(seq); seq ties the
telemetry "waypointReached" latch to THIS command so a stale latch can't read as reached.

All parameters are hardcoded below — edit the CONFIG block, then run:
    python test_go_to_wp.py
"""

import sys
import time

from djiInterface import DJIInterface

# ============================ CONFIG (edit me) ============================
RC_IP = ""            # RC IP; "" = auto-discover
TARGET_LAT = 55.4719  # target latitude (deg)
TARGET_LON = 10.3255  # target longitude (deg)
TARGET_ALT = 30.0     # target altitude (m)
TARGET_YAW = None     # target yaw (deg); None = use current heading
MAX_SPEED = 5.0       # max speed (m/s)
DISTANCE_KP = 0.5     # distance-PID Kp override
MAX_HORIZONTAL_ACCEL = 0.5  # max horizontal accel override (m/s^2)
# ==========================================================================

TELEMETRY_TIMEOUT = 15.0   # s to wait for first telemetry fix
ARRIVAL_TIMEOUT = 120.0    # s to wait for the drone to reach the waypoint
POLL_INTERVAL = 0.2        # s between arrival checks


def wait_for_fix(dji):
    """Block until telemetry has a usable location, or time out. Returns loc dict or None."""
    deadline = time.time() + TELEMETRY_TIMEOUT
    while time.time() < deadline:
        loc = dji.getLocation()
        if loc and loc.get("latitude") is not None and loc.get("longitude") is not None:
            return loc
        time.sleep(0.2)
    return None


def main():
    dji = DJIInterface(RC_IP)
    if dji.IP_RC == "":
        print("No drone IP. Set RC_IP in CONFIG or ensure discovery works.")
        return 1

    print(f"Connecting to {dji.IP_RC}...")
    dji.startTelemetryStream()
    try:
        print("Waiting for telemetry fix...")
        loc = wait_for_fix(dji)
        if loc is None:
            print("No telemetry fix received. Aborting.")
            return 1

        yaw = TARGET_YAW if TARGET_YAW is not None else dji.getHeading()

        print("--- Mission ---")
        print(f"  Target: lat={TARGET_LAT:.7f} lon={TARGET_LON:.7f} alt={TARGET_ALT:.1f} m  yaw={yaw:.1f} deg")
        print(f"  MaxSpeed={MAX_SPEED} m/s  distanceKp={DISTANCE_KP}  maxHorizontalAccel={MAX_HORIZONTAL_ACCEL} m/s^2")

        # Force a COLD start so the Kp / accel overrides are actually applied.
        print("Aborting any running loop to force a cold start...")
        dji.requestAbortAll()
        time.sleep(0.5)  # let the loop tear down before we re-enable the stick

        dji.requestSendEnableVirtualStick()
        seq = dji.requestSendGoToWPwithPIDXPRIZETuning(
            TARGET_LAT, TARGET_LON, TARGET_ALT, yaw, MAX_SPEED)
        if seq is None:
            print("Waypoint command rejected (no seq). Check manual override / arming.")
            return 1
        print(f"Waypoint accepted, seq={seq}. Flying...")

        # --- Arrival flag ---
        arrived = False
        deadline = time.time() + ARRIVAL_TIMEOUT
        t0 = time.time()
        while time.time() < deadline:
            if dji.isWaypointReached(seq):
                arrived = True
                print(f"\n*** ARRIVED at waypoint seq={seq} (t={time.time() - t0:.1f}s) ***")
                break
            print(f"\rFlying... t={time.time() - t0:5.1f}s  arrived=False", end="", flush=True)
            time.sleep(POLL_INTERVAL)

        if not arrived:
            print(f"\nTimed out after {ARRIVAL_TIMEOUT:.0f}s without reaching the waypoint. Aborting.")
            dji.requestAbortAll()
            return 1
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user. Aborting mission...")
        dji.requestAbortAll()
        return 1
    finally:
        dji.stopTelemetryStream()
        print("Telemetry stopped. Done.")


if __name__ == "__main__":
    sys.exit(main())
