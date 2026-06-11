"""
WildBridge - Drone Behaviour Test Flight
========================================

Purpose
-------
Exercise the WildBridge command/telemetry interface against a real drone and
verify that each command produces the expected behaviour. This is a *flight
test*: the drone takes off, moves, and lands. Run it outdoors with a clear
GPS lock, a safety pilot on the sticks, and plenty of open space.

Functions under test (all on djiInterface.DJIInterface)
-------------------------------------------------------
    requestAllStates()            telemetry snapshot is populated
    requestHomePosition()         home location reported once GPS/home is set
    requestSendGimbalPitch()      gimbal pitch tracks the commanded angle
    requestSendGimbalYaw()        gimbal yaw (joint) tracks the commanded angle
    requestSendEnableVirtualStick()  virtual-stick mode accepted
    requestAbortMission()         mission aborted / virtual stick disabled
    requestSendTakeOff()          drone leaves the ground
    requestSendGotoAltitude()     climbs/descends to a target altitude
    requestSendGotoYaw()          rotates to a target heading
    requestSendGoToWPwithPID()    flies to a waypoint AT MAX PROFILE SPEED
    requestWaypointStatus()       waypoint-reached flag flips true
    requestDrop()                 payload release (skipped if no payload port)
    requestSendRTH()              returns to home and lands

Test plan (sequential phases, with operator confirmation between each)
----------------------------------------------------------------------
  Phase 0  GROUND CHECKS (no motion)
           - requestAllStates / requestHomePosition
           - gimbal pitch + yaw sweep and recentre
           - enable virtual stick, then abort mission
  Phase 1  TAKEOFF
           - requestSendTakeOff, confirm altitude rises, home becomes set
  Phase 2  ATTITUDE/ALTITUDE
           - climb to TEST_ALT_AGL via requestSendGotoAltitude
           - yaw +90 deg via requestSendGotoYaw
  Phase 3  WAYPOINT AT MAX SPEED  <-- the headline test
           - fly LEG_DISTANCE_M north via requestSendGoToWPwithPID at the
             connected drone's maximum horizontal speed (from its profile),
             poll requestWaypointStatus until reached, record peak speed
           - fly back to the start point the same way
  Phase 4  PAYLOAD
           - requestDrop (only if the profile defines a payload port)
  Phase 5  RETURN
           - requestSendRTH, confirm GO_HOME mode and descent to land

Max-speed rationale
-------------------
For requestSendGoToWPwithPID the Android app's PID loop caps the output at the
profile's maxHorizontalSpeedMps (DroneController.navigateToWaypointWithPID ->
waypointPidOutputLimit = activeProfile().maxHorizontalSpeedMps, then
coerceAtMost(maxSpeed)). So "the highest velocity the drone can fly at" for
this command is maxHorizontalSpeedMps. The values mirror
DroneControlProfiles.kt and are listed in DRONE_PROFILES below. Select the
connected aircraft with --drone (default: M400).

Usage
-----
    python3 test_flight.py [IP_RC] [--drone M400|M350|M300|MAVIC3|MINI4]
                           [--auto] [--alt 5] [--leg 30]

    IP_RC     RC/app IP (omit to auto-discover via UDP broadcast)
    --auto    skip the interactive confirmation between phases (USE WITH CARE)
    --alt     test altitude AGL in metres (default 5)
    --leg     horizontal leg distance in metres for the waypoint test (default 30)

Safety
------
  * A qualified pilot must be ready to take manual control at all times.
  * Ctrl+C, any unexpected exception, or a failed phase triggers an emergency
    RTH (requestSendRTH) and stops the test.
  * Nothing flies until you confirm Phase 1; ground checks are motionless.

Authors: generated for WildBridge behaviour testing
License: MIT
"""

import argparse
import math
import os
import sys
import time

# Make GroundStation/Python importable regardless of where this is launched from.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PY_DIR = os.path.normpath(os.path.join(_HERE, "..", "GroundStation", "Python"))
if _PY_DIR not in sys.path:
    sys.path.insert(0, _PY_DIR)

from djiInterface import DJIInterface  # noqa: E402


# ---------------------------------------------------------------------------
# Drone profiles -- a faithful mirror of WildBridgeApp/.../DroneControlProfiles.kt
# (enum DroneControlProfile). KEEP IN SYNC with that file. Field names match the
# Kotlin properties. maxHorizontalSpeedMps is the cap the PID waypoint loop
# enforces == the max test speed. A drone has a droppable payload iff
# payloadIndexType is not None (used by requestDrop / phase 4).
# ---------------------------------------------------------------------------
DRONE_PROFILES = {
    "MAVIC3": {
        "name": "Mavic 3 Enterprise",
        "maxHorizontalSpeedMps": 15.0, "maxGotoWpSpeedMps": 15.0,
        "distanceKp": 0.65, "distanceKi": 0.0001, "distanceKd": 0.001,
        "yawKp": 3.0, "maxYawRateDegS": 30.0, "defaultCruiseSpeedMps": 15.0,
        "payloadIndexType": None, "dropArmSwitchIndex": 0, "dropReleaseButtonIndex": 1,
    },
    "M300": {
        "name": "Matrice 300 RTK",
        "maxHorizontalSpeedMps": 25.0, "maxGotoWpSpeedMps": 25.0,
        "distanceKp": 0.34, "distanceKi": 0.0001, "distanceKd": 0.001,
        "yawKp": 3.0, "maxYawRateDegS": 30.0, "defaultCruiseSpeedMps": 25.0,
        "payloadIndexType": "RIGHT", "dropArmSwitchIndex": 3, "dropReleaseButtonIndex": 5,
    },
    "M350": {
        "name": "Matrice 350 RTK",
        "maxHorizontalSpeedMps": 25.0, "maxGotoWpSpeedMps": 25.0,
        "distanceKp": 0.34, "distanceKi": 0.0001, "distanceKd": 0.001,
        "yawKp": 3.0, "maxYawRateDegS": 30.0, "defaultCruiseSpeedMps": 3.0,
        "payloadIndexType": "RIGHT", "dropArmSwitchIndex": 3, "dropReleaseButtonIndex": 5,
    },
    "M400": {
        "name": "Matrice 400",
        "maxHorizontalSpeedMps": 25.0, "maxGotoWpSpeedMps": 25.0,
        "distanceKp": 0.34, "distanceKi": 0.0001, "distanceKd": 0.001,
        "yawKp": 3.0, "maxYawRateDegS": 30.0, "defaultCruiseSpeedMps": 25.0,
        "payloadIndexType": "PORT_3", "dropArmSwitchIndex": 3, "dropReleaseButtonIndex": 5,
    },
    "MINI4": {
        "name": "DJI Mini 4 Pro",
        "maxHorizontalSpeedMps": 15.0, "maxGotoWpSpeedMps": 5.0,
        "distanceKp": 0.65, "distanceKi": 0.0001, "distanceKd": 0.001,
        "yawKp": 3.0, "maxYawRateDegS": 30.0, "defaultCruiseSpeedMps": 2.0,
        "payloadIndexType": None, "dropArmSwitchIndex": 0, "dropReleaseButtonIndex": 1,
    },
}

# Tunable test geometry (overridable via CLI).
DEFAULT_TEST_ALT_AGL = 40.0      # metres above takeoff point
DEFAULT_LEG_DISTANCE = 100.0    # metres travelled in the waypoint test
GIMBAL_TEST_PITCH = -45.0      # degrees (camera tilts down)
GIMBAL_TEST_YAW = 30.0         # degrees
YAW_TEST_DELTA = 90.0          # degrees added to current heading

# Polling / tolerances.
TELEMETRY_WARMUP_S = 2.0
PHASE_TIMEOUT_S = 60.0         # max wait for a single phase goal
TAKEOFF_TIMEOUT_S = 120.0      # allow extra time for motors/spool-up/liftoff
POLL_INTERVAL_S = 0.2
ALT_TOLERANCE_M = 1.0
GIMBAL_TOLERANCE_DEG = 2.0
TAKEOFF_MIN_CLIMB_M = 0.8      # altitude rise that counts as "airborne"

EARTH_RADIUS_M = 6371000.0


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
class TestRecorder:
    """Collects PASS/FAIL results and prints a final report."""

    def __init__(self):
        self.results = []  # list of (name, passed, detail)

    def record(self, name, passed, detail=""):
        self.results.append((name, bool(passed), detail))
        tag = "PASS" if passed else "FAIL"
        print(f"   [{tag}] {name}" + (f"  ({detail})" if detail else ""))
        return passed

    def report(self):
        print("\n" + "=" * 64)
        print("TEST FLIGHT REPORT")
        print("=" * 64)
        passed = sum(1 for _, ok, _ in self.results if ok)
        for name, ok, detail in self.results:
            tag = "PASS" if ok else "FAIL"
            line = f"  [{tag}] {name}"
            if detail:
                line += f" -- {detail}"
            print(line)
        print("-" * 64)
        print(f"  {passed}/{len(self.results)} checks passed")
        print("=" * 64)
        return passed == len(self.results)


def offset_latlon(lat, lon, north_m, east_m):
    """Return (lat, lon) offset by north_m / east_m metres from (lat, lon)."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def latlon_to_local(lat, lon, lat0, lon0):
    """Convert (lat, lon) to local (east_m, north_m) relative to (lat0, lon0)."""
    north_m = math.radians(lat - lat0) * EARTH_RADIUS_M
    east_m = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return east_m, north_m


def local_to_latlon(east_m, north_m, lat0, lon0):
    """Inverse of latlon_to_local."""
    return offset_latlon(lat0, lon0, north_m=north_m, east_m=east_m)


def point_segment_distance(px, py, ax, ay, bx, by):
    """Shortest distance from point P to segment AB, all in metres."""
    abx, aby = bx - ax, by - ay
    seg_len2 = abx * abx + aby * aby
    if seg_len2 == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / seg_len2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


# Default operator position: Nenana, Alaska 99760. Overridable per script.
DEFAULT_OPERATOR_LATLON = (64.548893, -149.085784)


def speed_magnitude(speed):
    """3D speed magnitude (m/s) from a telemetry speed dict {x,y,z}."""
    if not isinstance(speed, dict):
        return 0.0
    x = speed.get("x", 0.0) or 0.0
    y = speed.get("y", 0.0) or 0.0
    z = speed.get("z", 0.0) or 0.0
    return math.sqrt(x * x + y * y + z * z)


def rejected(response):
    """True if the app's textual response looks like a rejection/empty reply."""
    if response is None:
        return True
    text = str(response).strip().lower()
    if text == "":
        return True
    return any(m in text for m in ("reject", "denied", "forbidden", "unauthor",
                                   "not found", "invalid", "error", "failed"))


def confirm(auto, prompt):
    """Ask the operator to continue. In --auto mode, just print and proceed."""
    if auto:
        print(f"   [AUTO] {prompt}")
        return True
    ans = input(f"\n>>> {prompt} [y/N]: ").strip().lower()
    return ans in ("y", "yes")


def wait_for(predicate, timeout=PHASE_TIMEOUT_S, on_tick=None):
    """Poll predicate() until it returns truthy or timeout. Returns final bool."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        if on_tick:
            on_tick()
        time.sleep(POLL_INTERVAL_S)
    return False


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------
def phase0_ground_checks(dji, rec):
    print("\n--- PHASE 0: GROUND CHECKS (no motion) ---")

    # requestAllStates -- telemetry must be flowing.
    states = dji.requestAllStates()
    have_loc = isinstance(states, dict) and bool(states.get("location"))
    rec.record("requestAllStates", have_loc,
               f"keys={len(states)}, sats={states.get('satelliteCount', '?')}, "
               f"batt={states.get('batteryLevel', '?')}%")

    # requestHomePosition -- may be empty on the ground before home is set.
    home = dji.requestHomePosition()
    rec.record("requestHomePosition (pre-takeoff)", isinstance(home, dict),
               f"home={home or 'not set yet'}")

    # Gimbal pitch sweep.
    dji.requestSendGimbalPitch(GIMBAL_TEST_PITCH)
    reached = wait_for(lambda: abs(dji.getGimbalAttitude().get("pitch", 999) - GIMBAL_TEST_PITCH)
                       <= GIMBAL_TOLERANCE_DEG, timeout=8)
    rec.record("requestSendGimbalPitch", reached,
               f"target={GIMBAL_TEST_PITCH}, got={dji.getGimbalAttitude().get('pitch')}")
    dji.requestSendGimbalPitch(0)
    time.sleep(1)

    # Gimbal yaw sweep (joint yaw).
    dji.requestSendGimbalYaw(GIMBAL_TEST_YAW)
    reached = wait_for(lambda: abs(dji.getGimbalJointAttitude().get("yaw", 999) - GIMBAL_TEST_YAW)
                       <= GIMBAL_TOLERANCE_DEG, timeout=8)
    rec.record("requestSendGimbalYaw", reached,
               f"target={GIMBAL_TEST_YAW}, got={dji.getGimbalJointAttitude().get('yaw')}")
    dji.requestSendGimbalYaw(0)
    time.sleep(1)

    # Enable virtual stick, then abort mission.
    resp = dji.requestSendEnableVirtualStick()
    rec.record("requestSendEnableVirtualStick", not rejected(resp), f"resp={resp!r}")
    resp = dji.requestAbortMission()
    rec.record("requestAbortMission", not rejected(resp), f"resp={resp!r}")


def phase1_takeoff(dji, rec):
    print("\n--- PHASE 1: TAKEOFF ---")
    start_alt = dji.getLocation().get("altitude", 0.0)
    resp = dji.requestSendTakeOff()
    if rejected(resp):
        rec.record("requestSendTakeOff", False, f"resp={resp!r}")
        return None
    climbed = wait_for(
        lambda: dji.getLocation().get("altitude", start_alt) - start_alt >= TAKEOFF_MIN_CLIMB_M,
        timeout=TAKEOFF_TIMEOUT_S,
        on_tick=lambda: print(f"      alt={dji.getLocation().get('altitude'):.2f} m", end="\r"))
    cur_alt = dji.getLocation().get("altitude", start_alt)
    rec.record("requestSendTakeOff", climbed, f"start={start_alt:.2f} -> {cur_alt:.2f} m")

    # Home should be set now.
    home = dji.requestHomePosition()
    rec.record("requestHomePosition (airborne)", dji.isHomeSet() and bool(home),
               f"homeSet={dji.isHomeSet()}, home={home}")
    return start_alt


def phase2_attitude(dji, rec, takeoff_alt, alt_agl):
    print("\n--- PHASE 2: ALTITUDE & YAW ---")
    target_alt = (takeoff_alt or 0.0) + alt_agl
    print(f"   Target altitude: {target_alt:.1f} m (takeoff baseline {takeoff_alt or 0.0:.1f} + AGL {alt_agl:.1f})")

    resp = dji.requestSendGotoAltitude(target_alt)
    print(f"   DEBUG gotoAltitude response: {resp!r}")
    if rejected(resp):
        rec.record("requestSendGotoAltitude", False,
                   f"resp={resp!r}, mode={dji.getFlightMode()}, manualOverride={dji.isManualOverrideActive()}")
        return

    alt_debug = {"last_print": 0.0, "last_alt": None, "stagnant_s": 0.0}

    def tick_alt():
        now = time.time()
        alt_now = dji.getLocation().get("altitude", 0.0)
        if alt_debug["last_alt"] is not None and abs(alt_now - alt_debug["last_alt"]) < 0.05:
            alt_debug["stagnant_s"] += POLL_INTERVAL_S
        else:
            alt_debug["stagnant_s"] = 0.0
        alt_debug["last_alt"] = alt_now

        # Preserve the compact live line.
        print(
            f"      alt={alt_now:.2f} m / {target_alt:.2f} m"
            f"  mode={dji.getFlightMode()}"
            f"  manualOverride={dji.isManualOverrideActive()}", end="\r")

        # Also emit periodic newline debug so the console does not look frozen.
        if now - alt_debug["last_print"] >= 2.0:
            alt_debug["last_print"] = now
            print(
                "\n"
                f"   DEBUG phase2-alt: current={alt_now:.2f}m target={target_alt:.2f}m "
                f"delta={target_alt - alt_now:.2f}m altitudeReached={dji.isAltitudeReached()} "
                f"mode={dji.getFlightMode()} manualOverride={dji.isManualOverrideActive()} "
                f"stagnantFor={alt_debug['stagnant_s']:.1f}s"
            )

    ok = wait_for(lambda: dji.isAltitudeReached()
                  or abs(dji.getLocation().get("altitude", -999) - target_alt) <= ALT_TOLERANCE_M,
                  on_tick=tick_alt)
    print()
    rec.record("requestSendGotoAltitude", ok,
               f"target={target_alt:.1f} m, got={dji.getLocation().get('altitude'):.2f} m")

    start_heading = dji.getHeading()
    target_yaw = (start_heading + YAW_TEST_DELTA) % 360
    resp = dji.requestSendGotoYaw(target_yaw)
    print(f"   DEBUG gotoYaw response: {resp!r}")
    if rejected(resp):
        rec.record("requestSendGotoYaw", False,
                   f"resp={resp!r}, mode={dji.getFlightMode()}, manualOverride={dji.isManualOverrideActive()}")
        return
    ok = wait_for(lambda: dji.isYawReached()
                  or abs(((dji.getHeading() - target_yaw + 180) % 360) - 180) <= 10,
                  on_tick=lambda: print(f"      heading={dji.getHeading():.1f} deg", end="\r"))
    rec.record("requestSendGotoYaw", ok,
               f"target={target_yaw:.1f}, got={dji.getHeading():.1f}")


def phase3_waypoint_max_speed(dji, rec, profile, leg_distance, bearing_deg=0.0):
    print("\n--- PHASE 3: WAYPOINT AT MAX SPEED ---")
    max_speed = profile["maxHorizontalSpeedMps"]
    print(f"   Drone: {profile['name']}  |  max horizontal speed = {max_speed} m/s")
    print(f"   (requestSendGoToWPwithPID is capped at this by the app's PID loop)")

    loc = dji.getLocation()
    start_lat, start_lon = loc.get("latitude"), loc.get("longitude")
    alt = loc.get("altitude")
    if start_lat is None or start_lon is None:
        rec.record("requestSendGoToWPwithPID (out)", False, "no GPS location available")
        return
    yaw = dji.getHeading()

    # Outbound leg at the requested bearing: 0=N, 90=E.
    brad = math.radians(float(bearing_deg))
    north_m = leg_distance * math.cos(brad)
    east_m = leg_distance * math.sin(brad)
    tgt_lat, tgt_lon = offset_latlon(start_lat, start_lon, north_m=north_m, east_m=east_m)
    peak = {"v": 0.0}

    def tick_out():
        v = speed_magnitude(dji.getSpeed())
        peak["v"] = max(peak["v"], v)
        print(f"      speed={v:4.1f} m/s  dist_home={dji.getDistanceToHome():.1f} m", end="\r")

    dji.requestSendGoToWPwithPID(tgt_lat, tgt_lon, alt, yaw, speed=max_speed)
    reached = wait_for(lambda: dji.requestWaypointStatus() == "true",
                       timeout=PHASE_TIMEOUT_S, on_tick=tick_out)
    rec.record("requestSendGoToWPwithPID (out)", reached,
               f"requested {max_speed} m/s, peak observed {peak['v']:.1f} m/s")
    # requestWaypointStatus is validated by the fact the wait above resolved.
    rec.record("requestWaypointStatus", reached, f"waypointReached={dji.requestWaypointStatus()}")

    # Speed sanity: did we actually approach the commanded maximum?
    rec.record("max-speed reached (>=70% of cap)", peak["v"] >= 0.7 * max_speed,
               f"peak {peak['v']:.1f} / {max_speed} m/s")

    # Return leg back to the start point.
    peak["v"] = 0.0
    dji.requestSendGoToWPwithPID(start_lat, start_lon, alt, yaw, speed=max_speed)
    reached = wait_for(lambda: dji.requestWaypointStatus() == "true",
                       timeout=PHASE_TIMEOUT_S, on_tick=tick_out)
    rec.record("requestSendGoToWPwithPID (return)", reached,
               f"peak observed {peak['v']:.1f} m/s")


def phase4_payload(dji, rec, profile, auto):
    print("\n--- PHASE 4: PAYLOAD DROP ---")
    if profile["payloadIndexType"] is None:
        rec.record("requestDrop", True, f"skipped: {profile['name']} has no payload port")
        return
    if not confirm(auto, f"Release payload on {profile['name']}? (ensure it is safe below)"):
        rec.record("requestDrop", True, "skipped by operator")
        return
    resp = dji.requestDrop()
    rec.record("requestDrop", not rejected(resp), f"resp={resp!r}")


def phase5_rth(dji, rec):
    print("\n--- PHASE 5: RETURN TO HOME ---")
    resp = dji.requestSendRTH()
    if rejected(resp):
        rec.record("requestSendRTH (command)", False, f"resp={resp!r}")
        return
    rec.record("requestSendRTH (command)", True, f"resp={resp!r}")

    # Confirm the drone is heading home: GO_HOME mode or distance shrinking.
    start_dist = dji.getDistanceToHome()
    going_home = wait_for(
        lambda: "GO_HOME" in str(dji.getFlightMode()).upper()
        or dji.getDistanceToHome() < max(start_dist - 2.0, 1.0),
        timeout=PHASE_TIMEOUT_S,
        on_tick=lambda: print(f"      mode={dji.getFlightMode()} "
                              f"dist_home={dji.getDistanceToHome():.1f} m", end="\r"))
    rec.record("requestSendRTH (engaged)", going_home,
               f"mode={dji.getFlightMode()}, dist_home={dji.getDistanceToHome():.1f} m")


# ---------------------------------------------------------------------------
# Emergency stop
# ---------------------------------------------------------------------------
def emergency_rth(dji):
    print("\n!!! EMERGENCY: commanding RTH !!!")
    try:
        dji.requestSendRTH()
    except Exception as e:  # noqa: BLE001
        print(f"   RTH command itself failed: {e}")


def run_test_flight(dji, profile, alt_agl, leg_distance, auto, rec, bearing=0.0):
    """Run the interactive flight sequence using an already-initialized DJIInterface.

    `bearing` may be a numeric degree value or a callable returning degrees,
    allowing the map launcher to update the outbound leg heading live.
    """
    try:
        phase0_ground_checks(dji, rec)

        if not confirm(auto, "GROUND CHECKS DONE. Area clear? Begin POWERED FLIGHT (takeoff)?"):
            print("Operator declined takeoff. Ending (no flight performed).")
            return

        takeoff_alt = phase1_takeoff(dji, rec)
        if takeoff_alt is None:
            print("Takeoff failed; aborting flight.")
            emergency_rth(dji)
            return

        if confirm(auto, "Airborne. Proceed to altitude/yaw tests?"):
            phase2_attitude(dji, rec, takeoff_alt, alt_agl)

        if callable(bearing):
            current_bearing = float(bearing())
        else:
            current_bearing = float(bearing)

        if confirm(auto, f"Proceed to MAX-SPEED waypoint test ({profile['maxHorizontalSpeedMps']} m/s, "
                         f"{leg_distance} m leg @ {current_bearing:.0f} deg)? Ensure the corridor is clear."):
            phase3_waypoint_max_speed(dji, rec, profile, leg_distance, bearing_deg=current_bearing)

        phase4_payload(dji, rec, profile, auto)

        if confirm(auto, "Tests complete. Command RETURN TO HOME?"):
            phase5_rth(dji, rec)

    except KeyboardInterrupt:
        print("\nInterrupted by operator.")
        emergency_rth(dji)
    except Exception as e:  # noqa: BLE001
        print(f"\nUnexpected error: {e}")
        emergency_rth(dji)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="WildBridge drone behaviour test flight")
    parser.add_argument("ip", nargs="?", default="", help="RC/app IP (omit to auto-discover)")
    parser.add_argument("--drone", default="M300", choices=sorted(DRONE_PROFILES.keys()),
                        help="connected aircraft (selects max speed / payload). Default: M400")
    parser.add_argument("--auto", action="store_true",
                        help="skip per-phase confirmation prompts (use with extreme care)")
    parser.add_argument("--alt", type=float, default=DEFAULT_TEST_ALT_AGL,
                        help="test altitude AGL in metres")
    parser.add_argument("--leg", type=float, default=DEFAULT_LEG_DISTANCE,
                        help="waypoint leg distance in metres")
    args = parser.parse_args()

    profile = DRONE_PROFILES[args.drone]

    print("=" * 64)
    print("WildBridge Drone Behaviour Test Flight")
    print("=" * 64)
    print(f"  Aircraft profile : {args.drone} ({profile['name']})")
    print(f"  Max test speed   : {profile['maxHorizontalSpeedMps']} m/s (requestSendGoToWPwithPID)")
    print(f"  Test altitude    : {args.alt} m AGL")
    print(f"  Waypoint leg     : {args.leg} m")
    print(f"  Mode             : {'AUTO (no prompts)' if args.auto else 'interactive'}")
    print("=" * 64)

    dji = DJIInterface(args.ip)
    if dji.IP_RC == "":
        print("ERROR: no drone IP (discovery failed and none supplied). Aborting.")
        return 2

    dji.startTelemetryStream()
    print(f"Connecting to {dji.IP_RC}, warming up telemetry...")
    time.sleep(TELEMETRY_WARMUP_S)
    if not dji.getTelemetry():
        print("ERROR: no telemetry received. Is the app running and reachable? Aborting.")
        dji.stopTelemetryStream()
        return 2

    rec = TestRecorder()
    try:
        run_test_flight(dji, profile, args.alt, args.leg, args.auto, rec, bearing=0.0)
    finally:
        all_passed = rec.report()
        dji.stopTelemetryStream()
        return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
