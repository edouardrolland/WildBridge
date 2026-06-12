"""
WildBridge - Test Flight + Live Map (single launcher)
=====================================================

Runs the behaviour test flight (test_flight.py) AND shows the live map
(waypoint_gui.py) at the same time, from one script:

  * the map (main thread) shows the drone's live position + heading, the
    operator keep-out circle, and the waypoint/path the test is currently
    flying to (the path turns red if it enters the keep-out zone),
  * the test flight sequence runs in a background thread; its per-phase
    confirmation prompts appear in THIS TERMINAL,
  * STOP and RTH buttons on the map are live the whole time as emergency
    controls.

      >>> Edit IP_RC and DRONE below for your setup. <<<

Run
---
    python3 run_test_with_map.py                 # uses the constants below
    python3 run_test_with_map.py 10.0.0.5        # override IP
    python3 run_test_with_map.py --drone M350    # override drone
    python3 run_test_with_map.py --auto          # skip terminal confirmations

License: MIT
"""

import argparse
import os
import sys
import threading
import time

# Make GroundStation/Python importable and reuse the sibling modules.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PY_DIR = os.path.normpath(os.path.join(_HERE, "..", "GroundStation", "Python"))
if _PY_DIR not in sys.path:
    sys.path.insert(0, _PY_DIR)

from djiInterface import DJIInterface  # noqa: E402
import test_flight as tf  # noqa: E402
from test_flight import DEFAULT_OPERATOR_LATLON  # noqa: E402
from satellite_map_gui import SatelliteMapGUI  # noqa: E402

# ===========================================================================
#  EDIT THESE FOR YOUR FLIGHT
# ===========================================================================
IP_RC = "192.168.0.116"   # RC / app IP address
DRONE = "M300"            # one of: M300, M350, M400, MAVIC3, MINI4
# ===========================================================================

# Map / safety defaults (operator defaults to Nenana, Alaska via waypoint_gui).
BUFFER_M = 15.0           # keep-out radius around the operator (m)
MAX_RANGE_M = 100.0        # map extent / reference range ring (m)


class RecordingDJIInterface(DJIInterface):
    """DJIInterface that remembers the last horizontal waypoint commanded, so
    the live map can display where the test flight is currently sending the
    drone. Pure passthrough otherwise."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._last_wp = None  # (lat, lon) or None

    def requestSendGoToWPwithPID(self, latitude, longitude, altitude, yaw, speed: float = 5.0):
        self._last_wp = (latitude, longitude)
        return super().requestSendGoToWPwithPID(latitude, longitude, altitude, yaw, speed)

    def requestSendRTH(self):
        self._last_wp = None  # no longer heading to a waypoint
        return super().requestSendRTH()


def main():
    parser = argparse.ArgumentParser(description="WildBridge test flight + live map")
    parser.add_argument("ip", nargs="?", default=IP_RC,
                        help=f"RC/app IP (default: {IP_RC})")
    parser.add_argument("--drone", default=DRONE, choices=sorted(tf.DRONE_PROFILES.keys()),
                        help=f"connected aircraft (default: {DRONE})")
    parser.add_argument("--auto", action="store_true",
                        help="skip per-phase confirmation prompts (use with extreme care)")
    parser.add_argument("--alt", type=float, default=tf.DEFAULT_TEST_ALT_AGL,
                        help="test altitude AGL in metres")
    parser.add_argument("--leg", type=float, default=tf.DEFAULT_LEG_DISTANCE,
                        help="waypoint leg distance in metres")
    parser.add_argument("--bearing", type=float, default=0.0,
                        help="INITIAL leg bearing in degrees (0=N, 90=E); adjust live with the UI slider")
    parser.add_argument("--buffer", type=float, default=BUFFER_M,
                        help="keep-out radius around operator, metres")
    parser.add_argument("--max-range", type=float, default=MAX_RANGE_M,
                        help="map extent / reference range ring, metres")
    args = parser.parse_args()

    profile = tf.DRONE_PROFILES[args.drone]

    print("=" * 64)
    print("WildBridge Test Flight + Live Map")
    print("=" * 64)
    print(f"  IP address       : {args.ip}")
    print(f"  Aircraft profile : {args.drone} ({profile['name']})")
    print(f"  Waypoint leg     : {args.leg} m on bearing {args.bearing:.0f} deg")
    print(f"  Operator (keep-out): {DEFAULT_OPERATOR_LATLON} ± {args.buffer} m")
    print(f"  Mode             : {'AUTO (no prompts)' if args.auto else 'interactive (answer prompts here)'}")
    print("=" * 64)

    dji = RecordingDJIInterface(args.ip)
    if dji.IP_RC == "":
        print("ERROR: no drone IP. Edit IP_RC or pass one on the command line.")
        return 2

    dji.startTelemetryStream()
    print(f"Connecting to {dji.IP_RC}, warming up telemetry...")
    time.sleep(tf.TELEMETRY_WARMUP_S)
    if not dji.getTelemetry():
        print("WARNING: no telemetry yet. Opening the map anyway; it will populate "
              "once data arrives. (Check the app is running and reachable.)")

    # Build the map first so the test flight can read its bearing slider.
    gui = SatelliteMapGUI(dji, profile, operator_latlon=DEFAULT_OPERATOR_LATLON,
                          buffer_m=args.buffer, max_range_m=args.max_range,
                          monitor=True, wp_source=lambda: dji._last_wp,
                          plan_leg_m=args.leg, plan_bearing=args.bearing)

    # Test flight runs in the background; the map owns the main thread. Phase 3
    # reads the live slider value via gui.get_bearing when it flies the leg.
    rec = tf.TestRecorder()
    flight_thread = threading.Thread(
        target=tf.run_test_flight,
        args=(dji, profile, args.alt, args.leg, args.auto, rec),
        kwargs={"bearing": gui.get_bearing},
        daemon=True)
    flight_thread.start()
    try:
        gui.run()  # blocks until the map window is closed
    finally:
        flight_thread.join(timeout=2.0)
        rec.report()
        dji.stopTelemetryStream()
    return 0


if __name__ == "__main__":
    sys.exit(main())
