"""
WildBridge - Waypoint GUI
=========================

A small map-style GUI to fly a single waypoint SAFELY and PREDICTABLY:

  * shows the live drone position (+ heading arrow) on a local East/North map,
  * shows the target waypoint and the PREDICTED STRAIGHT-LINE PATH the drone
    will take to it (the PID controller flies a straight line to the target),
  * lets you click on the map to move the waypoint,
  * draws a keep-out circle around the operator and BLOCKS the flight if the
    path would pass within that buffer (unless you explicitly override),
  * requires an explicit two-step confirmation (FLY -> CONFIRM) before it ever
    sends a command to the drone.

This directly answers "how do I know where it's going to fly / make sure it
doesn't fly over us": you see the exact path before committing, and the corridor
is checked against your position.

Run
---
    python3 waypoint_gui.py [IP_RC] [--drone M400|M350|M300|MAVIC3|MINI4]
                            [--operator LAT,LON] [--buffer 15] [--max-range 80]

    IP_RC        RC/app IP (omit to auto-discover)
    --drone      aircraft profile -> selects the flight speed (default M400)
    --operator   your GPS position LAT,LON. If omitted, the operator is assumed
                 to be at the takeoff point (map origin).
    --buffer     keep-out radius around the operator, metres (default 15)
    --max-range  furthest a waypoint may be placed from takeoff, metres (default 80)

Controls
--------
    Left-click map : place / move the waypoint
    FLY            : arm the command (validates keep-out first)
    CONFIRM        : actually send gotoWPwithPID at the profile's max speed
    STOP           : abort mission (stops autonomous motion, hovers)
    RTH            : return to home
    Override box   : allow flying even when the path enters the keep-out zone

Safety
------
A qualified pilot must always be ready to take manual control. Touching the RC
sticks engages manual override and the app will reject these commands. This GUI
never flies anything until you click CONFIRM.

License: MIT
"""

import argparse
import math
import os
import sys

# Make GroundStation/Python importable and reuse the test-flight helpers.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PY_DIR = os.path.normpath(os.path.join(_HERE, "..", "GroundStation", "Python"))
if _PY_DIR not in sys.path:
    sys.path.insert(0, _PY_DIR)

import matplotlib  # noqa: E402

# Use the Tk backend, NOT Qt. djiInterface imports cv2 (OpenCV), which ships its
# own Qt plugins and hijacks QT_QPA_PLATFORM_PLUGIN_PATH; matplotlib's Qt backend
# then fails to load the "xcb" plugin and aborts. TkAgg avoids Qt entirely.
# Honour an explicitly-set MPLBACKEND (e.g. Agg for headless testing).
if not os.environ.get("MPLBACKEND"):
    matplotlib.use("TkAgg")

import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.animation as animation  # noqa: E402
from matplotlib.widgets import Button, CheckButtons  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

from djiInterface import DJIInterface  # noqa: E402
from test_flight import (  # noqa: E402
    speed_magnitude, rejected, DRONE_PROFILES,
    latlon_to_local, local_to_latlon, point_segment_distance, DEFAULT_OPERATOR_LATLON,
)

HEADING_ARROW_M = 6.0      # length of the heading arrow on the map (metres)
CONFIRM_TIMEOUT_MS = 6000  # arm auto-expires after this long
UPDATE_INTERVAL_MS = 200   # telemetry refresh / redraw cadence


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class WaypointGUI:
    def __init__(self, dji, profile, operator_latlon=None, buffer_m=15.0, max_range_m=80.0,
                 monitor=False, wp_source=None):
        self.dji = dji
        self.profile = profile
        self.speed = profile["maxHorizontalSpeedMps"]
        self.buffer_m = buffer_m
        self.max_range_m = max_range_m

        # Monitor mode: read-only live view of an externally driven flight
        # (e.g. test_flight). Clicks/FLY/CONFIRM are disabled; the target shown
        # comes from wp_source() -> (lat, lon) or None. STOP/RTH stay live as
        # emergency controls.
        self.monitor = monitor
        self.wp_source = wp_source

        self.operator_latlon = operator_latlon  # None -> takeoff point
        self.origin = None                       # (lat0, lon0), set on first fix
        self.operator_local = (0.0, 0.0)

        self.wp_local = None      # (east, north) or None
        self.wp_latlon = None
        self.armed = False
        self.override = False
        self.status = ("MONITOR - test flight is driving the drone."
                       if monitor else "Waiting for GPS fix...")
        self.status_color = "black"

        self._build_figure()
        self._timer = self.fig.canvas.new_timer(interval=CONFIRM_TIMEOUT_MS)
        self._timer.single_shot = True
        self._timer.add_callback(self._disarm_timeout)

    # ---- figure construction ----
    def _build_figure(self):
        self.fig = plt.figure(figsize=(11, 8))
        self.fig.canvas.manager.set_window_title(
            "WildBridge - Live Map (MONITOR)" if self.monitor else "WildBridge - Waypoint GUI")

        self.ax = self.fig.add_axes([0.06, 0.22, 0.62, 0.72])
        self.ax.set_aspect("equal", adjustable="box")
        lim = self.max_range_m + 15
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        self.ax.grid(True, alpha=0.3)
        self.ax.axhline(0, color="grey", lw=0.5)
        self.ax.axvline(0, color="grey", lw=0.5)

        # Artists
        self.keepout = Circle((0, 0), self.buffer_m, color="red", alpha=0.12, zorder=0)
        self.ax.add_patch(self.keepout)
        self.range_ring = Circle((0, 0), self.max_range_m, fill=False,
                                 ls=":", color="grey", lw=0.8, zorder=0)
        self.ax.add_patch(self.range_ring)
        (self.operator_dot,) = self.ax.plot(0, 0, "g^", ms=12, label="Operator")
        (self.drone_dot,) = self.ax.plot([], [], "bo", ms=12, label="Drone")
        (self.heading_line,) = self.ax.plot([], [], "b-", lw=2)
        (self.wp_dot,) = self.ax.plot([], [], "r*", ms=18, label="Waypoint")
        (self.path_line,) = self.ax.plot([], [], "r--", lw=1.5, label="Predicted path")
        self.ax.legend(loc="upper right", fontsize=8)

        # Status panel (right side)
        self.info_ax = self.fig.add_axes([0.70, 0.22, 0.28, 0.72])
        self.info_ax.axis("off")
        self.info_text = self.info_ax.text(0.0, 1.0, "", va="top", ha="left",
                                           family="monospace", fontsize=9)

        # Buttons. STOP/RTH always present (emergency controls). FLY/CONFIRM and
        # the keep-out override only exist in interactive mode.
        self.b_stop = Button(self.fig.add_axes([0.42, 0.06, 0.12, 0.08]), "STOP")
        self.b_rth = Button(self.fig.add_axes([0.56, 0.06, 0.12, 0.08]), "RTH")
        self.b_stop.color = "lightcoral"
        self.b_stop.on_clicked(self._on_stop)
        self.b_rth.on_clicked(self._on_rth)

        if not self.monitor:
            self.b_fly = Button(self.fig.add_axes([0.06, 0.06, 0.14, 0.08]), "FLY ▶")
            self.b_confirm = Button(self.fig.add_axes([0.22, 0.06, 0.18, 0.08]), "CONFIRM")
            self.b_confirm.color = "lightgrey"
            self.b_fly.on_clicked(self._on_fly)
            self.b_confirm.on_clicked(self._on_confirm)
            self.check = CheckButtons(self.fig.add_axes([0.70, 0.06, 0.28, 0.09]),
                                      ["Allow keep-out override"], [False])
            self.check.on_clicked(self._on_override)
            self.fig.canvas.mpl_connect("button_press_event", self._on_click)

    # ---- helpers ----
    def _ensure_origin(self, loc):
        if self.origin is not None:
            return True
        lat, lon = loc.get("latitude"), loc.get("longitude")
        if lat is None or lon is None:
            return False
        self.origin = (lat, lon)
        if self.operator_latlon is None:
            self.operator_local = (0.0, 0.0)
        else:
            self.operator_local = latlon_to_local(
                self.operator_latlon[0], self.operator_latlon[1], lat, lon)
        self.operator_dot.set_data([self.operator_local[0]], [self.operator_local[1]])
        self.keepout.center = self.operator_local
        return True

    def _closest_approach(self, drone_local):
        """Closest approach of the drone->wp path to the operator (metres)."""
        if self.wp_local is None or drone_local is None:
            return None
        ox, oy = self.operator_local
        return point_segment_distance(ox, oy, drone_local[0], drone_local[1],
                                      self.wp_local[0], self.wp_local[1])

    def _path_violates(self, drone_local):
        ca = self._closest_approach(drone_local)
        return ca is not None and ca < self.buffer_m

    def _set_status(self, msg, color="black"):
        self.status = msg
        self.status_color = color

    def _disarm(self):
        self.armed = False
        if hasattr(self, "b_confirm"):
            self.b_confirm.color = "lightgrey"
        self._timer.stop()

    def _disarm_timeout(self):
        if self.armed:
            self._disarm()
            self._set_status("Confirmation timed out - click FLY again.", "darkorange")

    # ---- callbacks ----
    def _on_click(self, event):
        if event.inaxes is not self.ax or event.button != 1:
            return
        # Ignore clicks while the matplotlib pan/zoom tool is active.
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        if toolbar is not None and getattr(toolbar, "mode", ""):
            return
        if self.origin is None:
            self._set_status("No GPS fix yet - cannot place waypoint.", "red")
            return
        east, north = event.xdata, event.ydata
        dist = math.hypot(east, north)
        if dist > self.max_range_m:
            scale = self.max_range_m / dist
            east, north = east * scale, north * scale
            self._set_status(f"Waypoint clamped to max range ({self.max_range_m:.0f} m).",
                             "darkorange")
        else:
            self._set_status("Waypoint set. Review the path, then FLY.", "black")
        self.wp_local = (east, north)
        self.wp_latlon = local_to_latlon(east, north, self.origin[0], self.origin[1])
        self._disarm()  # any change must be re-confirmed

    def _on_fly(self, _event):
        if self.wp_local is None:
            self._set_status("Set a waypoint first (click on the map).", "red")
            return
        drone_local = self._drone_local()
        if drone_local is None:
            self._set_status("No drone position - cannot fly.", "red")
            return
        ca = self._closest_approach(drone_local)
        if self._path_violates(drone_local) and not self.override:
            self._set_status(
                f"BLOCKED: path passes {ca:.1f} m from operator (< {self.buffer_m:.0f} m).\n"
                f"Tick override to fly anyway.", "red")
            self._disarm()
            return
        self.armed = True
        self.b_confirm.color = "yellow"
        self._timer.start()
        warn = "  [OVERRIDE: inside keep-out]" if self._path_violates(drone_local) else ""
        self._set_status(
            f"ARMED{warn}\nCONFIRM to fly to:\n  {self.wp_latlon[0]:.7f}, {self.wp_latlon[1]:.7f}\n"
            f"  speed {self.speed} m/s, closest approach {ca:.1f} m", "blue")

    def _on_confirm(self, _event):
        if not self.armed:
            self._set_status("Nothing armed. Click FLY first.", "darkorange")
            return
        loc = self.dji.getLocation()
        alt = loc.get("altitude", 0.0)
        yaw = self.dji.getHeading()  # keep current heading (no surprise rotation)
        lat, lon = self.wp_latlon
        resp = self.dji.requestSendGoToWPwithPID(lat, lon, alt, yaw, speed=self.speed)
        self._disarm()
        if rejected(resp):
            self._set_status(f"Command rejected by drone:\n  {resp!r}", "red")
        else:
            self._set_status(f"FLYING to waypoint at {self.speed} m/s.\n  resp: {resp!r}", "green")

    def _on_stop(self, _event):
        resp = self.dji.requestAbortMission()
        self._disarm()
        self._set_status(f"STOP sent (mission aborted).\n  resp: {resp!r}", "darkorange")

    def _on_rth(self, _event):
        resp = self.dji.requestSendRTH()
        self._disarm()
        self._set_status(f"RTH sent.\n  resp: {resp!r}", "purple")

    def _on_override(self, _label):
        self.override = self.check.get_status()[0]

    # ---- live update ----
    def _drone_local(self):
        if self.origin is None:
            return None
        loc = self.dji.getLocation()
        lat, lon = loc.get("latitude"), loc.get("longitude")
        if lat is None or lon is None:
            return None
        return latlon_to_local(lat, lon, self.origin[0], self.origin[1])

    def _update(self, _frame):
        loc = self.dji.getLocation()
        if not self._ensure_origin(loc):
            self.info_text.set_text("Waiting for GPS fix...\n(start the app, check telemetry)")
            return

        drone_local = self._drone_local()
        if drone_local is not None:
            dx, dy = drone_local
            self.drone_dot.set_data([dx], [dy])
            heading = self.dji.getHeading()
            hx = dx + HEADING_ARROW_M * math.sin(math.radians(heading))
            hy = dy + HEADING_ARROW_M * math.cos(math.radians(heading))
            self.heading_line.set_data([dx, hx], [dy, hy])

        # Monitor mode: the displayed waypoint mirrors whatever the external
        # flight is currently commanding.
        if self.monitor:
            tgt = self.wp_source() if self.wp_source else None
            if tgt is not None:
                self.wp_latlon = tgt
                self.wp_local = latlon_to_local(tgt[0], tgt[1], self.origin[0], self.origin[1])
            else:
                self.wp_latlon = None
                self.wp_local = None

        if self.wp_local is not None:
            wx, wy = self.wp_local
            self.wp_dot.set_data([wx], [wy])
            if drone_local is not None:
                self.path_line.set_data([drone_local[0], wx], [drone_local[1], wy])
                violates = self._path_violates(drone_local)
                self.path_line.set_color("red" if violates else "limegreen")
        else:
            self.wp_dot.set_data([], [])
            self.path_line.set_data([], [])

        self.info_text.set_text(self._info_string(drone_local))
        self.info_text.set_color(self.status_color)
        return ()

    def _info_string(self, drone_local):
        loc = self.dji.getLocation()
        spd = speed_magnitude(self.dji.getSpeed())
        lines = [
            f"Drone: {self.profile['name']}",
            f"Flight speed (cmd): {self.speed} m/s",
            f"Keep-out buffer: {self.buffer_m:.0f} m",
            f"Override: {'ON' if self.override else 'off'}",
            "-" * 30,
            f"Battery: {self.dji.getBatteryLevel()}%   Sats: {self.dji.getSatelliteCount()}",
            f"Alt: {loc.get('altitude', 0.0):.1f} m   Speed: {spd:.1f} m/s",
            f"Heading: {self.dji.getHeading():.0f} deg",
            f"Flight mode: {self.dji.getFlightMode()}",
            f"Manual override: {self.dji.isManualOverrideActive()}",
            "-" * 30,
        ]
        if self.wp_local is not None and drone_local is not None:
            dx = self.wp_local[0] - drone_local[0]
            dy = self.wp_local[1] - drone_local[1]
            dist = math.hypot(dx, dy)
            bearing = (math.degrees(math.atan2(dx, dy)) + 360) % 360
            ca = self._closest_approach(drone_local)
            lines += [
                f"WP: {self.wp_latlon[0]:.7f},",
                f"    {self.wp_latlon[1]:.7f}",
                f"Distance to WP: {dist:.1f} m",
                f"Bearing to WP: {bearing:.0f} deg",
                f"Path closest approach: {ca:.1f} m"
                + ("  <-- INSIDE KEEP-OUT" if ca < self.buffer_m else ""),
                "-" * 30,
            ]
        lines += ["STATUS:", self.status]
        return "\n".join(lines)

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self._update, interval=UPDATE_INTERVAL_MS,
            blit=False, cache_frame_data=False)
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_operator(value):
    if not value:
        return None
    try:
        lat, lon = (float(x) for x in value.split(","))
        return (lat, lon)
    except ValueError:
        raise argparse.ArgumentTypeError("operator must be 'LAT,LON', e.g. 55.367,10.43")


def main():
    parser = argparse.ArgumentParser(description="WildBridge waypoint GUI")
    parser.add_argument("ip", nargs="?", default="", help="RC/app IP (omit to auto-discover)")
    parser.add_argument("--drone", default="M400", choices=sorted(DRONE_PROFILES.keys()),
                        help="aircraft profile (selects flight speed). Default: M400")
    parser.add_argument("--operator", type=parse_operator, default=DEFAULT_OPERATOR_LATLON,
                        help="operator GPS as LAT,LON (default: Nenana, Alaska "
                             f"{DEFAULT_OPERATOR_LATLON[0]},{DEFAULT_OPERATOR_LATLON[1]})")
    parser.add_argument("--buffer", type=float, default=15.0,
                        help="keep-out radius around operator, metres (default 15)")
    parser.add_argument("--max-range", type=float, default=80.0,
                        help="max waypoint distance from takeoff, metres (default 80)")
    args = parser.parse_args()

    profile = DRONE_PROFILES[args.drone]
    dji = DJIInterface(args.ip)
    if dji.IP_RC == "":
        print("ERROR: no drone IP (discovery failed and none supplied).")
        return 2

    dji.startTelemetryStream()
    print(f"Connected to {dji.IP_RC}. Opening GUI "
          f"({profile['name']}, {profile['maxHorizontalSpeedMps']} m/s, "
          f"keep-out {args.buffer} m)...")
    gui = WaypointGUI(dji, profile, operator_latlon=args.operator,
                      buffer_m=args.buffer, max_range_m=args.max_range)
    try:
        gui.run()
    finally:
        dji.stopTelemetryStream()
    return 0


if __name__ == "__main__":
    sys.exit(main())
