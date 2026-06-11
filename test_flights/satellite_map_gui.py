"""
WildBridge - Satellite Map GUI
==============================

Same purpose as waypoint_gui.py (predict where the drone will fly, keep the
path clear of the operator, confirm before flying) but rendered on real
SATELLITE imagery using tkintermapview (Tk-based, no Qt -> no OpenCV/Qt clash).

Shows:
  * live drone marker (+ short heading line) on satellite tiles,
  * the operator position and a keep-out circle (red),
  * the target waypoint and the predicted straight-line path (green = clear,
    red = the path enters the keep-out zone),
  * interactive mode: left-click the map to set the waypoint, FLY -> CONFIRM
    to send it; monitor mode: read-only view of an externally driven flight,
    with STOP / RTH always live as emergency controls.

OFFLINE NOTE
------------
Satellite tiles are downloaded over the internet. At a remote site (e.g.
Nenana, AK) pre-cache tiles into an offline database while you have a
connection and pass it via database_path=... (use_database_only). See
tkintermapview's OfflineLoader. Without tiles AND without a cache the map is
blank (the drone/waypoint markers still work).

Run
---
    python3 satellite_map_gui.py [IP_RC] [--drone M400|...] [--operator LAT,LON]
                                 [--buffer 15] [--max-range 80]

License: MIT
"""

import argparse
import math
import os
import sys
import tkinter as tk

import tkintermapview

# Make GroundStation/Python importable and reuse the shared helpers.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PY_DIR = os.path.normpath(os.path.join(_HERE, "..", "GroundStation", "Python"))
if _PY_DIR not in sys.path:
    sys.path.insert(0, _PY_DIR)

from djiInterface import DJIInterface  # noqa: E402
from test_flight import (  # noqa: E402
    speed_magnitude, rejected, DRONE_PROFILES, DEFAULT_OPERATOR_LATLON,
    latlon_to_local, point_segment_distance, offset_latlon,
)

# Satellite tile servers (pick one). Google has the best coverage; Esri World
# Imagery is the more license-friendly choice.
TILE_GOOGLE_SAT = "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga"
TILE_ESRI_SAT = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                 "World_Imagery/MapServer/tile/{z}/{y}/{x}")

HEADING_ARROW_M = 8.0      # heading line length on the map (metres)
CONFIRM_TIMEOUT_MS = 6000  # arm auto-expires after this long
UPDATE_INTERVAL_MS = 500   # telemetry refresh / redraw cadence
START_ZOOM = 18


def circle_points(lat0, lon0, radius_m, n=60):
    """A closed ring of (lat, lon) approximating a circle of radius_m."""
    pts = []
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        north = radius_m * math.cos(a)
        east = radius_m * math.sin(a)
        dlat = math.degrees(north / 6371000.0)
        dlon = math.degrees(east / (6371000.0 * math.cos(math.radians(lat0))))
        pts.append((lat0 + dlat, lon0 + dlon))
    return pts


class SatelliteMapGUI:
    def __init__(self, dji, profile, operator_latlon=None, buffer_m=15.0, max_range_m=80.0,
                 monitor=False, wp_source=None, tile_server=TILE_GOOGLE_SAT, database_path=None,
                 plan_leg_m=None, plan_bearing=0.0):
        self.dji = dji
        self.profile = profile
        self.speed = profile["maxHorizontalSpeedMps"]
        self.buffer_m = buffer_m
        self.max_range_m = max_range_m
        self.monitor = monitor
        self.wp_source = wp_source
        self.tile_server = tile_server
        self.database_path = database_path

        # Planned Phase-3 leg the operator aims with the bearing slider. When
        # plan_leg_m is set, a slider + a preview line are shown; get_bearing()
        # feeds the live value to the test flight. Read as a plain float from
        # the test thread (never touch Tk vars off the main thread).
        self.plan_leg_m = plan_leg_m
        self._bearing = float(plan_bearing)
        self.plan_obj = None

        self.operator = operator_latlon or DEFAULT_OPERATOR_LATLON
        self.origin = None        # takeoff (lat, lon), set on first fix
        self.wp_latlon = None
        self.armed = False
        self.override = False
        self._confirm_after = None
        self._running = True

        # Map artists (created lazily / updated each tick)
        self.drone_marker = None
        self.wp_marker = None
        self.path_obj = None
        self.heading_obj = None

        self._build_ui()

    # ---- UI ----
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("WildBridge - Satellite Map" + (" (MONITOR)" if self.monitor else ""))
        self.root.geometry("1100x800")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.map = tkintermapview.TkinterMapView(self.root, corner_radius=0,
                                                 database_path=self.database_path,
                                                 use_database_only=bool(self.database_path))
        self.map.pack(fill="both", expand=True)
        self.map.set_tile_server(self.tile_server, max_zoom=22)
        self.map.set_position(self.operator[0], self.operator[1])
        self.map.set_zoom(START_ZOOM)
        if not self.monitor:
            self.map.add_left_click_map_command(self._on_click)

        # Operator marker + keep-out ring (static)
        self.map.set_marker(self.operator[0], self.operator[1], text="OPERATOR",
                            marker_color_circle="green", marker_color_outside="darkgreen")
        self.map.set_polygon(circle_points(self.operator[0], self.operator[1], self.buffer_m),
                             outline_color="red", border_width=2, fill_color=None)

        # Controls
        ctrl = tk.Frame(self.root)
        ctrl.pack(fill="x", side="bottom")
        if not self.monitor:
            self.b_fly = tk.Button(ctrl, text="FLY ▶", width=10, command=self._on_fly)
            self.b_fly.pack(side="left", padx=4, pady=6)
            self.b_confirm = tk.Button(ctrl, text="CONFIRM", width=12,
                                       bg="lightgrey", command=self._on_confirm)
            self.b_confirm.pack(side="left", padx=4, pady=6)
            self.override_var = tk.BooleanVar(value=False)
            tk.Checkbutton(ctrl, text="Allow keep-out override",
                           variable=self.override_var,
                           command=self._on_override).pack(side="left", padx=8)
        tk.Button(ctrl, text="STOP", width=8, bg="lightcoral",
                  command=self._on_stop).pack(side="left", padx=4, pady=6)
        tk.Button(ctrl, text="RTH", width=8,
                  command=self._on_rth).pack(side="left", padx=4, pady=6)

        # Bearing slider for the planned Phase-3 leg (0=N, 90=E, 180=S, 270=W).
        if self.plan_leg_m:
            self.bearing_var = tk.IntVar(value=int(self._bearing))
            tk.Label(ctrl, text="Leg bearing:").pack(side="left", padx=(12, 2))
            tk.Scale(ctrl, from_=0, to=359, orient="horizontal", length=200,
                     variable=self.bearing_var, command=self._on_bearing).pack(side="left")

        self.status_var = tk.StringVar(
            value="MONITOR - test flight is driving the drone." if self.monitor
            else "Waiting for GPS fix... click the map to set a waypoint.")
        tk.Label(ctrl, textvariable=self.status_var, anchor="w",
                 fg="blue").pack(side="left", padx=12)

        self.info_var = tk.StringVar(value="")
        tk.Label(self.root, textvariable=self.info_var, anchor="nw", justify="left",
                 font=("monospace", 9), bg="#111", fg="#0f0").place(x=10, y=10)

    # ---- geometry / safety ----
    def _drone_latlon(self):
        loc = self.dji.getLocation()
        lat, lon = loc.get("latitude"), loc.get("longitude")
        if lat is None or lon is None:
            return None
        return (lat, lon)

    def _closest_approach(self, drone):
        """Closest approach of the drone->wp path to the operator (metres)."""
        if self.wp_latlon is None or drone is None:
            return None
        dx, dy = latlon_to_local(drone[0], drone[1], self.operator[0], self.operator[1])
        wx, wy = latlon_to_local(self.wp_latlon[0], self.wp_latlon[1],
                                 self.operator[0], self.operator[1])
        return point_segment_distance(0.0, 0.0, dx, dy, wx, wy)

    def _path_violates(self, drone):
        ca = self._closest_approach(drone)
        return ca is not None and ca < self.buffer_m

    def _set_status(self, msg):
        self.status_var.set(msg)

    # ---- arming ----
    def _disarm(self):
        self.armed = False
        if hasattr(self, "b_confirm"):
            self.b_confirm.config(bg="lightgrey")
        if self._confirm_after is not None:
            self.root.after_cancel(self._confirm_after)
            self._confirm_after = None

    def _disarm_timeout(self):
        self._confirm_after = None
        if self.armed:
            self._disarm()
            self._set_status("Confirmation timed out - click FLY again.")

    # ---- callbacks ----
    def _on_click(self, coords):
        lat, lon = coords
        ref = self.origin or self.operator
        east, north = latlon_to_local(lat, lon, ref[0], ref[1])
        dist = math.hypot(east, north)
        if dist > self.max_range_m:
            self._set_status(f"Rejected: {dist:.0f} m from takeoff > max range "
                             f"{self.max_range_m:.0f} m.")
            return
        self.wp_latlon = (lat, lon)
        self._set_status(f"Waypoint set ({lat:.6f}, {lon:.6f}). Review path, then FLY.")
        self._disarm()

    def _on_fly(self):
        if self.wp_latlon is None:
            self._set_status("Set a waypoint first (click the map).")
            return
        drone = self._drone_latlon()
        if drone is None:
            self._set_status("No drone position - cannot fly.")
            return
        ca = self._closest_approach(drone)
        if self._path_violates(drone) and not self.override:
            self._set_status(f"BLOCKED: path passes {ca:.1f} m from operator "
                             f"(< {self.buffer_m:.0f} m). Tick override to fly anyway.")
            self._disarm()
            return
        self.armed = True
        self.b_confirm.config(bg="yellow")
        self._confirm_after = self.root.after(CONFIRM_TIMEOUT_MS, self._disarm_timeout)
        warn = "  [OVERRIDE]" if self._path_violates(drone) else ""
        self._set_status(f"ARMED{warn} - CONFIRM to fly at {self.speed} m/s "
                         f"(closest approach {ca:.1f} m).")

    def _on_confirm(self):
        if not self.armed:
            self._set_status("Nothing armed. Click FLY first.")
            return
        loc = self.dji.getLocation()
        alt = loc.get("altitude", 0.0)
        yaw = self.dji.getHeading()
        lat, lon = self.wp_latlon
        resp = self.dji.requestSendGoToWPwithPID(lat, lon, alt, yaw, speed=self.speed)
        self._disarm()
        self._set_status(f"Command rejected: {resp!r}" if rejected(resp)
                         else f"FLYING to waypoint at {self.speed} m/s.")

    def _on_stop(self):
        resp = self.dji.requestAbortMission()
        self._disarm()
        self._set_status(f"STOP sent (mission aborted). resp={resp!r}")

    def _on_rth(self):
        resp = self.dji.requestSendRTH()
        self._disarm()
        self._set_status(f"RTH sent. resp={resp!r}")

    def _on_override(self):
        self.override = self.override_var.get()

    def _on_bearing(self, value):
        # Runs on the main (Tk) thread; store as a plain float for the test thread.
        self._bearing = float(value)

    def get_bearing(self):
        """Current planned-leg bearing in degrees (thread-safe plain float read)."""
        return self._bearing

    def _on_close(self):
        self._running = False
        self.root.destroy()

    # ---- live update loop ----
    def _update(self):
        if not self._running:
            return
        drone = self._drone_latlon()
        if drone is not None and self.origin is None:
            self.origin = drone

        # Drone marker + heading line
        if drone is not None:
            if self.drone_marker is None:
                self.drone_marker = self.map.set_marker(
                    drone[0], drone[1], text="DRONE",
                    marker_color_circle="blue", marker_color_outside="navy")
            else:
                self.drone_marker.set_position(drone[0], drone[1])
            heading = self.dji.getHeading()
            ahead = (drone[0] + math.degrees(HEADING_ARROW_M * math.cos(math.radians(heading)) / 6371000.0),
                     drone[1] + math.degrees(HEADING_ARROW_M * math.sin(math.radians(heading))
                                             / (6371000.0 * math.cos(math.radians(drone[0])))))
            if self.heading_obj is not None:
                self.heading_obj.delete()
            self.heading_obj = self.map.set_path([drone, ahead], color="blue", width=3)

        # Monitor mode: target mirrors the external flight
        if self.monitor:
            self.wp_latlon = self.wp_source() if self.wp_source else None

        # Waypoint marker + predicted path
        if self.wp_latlon is not None:
            if self.wp_marker is None:
                self.wp_marker = self.map.set_marker(self.wp_latlon[0], self.wp_latlon[1],
                                                     text="WP", marker_color_circle="red",
                                                     marker_color_outside="darkred")
            else:
                self.wp_marker.set_position(self.wp_latlon[0], self.wp_latlon[1])
            if self.path_obj is not None:
                self.path_obj.delete()
            if drone is not None:
                color = "red" if self._path_violates(drone) else "lime green"
                self.path_obj = self.map.set_path([drone, self.wp_latlon], color=color, width=3)
        else:
            if self.wp_marker is not None:
                self.wp_marker.delete()
                self.wp_marker = None
            if self.path_obj is not None:
                self.path_obj.delete()
                self.path_obj = None

        # Planned Phase-3 leg preview: line from the drone along the slider
        # bearing for plan_leg_m metres. Red if it would pass within the keep-out
        # buffer of the operator, orange otherwise. Hidden once the real WP path
        # appears (the test has committed the leg).
        if self.plan_obj is not None:
            self.plan_obj.delete()
            self.plan_obj = None
        if self.plan_leg_m and drone is not None and self.wp_latlon is None:
            br = math.radians(self._bearing)
            end = offset_latlon(drone[0], drone[1],
                                north_m=self.plan_leg_m * math.cos(br),
                                east_m=self.plan_leg_m * math.sin(br))
            ex, ey = latlon_to_local(end[0], end[1], self.operator[0], self.operator[1])
            dx, dy = latlon_to_local(drone[0], drone[1], self.operator[0], self.operator[1])
            ca = point_segment_distance(0.0, 0.0, dx, dy, ex, ey)
            color = "red" if ca < self.buffer_m else "orange"
            self.plan_obj = self.map.set_path([drone, end], color=color, width=2)

        self.info_var.set(self._info_string(drone))
        self.root.after(UPDATE_INTERVAL_MS, self._update)

    def _info_string(self, drone):
        loc = self.dji.getLocation()
        spd = speed_magnitude(self.dji.getSpeed())
        lines = [
            f" {self.profile['name']}   speed {self.speed} m/s   keep-out {self.buffer_m:.0f} m ",
            f" Batt {self.dji.getBatteryLevel()}%  Sats {self.dji.getSatelliteCount()}"
            f"  Alt {loc.get('altitude', 0.0):.1f} m  Spd {spd:.1f} m/s ",
            f" Hdg {self.dji.getHeading():.0f}  Mode {self.dji.getFlightMode()}"
            f"  ManualOverride {self.dji.isManualOverrideActive()} ",
        ]
        if self.wp_latlon is not None and drone is not None:
            ca = self._closest_approach(drone)
            dx, dy = latlon_to_local(self.wp_latlon[0], self.wp_latlon[1], drone[0], drone[1])
            dist = math.hypot(dx, dy)
            lines.append(f" WP dist {dist:.1f} m   path closest approach {ca:.1f} m"
                         + ("  <-- INSIDE KEEP-OUT " if ca < self.buffer_m else " "))
        return "\n".join(lines)

    def run(self):
        self.root.after(300, self._update)
        self.root.mainloop()


# ---------------------------------------------------------------------------
# Main (standalone interactive use)
# ---------------------------------------------------------------------------
def parse_operator(value):
    if not value:
        return None
    try:
        lat, lon = (float(x) for x in value.split(","))
        return (lat, lon)
    except ValueError:
        raise argparse.ArgumentTypeError("operator must be 'LAT,LON'")


def main():
    parser = argparse.ArgumentParser(description="WildBridge satellite waypoint GUI")
    parser.add_argument("ip", nargs="?", default="", help="RC/app IP (omit to auto-discover)")
    parser.add_argument("--drone", default="M400", choices=sorted(DRONE_PROFILES.keys()))
    parser.add_argument("--operator", type=parse_operator, default=DEFAULT_OPERATOR_LATLON,
                        help=f"operator GPS LAT,LON (default Nenana {DEFAULT_OPERATOR_LATLON})")
    parser.add_argument("--buffer", type=float, default=15.0)
    parser.add_argument("--max-range", type=float, default=80.0)
    parser.add_argument("--tiles-db", default=None,
                        help="offline tile database path (use_database_only)")
    args = parser.parse_args()

    profile = DRONE_PROFILES[args.drone]
    dji = DJIInterface(args.ip)
    if dji.IP_RC == "":
        print("ERROR: no drone IP (discovery failed and none supplied).")
        return 2
    dji.startTelemetryStream()
    print(f"Connected to {dji.IP_RC}. Opening satellite map...")
    gui = SatelliteMapGUI(dji, profile, operator_latlon=args.operator,
                          buffer_m=args.buffer, max_range_m=args.max_range,
                          database_path=args.tiles_db)
    try:
        gui.run()
    finally:
        dji.stopTelemetryStream()
    return 0


if __name__ == "__main__":
    sys.exit(main())
