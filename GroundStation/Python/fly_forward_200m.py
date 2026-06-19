#!/usr/bin/env python3
"""Fly the drone 200 m forward (along its current heading) at 25 m/s, while
live-plotting the measured horizontal ground speed.

Flow:
  1. Connect + start telemetry.
  2. Read current location (lat/lon/alt) and heading.
  3. Compute a waypoint 200 m forward along the current heading.
  4. requestAbortAll() to force a COLD start (so the Kp / accel overrides apply),
     then command the waypoint via the XPRIZE tuning endpoint
     requestSendGoToWPwithPIDXPRIZETuning(target, yaw, speed, Kp, accel).
  5. Live-plot horizontal ground speed sqrt(vx^2 + vy^2) with a horizontal
     reference line at the commanded max speed (25 m/s).

When the waypoint-reached flag fires, data logging stops (curve freezes) but the
plot window stays open for inspection. Ctrl+C aborts.

Usage:
    python fly_forward_200m.py [RC_IP]
"""

import csv
import math
import os
import signal
import sys
import time
from datetime import datetime

# Force a non-Qt backend BEFORE any pyplot import. djiInterface imports cv2, which
# ships its own bundled Qt xcb plugin that hijacks matplotlib's Qt backend and
# crashes with "Could not load the Qt platform plugin xcb". TkAgg avoids Qt entirely.
# Must run before `import matplotlib.pyplot` anywhere (incl. inside djiInterface).
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from djiInterface import DJIInterface

# ---- Mission parameters ----
DISTANCE_M = 200.0     # forward distance
MAX_SPEED = 20.0       # m/s commanded max speed
DISTANCE_KP = 0.65      # distance-PID Kp override (XPRIZE tuning)
DISTANCE_KD = 0.001     # distance-PID Kd override (XPRIZE tuning)
MAX_HORIZONTAL_ACCEL = 1.0  # max horizontal accel override (m/s^2, XPRIZE tuning)
EARTH_R = 6378137.0    # WGS-84 equatorial radius (m)
TELEMETRY_TIMEOUT = 15.0  # s to wait for first telemetry fix
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trials")  # per-trial logs


def offset_latlon(lat, lon, heading_deg, distance_m):
    """Return (lat, lon) moved `distance_m` along compass `heading_deg`
    (0 = North, clockwise). Equirectangular offset, fine for a few hundred m."""
    h = math.radians(heading_deg)
    north = distance_m * math.cos(h)   # m toward N
    east = distance_m * math.sin(h)    # m toward E
    dlat = north / EARTH_R
    dlon = east / (EARTH_R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def horizontal_speed(speed):
    """Horizontal ground speed sqrt(vx^2 + vy^2) from telemetry speed dict."""
    if not isinstance(speed, dict):
        return 0.0
    vx = speed.get("x", 0.0) or 0.0
    vy = speed.get("y", 0.0) or 0.0
    return math.hypot(vx, vy)


def wait_for_fix(dji):
    """Block until telemetry has a usable location + heading, or time out."""
    deadline = time.time() + TELEMETRY_TIMEOUT
    while time.time() < deadline:
        loc = dji.getLocation()
        if loc and loc.get("latitude") is not None and loc.get("longitude") is not None:
            return loc
        time.sleep(0.1)
    return None


def open_trial_csv():
    """Open the per-trial CSV up front and write its header, so every sample can be
    flushed to disk as it arrives. Guarantees the raw data survives even a hard kill
    (Ctrl+C, SIGTERM, even SIGKILL) mid-flight — nothing waits for a clean exit.
    Returns (file, writer, tag, csv_path)."""
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"kp{DISTANCE_KP:g}_kd{DISTANCE_KD:g}_acc{MAX_HORIZONTAL_ACCEL:g}_{stamp}"
    csv_path = os.path.join(LOG_DIR, tag + ".csv")
    f = open(csv_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["# distance_kp", DISTANCE_KP])
    w.writerow(["# distance_kd", DISTANCE_KD])
    w.writerow(["# max_horizontal_accel", MAX_HORIZONTAL_ACCEL])
    w.writerow(["# max_speed", MAX_SPEED])
    w.writerow(["# distance_m", DISTANCE_M])
    w.writerow(["t_s", "speed_mps", "roll_deg", "pitch_deg"])
    f.flush()
    print(f"Logging to: {csv_path}")
    return f, w, tag, csv_path


def finalize_trial(tag, csv_path, fig, vs, rolls, pitches, reached_t):
    """Render the PNG and append the index.csv summary row. The raw CSV is already
    on disk (written incrementally); this only adds derived artifacts on a clean exit.
    Guarded by the caller so it runs at most once."""
    stamp = tag.split("_")[-1]
    png_path = os.path.join(LOG_DIR, tag + ".png")
    try:
        fig.savefig(png_path, dpi=120, bbox_inches="tight")
    except Exception as e:
        png_path = ""  # figure may already be torn down on a hard exit
        print(f"PNG save skipped: {e}")

    peak = max(vs) if vs else 0.0
    # Peak-to-peak swing is a quick oscillation proxy for each axis.
    roll_pp = (max(rolls) - min(rolls)) if rolls else 0.0
    pitch_pp = (max(pitches) - min(pitches)) if pitches else 0.0
    index_path = os.path.join(LOG_DIR, "index.csv")
    new = not os.path.exists(index_path)
    with open(index_path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["timestamp", "distance_kp", "distance_kd", "max_horizontal_accel",
                        "max_speed", "distance_m", "peak_speed_mps",
                        "roll_pp_deg", "pitch_pp_deg",
                        "time_to_reach_s", "n_samples", "csv", "png"])
        w.writerow([stamp, DISTANCE_KP, DISTANCE_KD, MAX_HORIZONTAL_ACCEL, MAX_SPEED, DISTANCE_M,
                    f"{peak:.3f}", f"{roll_pp:.3f}", f"{pitch_pp:.3f}",
                    f"{reached_t:.1f}" if reached_t else "",
                    len(vs), os.path.basename(csv_path),
                    os.path.basename(png_path) if png_path else ""])

    print(f"Trial saved: {csv_path}")
    if png_path:
        print(f"             {png_path}")
    print(f"             peak={peak:.2f} m/s  roll_pp={roll_pp:.1f}  pitch_pp={pitch_pp:.1f}  "
          f"reached_t={reached_t}  -> {index_path}")


def main():
    ip = "10.101.210.186"  # REPLACE WITH YOUR RC IP
    dji = DJIInterface(ip)
    if dji.IP_RC == "":
        print("No drone IP. Pass it as an argument or ensure discovery works.")
        return

    print(f"Connecting to {dji.IP_RC}...")
    dji.startTelemetryStream()

    print("Waiting for telemetry fix...")
    loc = wait_for_fix(dji)
    if loc is None:
        print("No telemetry fix received. Aborting.")
        dji.stopTelemetryStream()
        return

    lat0 = loc["latitude"]
    lon0 = loc["longitude"]
    alt0 = loc.get("altitude", 0.0)
    heading = dji.getHeading()

    tgt_lat, tgt_lon = offset_latlon(lat0, lon0, heading, DISTANCE_M)

    print("--- Mission ---")
    print(f"  Start:   lat={lat0:.7f} lon={lon0:.7f} alt={alt0:.1f} m")
    print(f"  Heading: {heading:.1f} deg")
    print(f"  Target:  lat={tgt_lat:.7f} lon={tgt_lon:.7f} alt={alt0:.1f} m")
    print(f"  Yaw sent: {heading:.1f} deg   Max speed: {MAX_SPEED} m/s")

    # Force a COLD start so the Kp / accel overrides are actually applied (a running
    # loop would just hot-swap the target and keep the gains it already captured).
    dji.requestAbortAll()
    time.sleep(0.5)  # let the loop tear down before re-enabling the stick

    tgt_lat, tgt_lon = 65.082876, -147.709281


    # Evaluate the actual distance from the start position to the waypoint being sent.
    dlat = math.radians(tgt_lat - lat0)
    dlon = math.radians(tgt_lon - lon0)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat0)) * math.cos(math.radians(tgt_lat))
         * math.sin(dlon / 2) ** 2)
    actual_distance_m = 2 * EARTH_R * math.asin(math.sqrt(a))
    print(f"  Waypoint distance from start: {actual_distance_m:.2f} m  "
          f"(expected {DISTANCE_M:.1f} m, delta={actual_distance_m - DISTANCE_M:+.2f} m)")

    
    # Enable virtual stick, then command the waypoint via the XPRIZE tuning endpoint.
    dji.requestSendEnableVirtualStick()
    dji.requestSendGoToWPwithPID(
        tgt_lat, tgt_lon, alt0, 90.0, MAX_SPEED)
    
    time.sleep(30)  # let the command take effect before starting to log
    tgt_lat, tgt_lon = 65.082876, -147.709281
    dji.requestSendGoToWPwithPIDprecise(
        tgt_lat, tgt_lon, alt0, 45.0, MAX_SPEED)
        


if __name__ == "__main__":
    main()
