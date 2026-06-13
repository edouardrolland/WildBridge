#!/usr/bin/env python3
"""
Timing benchmark for the synchronized thermal + wide + zoom capture.

Fires ONE H20T shutter through the WildBridge Android bridge and measures:
  1. the time to TAKE the pictures (the single shutter — all lenses are exposed at once),
  2. the DOWNLOAD time of each lens individually (thermal, wide, zoom),
  3. end-to-end times for two groupings: all 3 pictures, and zoom + thermal only.

A single H20T shutter is atomic: it exposes thermal, wide (RGB) and zoom simultaneously,
so "taking" any subset costs the same one shutter. The lenses differ only in how long they
take to download off the camera SD card — which is what this script measures per type.

Usage:
    python3 test_capture_thermal_rgb.py            # uses the hardcoded IP below
    python3 test_capture_thermal_rgb.py 10.0.0.42  # override IP on the command line
"""

import os
import sys
import time
from datetime import datetime

from djiInterface import DJIInterface, LENS_KEYS

# ----------------------------------------------------------------------------
# Hardcode the controller (RC) IP address here. This is the IP shown by the
# WildBridge app / the RC's WLAN address, reachable on port 8080.
IP_RC = "192.168.1.166"
# ----------------------------------------------------------------------------


def main():

    print(f"Connecting to WildBridge bridge at {IP_RC}:8080 ...")

    drone = DJIInterface(IP_RC)

    # Save all images into a timestamped folder next to this script.
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    
    os.makedirs(out_dir, exist_ok=True)

    # --- 1) TAKE: one shutter exposes all lenses simultaneously -------------------
    print("\nTaking the picture(s) — one shutter (no download yet)...")
    drone.requestLRFMeasure()
    info = drone.requestCapture()
    print(f"Capture info: {info}")
    capture_id = info["captureId"]
    
    # --- 2) DOWNLOAD: time each lens individually ---------------------------------
    results = drone.requestDownload(capture_id, ("thermal", "zoom"), out_dir=out_dir)
        

if __name__ == "__main__":
    raise SystemExit(main())
