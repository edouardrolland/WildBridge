#!/usr/bin/env python3
"""
Quick manual test for the synchronized thermal + RGB capture.

Fires one H20T shutter through the WildBridge Android bridge and saves BOTH images
it produces from that single shutter:
  - the radiometric thermal R-JPEG
  - the co-aligned visual RGB photo (same scene shown on the controller / WHIP stream)

Because both come from the same shutter on the camera SD card, the thermal and RGB
views are inherently synchronized.

Usage:
    python3 test_capture_thermal_rgb.py            # uses the hardcoded IP below
    python3 test_capture_thermal_rgb.py 10.0.0.42  # override IP on the command line
"""

import os
import sys
from datetime import datetime

from djiInterface import DJIInterface

# ----------------------------------------------------------------------------
# Hardcode the controller (RC) IP address here. This is the IP shown by the
# WildBridge app / the RC's WLAN address, reachable on port 8080.
IP_RC = "192.168.0.113"
# ----------------------------------------------------------------------------


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else IP_RC
    print(f"Connecting to WildBridge bridge at {ip}:8080 ...")

    drone = DJIInterface(ip)
    if drone.IP_RC == "":
        print("ERROR: no IP_RC set — aborting.")
        return 1

    # Save both images into a timestamped folder next to this script.
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    os.makedirs(out_dir, exist_ok=True)
    thermal_path = os.path.join(out_dir, "thermal.jpg")
    rgb_path = os.path.join(out_dir, "rgb.jpg")
    zoom_path = os.path.join(out_dir, "zoom.jpg")

    print("Triggering thermal + RGB + zoom capture (this takes several seconds)...")
    result = drone.requestCaptureThermalImage(
        save_path=thermal_path, rgb_save_path=rgb_path, zoom_save_path=zoom_path)

    if not result:
        print("\nCapture FAILED. Check that:")
        print("  - the IP is correct and the bridge app is running,")
        print("  - the Pilot Computer currently holds control authority,")
        print("  - an H20T (or thermal-capable) payload is connected.")
        return 1

    print("\nCapture succeeded:")
    for key, label in (("thermal", "thermal"), ("rgb", "rgb    "), ("zoom", "zoom   ")):
        path = result.get(key)
        if path:
            print(f"  {label} -> {path} ({os.path.getsize(path)} bytes)")
        else:
            print(f"  {label} -> (none returned)")

    if not result.get("rgb") and not result.get("zoom"):
        print("\nOnly thermal returned — enable 'Visible + Infrared' photo storage on the H20T")
        print("so it also writes the wide/zoom JPEGs to the SD card.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
