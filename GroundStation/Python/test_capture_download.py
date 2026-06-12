#!/usr/bin/env python3
"""
Manual timing test for the two-step capture API (requestCapture / requestDownload):

    1. take 3 pictures            — requestCapture()  (one shutter, all lenses)
    2. take 2 pictures (zoom+ir)  — requestCapture()  (another shutter)
    3. download each picture      — requestDownload() per lens, timed individually

A single H20T shutter is atomic: it exposes every lens at once, so "taking 3" and
"taking 2" both cost one shutter — the two measurements let you compare them directly.
The per-lens downloads are then timed one at a time.

Usage:
    python3 test_capture_download.py            # uses the hardcoded IP below
    python3 test_capture_download.py 10.0.0.42  # override IP on the command line
"""

import os
import sys
import time
from datetime import datetime

from djiInterface import DJIInterface, LENS_KEYS

# ----------------------------------------------------------------------------
# Hardcode the controller (RC) IP address here (the RC's WLAN address, port 8080).
IP_RC = "192.168.1.195"
# ----------------------------------------------------------------------------


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else IP_RC
    print(f"Connecting to WildBridge bridge at {ip}:8080 ...")

    drone = DJIInterface(ip)

    capture_lenses = list(LENS_KEYS)

    # Trip the shutter once. requestCapture() returns the descriptor dict
    # (containing the captureId) on success, or False on failure.
    print("Tripping shutter (requestCapture) ...")
    capture_info = drone.requestCapture()
    if not capture_info:
        print("Capture failed; aborting.")
        return 1
    print(f"Capture OK: {capture_info}")

    # Give the H20T a moment to finish writing all lenses to the SD card.
    time.sleep(15)

    download_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "downloads",
        datetime.now().strftime("capture_%Y%m%d_%H%M%S"),
    )

    os.makedirs(download_dir, exist_ok=True)

    print(f"Downloading {len(capture_lenses)} pictures to {download_dir} ...")
    failures = []
    for lens in capture_lenses:
        # Pass the capture descriptor (captureId) so the bridge knows which
        # shutter to pull from, the lens to fetch, and where to write it.
        t0 = time.time()
        result = drone.requestDownload(capture_info, lens, download_dir)
        dt = time.time() - t0
        path = result.get(lens) if isinstance(result, dict) else None
        if path:
            print(f"  {lens}: downloaded in {dt:.1f}s -> {path}")
        else:
            print(f"  {lens}: FAILED after {dt:.1f}s")
            failures.append(lens)
        time.sleep(15)

    if failures:
        print(f"Done with failures: {', '.join(failures)}")
        return 1
    print("All pictures downloaded successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
