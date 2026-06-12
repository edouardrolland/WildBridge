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

CAPTURE_FAILED_HELP = (
    "\nCapture FAILED. Check that:\n"
    "  - the IP is correct and the bridge app is running,\n"
    "  - the Pilot Computer currently holds control authority,\n"
    "  - an H20T (or thermal-capable) payload is connected."
)


def timed_capture(drone, label):
    """Run requestCapture(), print and return (elapsed_seconds, info | None).

    A shutter is atomic — it always exposes all lenses — so requestCapture() takes no
    lens argument; the label only documents what the shot is being used for.
    """
    print(f"\n[{label}] requestCapture() — tripping one shutter ...")
    t0 = time.perf_counter()
    info = drone.requestCapture()
    elapsed = time.perf_counter() - t0
    if not info:
        return elapsed, None
    print(f"    captureId = {info['captureId']}  ({elapsed:.2f} s)")
    return elapsed, info


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else IP_RC
    print(f"Connecting to WildBridge bridge at {ip}:8080 ...")

    drone = DJIInterface(ip)
    if drone.IP_RC == "":
        print("ERROR: no IP_RC set — aborting.")
        return 1

    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )

    # --- 1) Take 3 pictures (thermal + wide + zoom) ------------------------------
    take3_time, info3 = timed_capture(drone, "1: take 3 (thermal+wide+zoom)")
    if info3 is None:
        print(CAPTURE_FAILED_HELP)
        return 1
    for lens in LENS_KEYS:
        print(f"    {lens:<7} -> {info3.get(lens) or '(none stored)'}")

    # --- 2) Take 2 pictures (zoom + ir/thermal) ----------------------------------
    take2_time, info2 = timed_capture(drone, "2: take 2 (zoom+ir)")
    if info2 is None:
        print(CAPTURE_FAILED_HELP)
        return 1

    # --- 3) Download each picture individually (from the 3-lens capture) ----------
    print(f"\n[3] requestDownload each lens individually -> {out_dir}")
    dl_time = {}  # lens -> seconds (only for lenses that downloaded successfully)
    for lens in LENS_KEYS:
        if not info3.get(lens):
            print(f"    {lens:<7} -> skipped (not stored this shutter)")
            continue
        t0 = time.perf_counter()
        files = drone.requestDownload(info3["captureId"], lens, out_dir=out_dir)
        elapsed = time.perf_counter() - t0
        path = files.get(lens) if files else None
        if path and os.path.exists(path):
            dl_time[lens] = elapsed
            print(f"    {lens:<7} -> {path} ({os.path.getsize(path)} bytes) in {elapsed:.2f} s")
        else:
            print(f"    {lens:<7} -> download FAILED after {elapsed:.2f} s")

    # --- Summary -----------------------------------------------------------------
    print("\n=== Take time (one shutter — all lenses exposed simultaneously) ===")
    print(f"    take 3 pictures (thermal+wide+zoom): {take3_time:.2f} s")
    print(f"    take 2 pictures (zoom + ir)        : {take2_time:.2f} s")

    print("\n=== Download time per lens (individual) ===")
    for lens in LENS_KEYS:
        secs = dl_time.get(lens)
        print(f"    {lens:<7}: {secs:.2f} s" if secs is not None else f"    {lens:<7}: n/a")

    return 0 if dl_time else 1


if __name__ == "__main__":
    raise SystemExit(main())
