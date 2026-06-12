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

import requests

from djiInterface import (
    DJIInterface,
    LENS_KEYS,
    EP_CAPTURE_THERMAL_IMAGE,
    EP_DOWNLOAD_CAPTURED_IMAGE,
)

# ----------------------------------------------------------------------------
# Hardcode the controller (RC) IP address here. This is the IP shown by the
# WildBridge app / the RC's WLAN address, reachable on port 8080.
IP_RC = "192.168.1.195"
# ----------------------------------------------------------------------------

# lens name (as known by the bridge) -> output filename in the capture folder.
LENS_FILENAMES = {
    "thermal": "thermal.jpg",
    "wide": "wide.jpg",
    "zoom": "zoom.jpg",
}


def take_shutter(base_url):
    """Trip one shutter. Returns (elapsed_seconds, info_dict | None).

    info_dict is the bridge's JSON descriptor: {captureId, thermal, wide, zoom}.
    """
    start = time.perf_counter()
    response = requests.post(base_url + EP_CAPTURE_THERMAL_IMAGE, data="", timeout=30)
    elapsed = time.perf_counter() - start
    try:
        info = response.json()
    except ValueError:
        print(f"  capture returned non-JSON: HTTP {response.status_code}, "
              f"body={response.text[:200]!r}")
        return elapsed, None
    if info.get("error") or not info.get("captureId"):
        print(f"  capture failed: {info}")
        return elapsed, None
    return elapsed, info


def download_lens(base_url, capture_id, lens, save_path):
    """Download one lens from a prior shutter. Returns (elapsed_seconds, path | None)."""
    start = time.perf_counter()
    response = requests.post(
        base_url + EP_DOWNLOAD_CAPTURED_IMAGE,
        data=f"{capture_id},{lens}", timeout=120)
    elapsed = time.perf_counter() - start
    content_type = response.headers.get("Content-Type", "")
    if response.status_code != 200 or not content_type.startswith("image/"):
        print(f"  {lens}: download failed (HTTP {response.status_code}, {content_type!r})")
        return elapsed, None
    with open(save_path, "wb") as f:
        f.write(response.content)
    return elapsed, save_path


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else IP_RC
    print(f"Connecting to WildBridge bridge at {ip}:8080 ...")

    drone = DJIInterface(ip)
    if drone.IP_RC == "":
        print("ERROR: no IP_RC set — aborting.")
        return 1
    base_url = drone.baseCommandUrl

    # Save all images into a timestamped folder next to this script.
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    os.makedirs(out_dir, exist_ok=True)

    # --- 1) TAKE: one shutter exposes all lenses simultaneously -------------------
    print("\nTaking the picture(s) — one shutter (no download yet)...")
    shutter_elapsed, info = take_shutter(base_url)
    if info is None:
        print("\nCapture FAILED. Check that:")
        print("  - the IP is correct and the bridge app is running,")
        print("  - the Pilot Computer currently holds control authority,")
        print("  - an H20T (or thermal-capable) payload is connected.")
        return 1

    capture_id = info["captureId"]
    print(f"Capture {capture_id} succeeded in {shutter_elapsed:.2f} s. Lenses stored on drone:")
    for lens in LENS_KEYS:
        print(f"  {lens:<7} -> {info.get(lens) or '(none stored)'}")

    # --- 2) DOWNLOAD: time each lens individually ---------------------------------
    print("\nDownloading each lens individually:")
    dl = {}  # lens -> download seconds (only for lenses actually downloaded)
    for lens in LENS_KEYS:
        if not info.get(lens):
            print(f"  {lens:<7} -> skipped (not stored for this shutter)")
            continue
        save_path = os.path.join(out_dir, LENS_FILENAMES[lens])
        elapsed, path = download_lens(base_url, capture_id, lens, save_path)
        if path:
            dl[lens] = elapsed
            print(f"  {lens:<7} -> {path} ({os.path.getsize(path)} bytes) in {elapsed:.2f} s")
        else:
            print(f"  {lens:<7} -> download FAILED after {elapsed:.2f} s")

    missing = [lens for lens in LENS_KEYS if not info.get(lens)]
    if missing:
        print(f"\nNot stored this shutter: {', '.join(missing)} — enable the matching")
        print("photo storage (e.g. 'Visible + Infrared') on the H20T so it writes those lenses.")

    # --- 3) SUMMARY ---------------------------------------------------------------
    def grouped(lenses):
        """End-to-end (shutter + the given lenses' downloads), or None if any is missing."""
        if all(lens in dl for lens in lenses):
            return shutter_elapsed + sum(dl[lens] for lens in lenses)
        return None

    print("\n=== Take time (one shutter — all lenses exposed simultaneously) ===")
    print(f"  take 3 pictures (thermal+wide+zoom): {shutter_elapsed:.2f} s")
    print(f"  take zoom + thermal                : {shutter_elapsed:.2f} s   (same shutter)")

    print("\n=== Download time per lens ===")
    for lens in LENS_KEYS:
        if lens in dl:
            print(f"  {lens:<7}: {dl[lens]:.2f} s")
        else:
            print(f"  {lens:<7}: n/a")

    print("\n=== End-to-end (shutter + downloads) ===")
    for label, lenses in (
        ("3 pictures (thermal+wide+zoom)", ("thermal", "wide", "zoom")),
        ("zoom + thermal",                 ("thermal", "zoom")),
    ):
        e2e = grouped(lenses)
        print(f"  {label:<31}: {e2e:.2f} s" if e2e is not None
              else f"  {label:<31}: n/a (a required lens was not stored/downloaded)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
