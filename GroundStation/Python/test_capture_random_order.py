#!/usr/bin/env python3
"""
Robustness test for the filename-based media path: take N shutters, then download every
resulting picture in RANDOM order.

Why random order matters: downloads now resolve by on-camera filename against the camera's
live media list (the SD card's own index), not a bounded server-side cache. So a file taken
first must still be downloadable after 29 later shutters, and the request order must not
matter. Capturing all N first, then shuffling the download order, exercises exactly that —
the old bounded-cache approach would have lost the earliest captures.

Each H20T/H20N/H30T shutter is atomic and exposes multiple lenses at once; this script
collects every stored lens file across all shutters and downloads the whole shuffled set.

Usage:
    python3 test_capture_random_order.py                 # hardcoded IP, 30 shutters
    python3 test_capture_random_order.py 10.0.0.42       # override IP
    python3 test_capture_random_order.py 10.0.0.42 10    # override IP + shutter count
"""

import hashlib
import os
import random
import statistics
import sys
import time
from datetime import datetime

from djiInterface import DJIInterface, canonical_lenses, LENS_KEYS

# Default RC IP (WildBridge app WLAN address, port 8080). Override on the CLI.
IP_RC = "10.177.40.4"
DEFAULT_SHUTTERS = 1000


def is_jpeg(data: bytes) -> bool:
    """True if bytes look like a complete JPEG: SOI (FF D8) start, EOI (FF D9) end."""
    return len(data) > 4 and data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else IP_RC
    shutters = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_SHUTTERS
    lenses = canonical_lenses(LENS_KEYS)

    print(f"Random-order test: {shutters} shutters of lenses {lenses} against {ip}:8080")

    drone = DJIInterface(ip)
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"random_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    os.makedirs(out_dir, exist_ok=True)

    failures = []  # human-readable problem strings

    # --- 1) TAKE: fire all shutters first, collecting every stored filename -----------
    # jobs = [(shutter_index, lens, filename)] for every lens the payload actually stored.
    jobs = []
    print(f"\nTaking {shutters} shutters ...")
    for i in range(1, shutters + 1):
        info = drone.requestCapture()
        if not info:
            failures.append(f"shutter {i}: requestCapture returned False")
            print(f"  [{i:>3}/{shutters}] CAPTURE FAILED")
            continue
        stored = [(lens, info[lens]) for lens in lenses if info.get(lens)]
        for lens in lenses:
            if not info.get(lens):
                failures.append(f"shutter {i}: descriptor missing '{lens}'")
        for lens, name in stored:
            jobs.append((i, lens, name))
        print(f"  [{i:>3}/{shutters}] OK -> " + ", ".join(f"{l}={n}" for l, n in stored))

    if not jobs:
        print("\nNo files captured; aborting.")
        return 1

    # Filenames must be unique across the whole run (a stale repeat means a capture bug).
    names = [name for _, _, name in jobs]
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        failures.append(f"duplicate filenames across shutters: {sorted(dupes)}")

    # --- 2) (optional) cross-check the live SD-card list contains every captured file --
    listing = drone.listMedia()
    if isinstance(listing, list):
        on_card = {f.get("name") for f in listing}
        missing = [n for n in names if n not in on_card]
        if missing:
            failures.append(f"{len(missing)} captured file(s) absent from listMedia(): {missing[:5]}...")
        print(f"\nlistMedia() reports {len(on_card)} files on card; "
              f"{len(names) - len(missing) if missing else len(names)}/{len(names)} captured files present")

    # --- 3) DOWNLOAD: shuffle, then fetch every file by name in random order -----------
    random.shuffle(jobs)
    print(f"\nDownloading {len(jobs)} files in random order ...")
    dl_times = {lens: [] for lens in lenses}
    digests = {}  # md5 -> filename, to detect identical bytes across distinct files
    for k, (shutter_i, lens, name) in enumerate(jobs, 1):
        save_path = os.path.join(out_dir, name)
        t0 = time.time()
        path = drone.downloadByName(name, save_path=save_path)
        dt_ms = (time.time() - t0) * 1000
        if path is None:
            failures.append(f"{name} (shutter {shutter_i}, {lens}): download failed")
            print(f"  [{k:>3}/{len(jobs)}] FAIL {name}")
            continue
        data = open(path, "rb").read()
        if not is_jpeg(data):
            failures.append(f"{name}: not a valid JPEG ({len(data)} B)")
        dg = hashlib.md5(data).hexdigest()
        if dg in digests and digests[dg] != name:
            failures.append(f"{name}: identical bytes to {digests[dg]}")
        digests[dg] = name
        dl_times[lens].append(dt_ms)
        print(f"  [{k:>3}/{len(jobs)}] OK   {name} ({len(data)//1024} KB, {dt_ms:.0f} ms)")

    # --- 4) REPORT --------------------------------------------------------------------
    print("\nPer-lens download time (successful only):")
    for lens in lenses:
        ts = dl_times[lens]
        if ts:
            print(f"  {lens:>8}: n={len(ts):>3}  mean={statistics.mean(ts):.0f} ms  max={max(ts):.0f} ms")
        else:
            print(f"  {lens:>8}: no successful downloads")

    ok = len(jobs) - sum(1 for f in failures if "download failed" in f)
    print(f"\nDownloaded {ok}/{len(jobs)} files into {out_dir}")
    if failures:
        print(f"\nFAILED with {len(failures)} problem(s):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: all pictures captured and downloaded in random order.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
