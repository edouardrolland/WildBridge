#!/usr/bin/env python3
"""
Stress test for synchronized H20T multi-lens capture (thermal + wide + zoom).
"""

import hashlib
import os
import statistics
import sys
import time
from datetime import datetime

from djiInterface import DJIInterface, canonical_lenses, LENS_KEYS

# Default RC IP (WildBridge app WLAN address, port 8080). Override on the CLI.
IP_RC = "172.18.64.251"
DEFAULT_ITERS = 1


def is_jpeg(data: bytes) -> bool:
    """True if bytes look like a complete JPEG: SOI (FF D8) start, EOI (FF D9) end."""
    return len(data) > 4 and data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"


def main():
    ip = IP_RC
    iters = DEFAULT_ITERS
    lenses = canonical_lenses(LENS_KEYS)

    print(f"Stress test: {iters} captures of lenses {lenses} against {ip}:8080")

    drone = DJIInterface(ip)
    out_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"stress_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    os.makedirs(out_dir, exist_ok=True)

    # Per-lens accumulators for the timing/size report.
    dl_times = {lens: [] for lens in lenses}
    dl_sizes = {lens: [] for lens in lenses}
    # Remember each shutter's on-camera filenames to detect stale repeats.
    prev_names = {}
    n_pass = 0
    failures = []  # (iter_index, reason)

    for i in range(1, iters + 1):
        tag = f"[{i:>3}/{iters}]"
        problems = []

        # --- 1) TAKE: one shutter -------------------------------------------------
        t0 = time.time()
        info = drone.requestCapture()
        take_ms = (time.time() - t0) * 1000
        if not info:
            failures.append((i, "requestCapture returned False"))
            print(f"{tag} CAPTURE FAILED (no descriptor)")
            continue

        # --- 2) descriptor names every requested lens (none null) -----------------
        for lens in lenses:
            if not info.get(lens):
                problems.append(f"descriptor missing '{lens}' (null)")

        # --- 3) filenames advanced vs previous shutter ----------------------------
        for lens in lenses:
            name = info.get(lens)
            if name and prev_names.get(lens) == name:
                problems.append(f"'{lens}' filename stale (== prev {name})")
            if name:
                prev_names[lens] = name

        # --- 4) DOWNLOAD each lens, validate the bytes ----------------------------
        iter_dir = os.path.join(out_dir, f"cap_{i:03d}")
        digests = {}
        for lens in lenses:
            name = info.get(lens)
            if not name:
                continue  # already flagged above
            save_path = os.path.join(iter_dir, f"{lens}.jpg")
            os.makedirs(iter_dir, exist_ok=True)
            t0 = time.time()
            path = drone.downloadByName(name, save_path=save_path)
            dt_ms = (time.time() - t0) * 1000
            if path is None:
                problems.append(f"'{lens}' download failed")
                continue
            data = open(path, "rb").read()
            if not data:
                problems.append(f"'{lens}' empty file")
                continue
            if not is_jpeg(data):
                problems.append(f"'{lens}' not a valid JPEG ({len(data)} B)")
                continue
            dl_times[lens].append(dt_ms)
            dl_sizes[lens].append(len(data))
            digests[lens] = hashlib.md5(data).hexdigest()

        # --- 5) lenses of this shutter are distinct from each other ---------------
        seen = {}
        for lens, dg in digests.items():
            if dg in seen:
                problems.append(f"'{lens}' identical bytes to '{seen[dg]}'")
            seen[dg] = lens

        if problems:
            failures.append((i, "; ".join(problems)))
            print(f"{tag} FAIL ({take_ms:.0f}ms take) — " + "; ".join(problems))
        else:
            n_pass += 1
            sizes = ", ".join(f"{l}={dl_sizes[l][-1]//1024}KB" for l in lenses if dl_sizes[l])
            print(f"{tag} ok  ({take_ms:.0f}ms take) — {sizes}")

    # ---------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print(f"SUMMARY: {n_pass}/{iters} captures fully correct")
    if failures:
        print(f"{len(failures)} FAILED:")
        for idx, reason in failures:
            print(f"  #{idx}: {reason}")

    print("\nPer-lens download stats (successful downloads only):")
    for lens in lenses:
        times = dl_times[lens]
        if not times:
            print(f"  {lens:>8}: no successful downloads")
            continue
        sizes = dl_sizes[lens]
        print(f"  {lens:>8}: n={len(times):>3}  "
              f"time min/med/max = {min(times):.0f}/{statistics.median(times):.0f}/{max(times):.0f} ms  "
              f"size med = {statistics.median(sizes)/1024:.0f} KB")

    print(f"\nImages saved under: {out_dir}")
    return 0 if not failures else 1


if __name__ == "__main__":
    main()