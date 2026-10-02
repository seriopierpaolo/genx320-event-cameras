#!/usr/bin/env python3
"""
Load GenX320 binary event recordings.

32-bit word layout (big-endian, B0 transmitted first):
  bit 31     : sync (always 1)
  bits 30-19 : 12-bit timestamp in µs  (wraps at 4096 µs)
  bit 18     : polarity  (1 = ON, 0 = OFF)
  bits 17-9  : 9-bit x  (0..319)
  bits 8-0   : 9-bit y  (0..319)
"""

import struct
import sys
import numpy as np


def load_events(path: str) -> np.ndarray:
    """
    Load a GenX320 binary recording.

    Returns a structured NumPy array with fields:
      t  (uint16) – 12-bit timestamp in µs
      p  (uint8)  – polarity
      x  (uint16) – x coordinate
      y  (uint16) – y coordinate
    """
    dtype = np.dtype([("t", np.uint16), ("p", np.uint8),
                      ("x", np.uint16), ("y", np.uint16)])

    with open(path, "rb") as f:
        data = f.read()

    # --- Find first sync byte (B0 always has MSB set) ---
    start = next((i for i, b in enumerate(data) if b & 0x80), None)
    if start is None:
        raise ValueError(f"No sync byte found in '{path}'")

    skipped = start
    if skipped:
        print(f"[load_events] Skipped {skipped} leading byte(s) to align on first sync byte.")

    # Trim to aligned region; drop trailing partial word
    aligned = data[start:]
    n_words = len(aligned) // 4
    aligned = aligned[: n_words * 4]

    # Decode all words at once as big-endian uint32
    words = np.frombuffer(aligned, dtype=">u4")

    # Keep only words with sync bit set
    valid_mask = (words >> 31).astype(bool)
    n_invalid = int((~valid_mask).sum())
    if n_invalid:
        print(f"[load_events] Dropped {n_invalid} word(s) with sync bit not set.")
    words = words[valid_mask]

    # Unpack fields
    t = ((words >> 19) & 0xFFF).astype(np.uint16)
    p = ((words >> 18) & 0x001).astype(np.uint8)
    x = ((words >>  9) & 0x1FF).astype(np.uint16)
    y = ( words        & 0x1FF).astype(np.uint16)

    events = np.empty(len(words), dtype=dtype)
    events["t"] = t
    events["p"] = p
    events["x"] = x
    events["y"] = y

    print(f"[load_events] Loaded {len(events):,} events from '{path}'")

    # --- Validate first 10 000 events ---
    check = events[: min(len(events), 10_000)]
    oob = int(((check["x"] >= 320) | (check["y"] >= 320)).sum())
    if oob:
        print(f"[load_events] WARNING: {oob}/{len(check)} of the first {len(check)} "
              f"events have out-of-bounds coordinates (x or y >= 320) "
              f"– check data format.")
    else:
        print(f"[load_events] Validation OK: first {len(check)} events all within [0, 319].")

    return events


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "data/PSGX320LogBin.bin"
    events = load_events(path)

    print(f"\n--- Summary ---")
    print(f"  Total events : {len(events):,}")
    print(f"  ON  events   : {int((events['p'] == 1).sum()):,}")
    print(f"  OFF events   : {int((events['p'] == 0).sum()):,}")
    print(f"  x range      : [{int(events['x'].min())}, {int(events['x'].max())}]")
    print(f"  y range      : [{int(events['y'].min())}, {int(events['y'].max())}]")
    print(f"  t range (µs) : [{int(events['t'].min())}, {int(events['t'].max())}]")
