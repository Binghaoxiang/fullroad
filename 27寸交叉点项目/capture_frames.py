#!/usr/bin/env python3
"""Robust serial capture for the laser screen master.

The old capture scripts searched for the first FE FD after AA 55.  Raw
samples can contain FE FD, so that parser often locked onto a false footer
and reported ~1 FPS even though the firmware was running at ~17 FPS.  This
script uses the exact frame length from board_layout.py and verifies the
checksum before accepting a frame.

Usage:
    python capture_frames.py COM7 10
    python capture_frames.py COM7 30 --save frames.npz
"""
import argparse
import struct
import time

import numpy as np
import serial

from board_layout import EDGE_SENSORS, EDGE_ORDER

RAW_BYTES = sum(EDGE_SENSORS.values()) * 2
RAW_LEN = 2 + RAW_BYTES + 1 + 2
RESULT_BYTES = 8 * 9 + 9 + 18
FULL_LEN = 2 + RAW_BYTES + RESULT_BYTES + 1 + 2
POS_LEN = 14
HDR = b"\xaa\x55"
FTR = b"\xfe\xfd"


def checksum_ok(frame):
    if frame[-2:] != FTR:
        return False
    cs = 0
    for b in frame[2:-3]:
        cs ^= b
    return cs == frame[-3]


def parse_position(frame):
    x, y, valid = struct.unpack_from("<ffB", frame, 2)
    return {"kind": "pos", "x": x, "y": y, "valid": valid}


def parse_full(frame):
    data = frame[2:-3]
    raw = np.frombuffer(data[:RAW_BYTES], dtype="<u2").astype(np.float32) / 4095.0
    off = RAW_BYTES
    cents = []
    for _ in range(8):
        pos, peak, valid = struct.unpack_from("<ffB", data, off)
        off += 9
        cents.append((pos, peak, valid))
    x, y, valid = struct.unpack_from("<ffB", data, off)
    off += 9
    dbg = struct.unpack_from("<ffffBB", data, off) if off + 18 <= len(data) else None
    return {"kind": "full", "raw": raw, "cents": cents, "x": x, "y": y,
            "valid": valid, "dbg": dbg}


def parse_raw(frame):
    data = frame[2:-3]
    raw = np.frombuffer(data[:RAW_BYTES], dtype="<u2").astype(np.uint16)
    return {"kind": "raw", "raw": raw}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM7")
    ap.add_argument("seconds", nargs="?", type=float, default=10.0)
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 460800, timeout=0.02)
    buf = bytearray()
    frames = []
    timing = []
    mode = "auto"          # auto -> pos/full after the first valid frame
    t0 = time.time()

    while time.time() - t0 < args.seconds:
        # Do not ask ReadFile for 4096 bytes: the CH340 driver can hold the
        # call until the whole buffer fills (~3-4 s at 14 B/frame), which makes
        # every frame in the burst share one timestamp.  Read what is available.
        n = ser.in_waiting
        chunk = ser.read(n if n > 0 else 1)
        if not chunk:
            continue
        buf.extend(chunk)

        # Timing text is printed between frames by TIMING_DEBUG.
        while True:
            k = buf.find(b"T acq=")
            if k < 0:
                break
            e = buf.find(b"\r\n", k)
            if e < 0:
                break
            timing.append(buf[k:e].decode("latin1"))
            del buf[k:e + 2]          # keep any binary frame before the text

        while True:
            i = buf.find(HDR)
            if i < 0:
                if len(buf) > 1:
                    del buf[:-1]
                break
            if i > 0:
                del buf[:i]
            now = time.time()

            if len(buf) < POS_LEN:
                break                 # wait for the rest of the frame

            if mode in ("auto", "pos") and checksum_ok(bytes(buf[:POS_LEN])):
                f = parse_position(bytes(buf[:POS_LEN]))
                f["t"] = now
                frames.append(f)
                del buf[:POS_LEN]
                mode = "pos"
                continue

            if mode == "pos":
                del buf[:1]           # corrupted position frame; resync now
                continue

            # Raw-only frame: AA55 + all envelope u16 + XOR + FEFD.
            if mode in ("auto", "raw") and len(buf) >= RAW_LEN and checksum_ok(bytes(buf[:RAW_LEN])):
                f = parse_raw(bytes(buf[:RAW_LEN]))
                f["t"] = now
                frames.append(f)
                del buf[:RAW_LEN]
                mode = "raw"
                continue

            if mode == "raw":
                if len(buf) < RAW_LEN:
                    break
                del buf[:1]
                continue

            # Full frame length comes from board_layout.py.  If the header is real but the frame
            # has not arrived yet, wait instead of deleting the header.
            if len(buf) < FULL_LEN:
                break

            if mode in ("auto", "full") and checksum_ok(bytes(buf[:FULL_LEN])):
                f = parse_full(bytes(buf[:FULL_LEN]))
                f["t"] = now
                frames.append(f)
                del buf[:FULL_LEN]
                mode = "full"
                continue

            # Not a valid frame at the current alignment; resync one byte.
            del buf[:1]

    ser.close()
    dt = time.time() - t0
    npos = sum(1 for f in frames if f["kind"] == "pos")
    nraw = sum(1 for f in frames if f["kind"] == "raw")
    nfull = len(frames) - npos - nraw
    print(f"frames={len(frames)} pos={npos} raw={nraw} full={nfull} "
          f"duration={dt:.2f}s fps={len(frames)/dt:.2f}")

    pos = [f for f in frames if f["kind"] == "pos"]
    if pos:
        xs = np.array([f["x"] for f in pos])
        ys = np.array([f["y"] for f in pos])
        vs = np.array([f["valid"] for f in pos])
        m = vs == 1
        print(f"position valid={m.mean()*100:.1f}%")
        if m.any():
            print(f"  x mean={xs[m].mean():.3f} sd={xs[m].std():.3f} "
                  f"range={xs[m].min():.3f}..{xs[m].max():.3f} mm")
            print(f"  y mean={ys[m].mean():.3f} sd={ys[m].std():.3f} "
                  f"range={ys[m].min():.3f}..{ys[m].max():.3f} mm")
            if m.sum() > 5:
                print(f"  high-freq dx rms={np.diff(xs[m]).std()/np.sqrt(2):.3f} "
                      f"dy rms={np.diff(ys[m]).std()/np.sqrt(2):.3f} mm")

    full = [f for f in frames if f["kind"] == "full"]
    if full:
        cents = np.array([[c[0] for c in f["cents"]] for f in full])
        val = np.array([[c[2] for c in f["cents"]] for f in full])
        print("edge centroids:")
        for e, name in enumerate(EDGE_ORDER):
            m = val[:, e] == 1
            if m.sum() > 1:
                print(f"  {name:8s} valid={m.mean()*100:5.1f}% "
                      f"mean={cents[m, e].mean():7.3f} sd={cents[m, e].std():5.3f}")
        xs = np.array([f["x"] for f in full])
        ys = np.array([f["y"] for f in full])
        vs = np.array([f["valid"] for f in full])
        m = vs == 1
        if m.any():
            print(f"position x sd={xs[m].std():.3f} y sd={ys[m].std():.3f} mm")

    raw_frames = [f for f in frames if f["kind"] == "raw"]
    if raw_frames:
        r = np.array([f["raw"] for f in raw_frames])
        off = 0
        print("raw envelopes (max/mean):")
        for name in EDGE_ORDER:
            n = EDGE_SENSORS[name]
            seg = r[:, off:off + n]
            print(f"  {name:8s} max={seg.max():6.0f} mean={seg.mean():8.1f}")
            off += n

    for line in timing[-5:]:
        print(line)

    if args.save:
        out = {}
        if pos:
            out["pos_t"] = np.array([f["t"] for f in pos])
            out["pos_x"] = np.array([f["x"] for f in pos])
            out["pos_y"] = np.array([f["y"] for f in pos])
            out["pos_v"] = np.array([f["valid"] for f in pos])
        if full:
            out["raw"] = np.array([f["raw"] for f in full])
            out["cents"] = np.array([[c[0] for c in f["cents"]] for f in full])
            out["val"] = np.array([[c[2] for c in f["cents"]] for f in full])
            out["x"] = np.array([f["x"] for f in full])
            out["y"] = np.array([f["y"] for f in full])
            out["v"] = np.array([f["valid"] for f in full])
        if raw_frames:
            out["raw"] = np.array([f["raw"] for f in raw_frames])
        np.savez(args.save, **out)
        print("saved", args.save)


if __name__ == "__main__":
    main()
