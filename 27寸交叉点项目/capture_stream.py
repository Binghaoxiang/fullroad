#!/usr/bin/env python3
"""连续采集 RAW 帧到单个 npz（默认 60 s）。

用法:
    python capture_stream.py --port COM7 --seconds 60 --delay 3 --out test_data/line.npz
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

try:
    import serial
except ImportError:
    serial = None

from board_layout import EDGE_ORDER, EDGE_SENSORS

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


def parse_raw(frame):
    return np.frombuffer(frame[2:-3], dtype="<u2").astype(np.uint16)


def parse_full_raw(frame):
    return np.frombuffer(frame[2:2 + RAW_BYTES], dtype="<u2").astype(np.uint16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--delay", type=float, default=3.0)
    ap.add_argument("--out", default="test_data/capture_stream.npz")
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if serial is None:
        raise SystemExit("pyserial 未安装")
    if args.delay > 0:
        print(f"delay {args.delay:.1f}s ...", flush=True)
        time.sleep(args.delay)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.02)
    except Exception as exc:
        raise SystemExit(f"打开 {args.port} 失败: {exc}\n请先关闭综合UI或其他占用串口的程序")
    ser.reset_input_buffer()

    frames = []
    times = []
    buf = bytearray()
    mode = "auto"
    pos = 0
    t0 = None
    t_end = None
    print("START", flush=True)
    try:
        while True:
            now = time.time()
            if t0 is not None and now >= t_end:
                break
            n = ser.in_waiting
            chunk = ser.read(n if n > 0 else 1)
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                i = buf.find(HDR)
                if i < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if i:
                    del buf[:i]
                if len(buf) < POS_LEN:
                    break
                if mode in ("auto", "pos"):
                    frame = bytes(buf[:POS_LEN])
                    if checksum_ok(frame):
                        pos += 1
                        del buf[:POS_LEN]
                        mode = "pos"
                        continue
                if mode == "pos":
                    del buf[:1]
                    continue
                if mode in ("auto", "raw") and len(buf) >= RAW_LEN and checksum_ok(bytes(buf[:RAW_LEN])):
                    frame = bytes(buf[:RAW_LEN])
                    del buf[:RAW_LEN]
                    if t0 is None:
                        t0 = time.time()
                        t_end = t0 + args.seconds
                    frames.append(parse_raw(frame))
                    times.append(time.time() - t0)
                    mode = "raw"
                    continue
                if mode == "raw":
                    if len(buf) < RAW_LEN:
                        break
                    del buf[:1]
                    continue
                if len(buf) < FULL_LEN:
                    break
                if mode in ("auto", "full") and checksum_ok(bytes(buf[:FULL_LEN])):
                    frame = bytes(buf[:FULL_LEN])
                    del buf[:FULL_LEN]
                    if t0 is None:
                        t0 = time.time()
                        t_end = t0 + args.seconds
                    frames.append(parse_full_raw(frame))
                    times.append(time.time() - t0)
                    mode = "full"
                    continue
                del buf[:1]
    finally:
        ser.close()

    if not frames:
        raise SystemExit(f"没有收到原始帧（坐标帧 {pos} 个），确认固件 RAW_ONLY=1")
    raw = np.array(frames, dtype=np.uint16)
    t = np.array(times, dtype=np.float64)
    np.savez_compressed(
        out_path, raw=raw, t=t,
        edge_order=np.array(EDGE_ORDER),
        edge_sensors=np.array([EDGE_SENSORS[e] for e in EDGE_ORDER], dtype=np.int32),
    )
    meta = {
        "port": args.port, "baud": args.baud, "mode": mode,
        "seconds": args.seconds, "frames": int(len(raw)),
        "duration": float(t[-1] - t[0]) if len(t) > 1 else 0.0,
        "fps": float((len(t) - 1) / (t[-1] - t[0])) if len(t) > 1 else 0.0,
        "note": args.note,
        "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "file": str(out_path),
    }
    with out_path.with_suffix(".json").open("w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"saved {out_path} frames={len(raw)} duration={meta['duration']:.2f}s fps={meta['fps']:.1f}", flush=True)


if __name__ == "__main__":
    main()
