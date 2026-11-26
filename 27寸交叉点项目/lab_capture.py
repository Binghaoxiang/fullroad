#!/usr/bin/env python3
"""单点采集：按条件抓取 RAW_ONLY 帧，保存 npz + 元数据，并写入 index.csv。

用法:
    python lab_capture.py --phase P1 --x 80 --y0 15 --theta 0 --direction 0 --repeat 1
    python lab_capture.py --next
    python lab_capture.py --dark

plan.csv 放在 test_data/ 下，列:
    seq,phase,x_mm,y0_mm,theta_deg,direction,repeat,note,status,file
"""
import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

try:
    import serial
except ImportError:
    serial = None

from board_layout import EDGE_SENSORS, EDGE_ORDER

RAW_BYTES = sum(EDGE_SENSORS.values()) * 2
RAW_LEN = 2 + RAW_BYTES + 1 + 2
RESULT_BYTES = 8 * 9 + 9 + 18
FULL_LEN = 2 + RAW_BYTES + RESULT_BYTES + 1 + 2
POS_LEN = 14
HDR = b"\xaa\x55"
FTR = b"\xfe\xfd"

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "test_data"
PLAN = DATA / "plan.csv"
INDEX = DATA / "index.csv"
INDEX_COLS = [
    "time", "phase", "seq", "x_mm", "y0_mm", "theta_deg", "direction", "repeat",
    "frames", "fps", "saturated", "file", "note",
]


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


def capture_raw(port, baud, frames, settle, timeout):
    if serial is None:
        raise RuntimeError("pyserial 未安装")
    if settle > 0:
        time.sleep(settle)
    try:
        ser = serial.Serial(port, baud, timeout=0.02)
    except Exception as exc:
        raise RuntimeError(f"打开 {port} 失败: {exc}\n请先关闭综合UI或其他占用串口的程序")
    out, times = [], []
    buf = bytearray()
    mode = "auto"
    pos = 0
    t_end = time.time() + timeout
    try:
        ser.reset_input_buffer()
        while len(out) < frames and time.time() < t_end:
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
                    out.append(parse_raw(bytes(buf[:RAW_LEN])))
                    times.append(time.time())
                    del buf[:RAW_LEN]
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
                    out.append(parse_full_raw(bytes(buf[:FULL_LEN])))
                    times.append(time.time())
                    del buf[:FULL_LEN]
                    mode = "full"
                    continue
                del buf[:1]
    finally:
        ser.close()
    if not out:
        raise RuntimeError(f"没有收到原始波形帧（收到坐标帧 {pos} 个）。确认固件 RAW_ONLY=1，且只有主控在发数据。")
    return np.array(out, dtype=np.uint16), np.array(times), mode


def next_plan_row():
    if not PLAN.exists():
        raise RuntimeError(f"找不到 {PLAN}，先生成测试计划")
    with PLAN.open("r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    for k, row in enumerate(rows):
        if row.get("status", "pending") not in ("done", "skip"):
            return k, row
    return None, None


def mark_plan_row(k, file_name):
    with PLAN.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        rows = list(reader)
    rows[k]["status"] = "done"
    rows[k]["file"] = file_name
    with PLAN.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def next_pending_text():
    idx, row = next_plan_row()
    if row is None:
        return "计划全部完成"
    return (f"下一点: seq={row['seq']} phase={row['phase']} x={row['x_mm']}mm "
            f"y0={row['y0_mm']}mm theta={row['theta_deg']}deg dir={row['direction']} "
            f"rep={row['repeat']} ({row['note']})")


def append_index(meta, frames, fps, saturated, file_name):
    new = not INDEX.exists()
    with INDEX.open("a", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=INDEX_COLS)
        if new:
            writer.writeheader()
        writer.writerow({
            "time": meta["time"], "phase": meta["phase"], "seq": meta["seq"],
            "x_mm": meta["x_mm"], "y0_mm": meta["y0_mm"], "theta_deg": meta["theta_deg"],
            "direction": meta["direction"], "repeat": meta["repeat"],
            "frames": frames, "fps": f"{fps:.2f}", "saturated": int(saturated),
            "file": file_name, "note": meta["note"],
        })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--frames", type=int, default=25)
    ap.add_argument("--settle", type=float, default=0.5)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--phase", default="tmp")
    ap.add_argument("--seq", default="")
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y0", type=float, default=0.0)
    ap.add_argument("--theta", type=float, default=0.0)
    ap.add_argument("--direction", default="0")
    ap.add_argument("--repeat", default="1")
    ap.add_argument("--note", default="")
    ap.add_argument("--next", action="store_true")
    ap.add_argument("--dark", action="store_true")
    args = ap.parse_args()

    DATA.mkdir(exist_ok=True)
    plan_k = None
    if args.next:
        plan_k, row = next_plan_row()
        if row is None:
            print("计划全部完成")
            return
        args.phase = row.get("phase", args.phase)
        args.seq = row.get("seq", "")
        args.x = float(row.get("x_mm") or 0)
        args.y0 = float(row.get("y0_mm") or 0)
        args.theta = float(row.get("theta_deg") or 0)
        args.direction = row.get("direction", "0")
        args.repeat = row.get("repeat", "1")
        args.note = row.get("note", "")
    if args.dark:
        args.phase = "P4_dark"
        args.note = args.note or "laser off, dark"

    raw, t, mode = capture_raw(args.port, args.baud, args.frames, args.settle, args.timeout)
    dt = t[-1] - t[0]
    fps = (len(t) - 1) / dt if len(t) > 1 else 0.0
    saturated = int(raw.max() >= 4090)

    phase_dir = DATA / args.phase
    phase_dir.mkdir(exist_ok=True)
    name = f"{args.seq + '_' if args.seq else ''}{args.phase}_y{args.y0:g}_x{args.x:g}_th{args.theta:g}_{args.direction}_r{args.repeat}"
    name = name.replace(".", "p").replace('"', "")
    npz_path = phase_dir / (name + ".npz")
    k = 1
    while npz_path.exists():
        npz_path = phase_dir / f"{name}_{k}.npz"
        k += 1

    np.savez_compressed(
        npz_path, raw=raw, t=t,
        edge_order=np.array(EDGE_ORDER),
        edge_sensors=np.array([EDGE_SENSORS[e] for e in EDGE_ORDER], dtype=np.int32),
    )
    meta = {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "phase": args.phase, "seq": args.seq,
        "x_mm": args.x, "y0_mm": args.y0, "theta_deg": args.theta,
        "direction": args.direction, "repeat": args.repeat, "note": args.note,
        "port": args.port, "mode": mode, "frames": len(raw), "fps": round(fps, 3),
        "file": npz_path.name,
    }
    with npz_path.with_suffix(".json").open("w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    append_index(meta, len(raw), fps, saturated, str(npz_path.relative_to(ROOT)))
    if plan_k is not None:
        mark_plan_row(plan_k, str(npz_path.relative_to(ROOT)))

    print(f"saved {npz_path.relative_to(ROOT)}  frames={len(raw)} fps={fps:.1f} mode={mode}")
    off = 0
    parts = []
    for e in EDGE_ORDER:
        n = EDGE_SENSORS[e]
        parts.append(f"{e}={raw[:, off:off + n].max()}")
        off += n
    print("edge max:", " ".join(parts))
    if saturated:
        print("WARNING: 有传感器接近饱和 (>=4090)，请降低功率或缩短采样窗口")
    if args.next:
        print(next_pending_text())


if __name__ == "__main__":
    main()
