#!/usr/bin/env python3
"""批量/单文件原始包络分析：多帧峰值聚类，输出每边峰数、位置、幅值、宽度。

用法:
    python lab_analyze.py test_data/P1/xxx.npz
    python lab_analyze.py test_data/P1/*.npz --dark test_data/P4_dark/xxx.npz
    python lab_analyze.py test_data/P2 --dark ... --csv test_data/peaks.csv
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

from board_layout import EDGE_ORDER, EDGE_SENSORS

ADC_MAX = 4095.0


def load_raw(path):
    d = np.load(path)
    raw = np.asarray(d["raw"], dtype=np.float32)
    if raw.ndim == 1:
        raw = raw[None, :]
    order = [str(x) for x in d["edge_order"]] if "edge_order" in d.files else EDGE_ORDER
    sensors = [int(x) for x in d["edge_sensors"]] if "edge_sensors" in d.files else [EDGE_SENSORS[e] for e in order]
    return raw, order, sensors


def load_dark(path):
    if not path:
        return None
    raw, _, _ = load_raw(path)
    return np.median(raw / ADC_MAX, axis=0)


def local_maxima(v):
    n = len(v)
    if n < 3:
        return []
    idx = np.where((v[1:-1] >= v[:-2]) & (v[1:-1] > v[2:]))[0] + 1
    return idx.tolist()


def prominence(v, i):
    n = len(v)
    hi = float(v[i])
    left_min = hi
    j = i - 1
    while j >= 0 and v[j] <= hi:
        left_min = min(left_min, float(v[j]))
        j -= 1
    right_min = hi
    j = i + 1
    while j < n and v[j] <= hi:
        right_min = min(right_min, float(v[j]))
        j += 1
    left_base = 0.0 if j < 0 else left_min
    j_right = j
    right_base = 0.0 if j_right >= n else right_min
    base = max(left_base, right_base)
    return hi - base, base


def sub_index(v, i):
    if i <= 0 or i >= len(v) - 1:
        return float(i)
    den = float(v[i - 1]) - 2.0 * float(v[i]) + float(v[i + 1])
    if abs(den) < 1e-12:
        return float(i)
    d = 0.5 * (float(v[i - 1]) - float(v[i + 1])) / den
    return float(i) + float(np.clip(d, -0.5, 0.5))


def half_width(v, i, base, amp):
    n = len(v)
    half = base + 0.5 * (amp - base)
    left = float(i)
    j = i
    while j > 0 and v[j] >= half:
        j -= 1
    if j < i:
        v0, v1 = float(v[j]), float(v[j + 1])
        left = j + (half - v0) / (v1 - v0) if v1 != v0 else float(j + 1)
    right = float(i)
    j = i
    while j < n - 1 and v[j] >= half:
        j += 1
    if j > i:
        v0, v1 = float(v[j - 1]), float(v[j])
        right = (j - 1) + (v0 - half) / (v0 - v1) if v0 != v1 else float(j - 1)
    return max(0.0, right - left)


def detect(v, thr, prom, min_dist):
    cand = []
    for i in local_maxima(v):
        amp = float(v[i])
        if amp < thr:
            continue
        p, base = prominence(v, i)
        if p < prom:
            continue
        cand.append((i, amp, p, base))
    cand.sort(key=lambda x: x[1], reverse=True)
    kept = []
    for i, amp, p, base in cand:
        if all(abs(i - k[0]) >= min_dist for k in kept):
            kept.append((i, amp, p, base))
    kept.sort(key=lambda x: x[0])
    peaks = []
    for i, amp, p, base in kept:
        peaks.append({
            "pos": sub_index(v, i),
            "amp": amp,
            "prom": p,
            "width": half_width(v, i, base, amp),
            "area": float(np.sum(np.maximum(v[max(0, i - 8):i + 9] - base, 0.0))),
        })
    return peaks


def frame_peaks(profiles, order, sensors, thr, prom, min_dist):
    all_peaks = {e: [] for e in order}
    off = 0
    for e, n in zip(order, sensors):
        for frame in profiles[:, off:off + n]:
            all_peaks[e].append(detect(frame, thr, prom, min_dist))
        off += n
    return all_peaks


def cluster(peaks_per_frame, tol, min_support):
    clusters = []
    for peaks in peaks_per_frame:
        for p in peaks:
            best = None
            best_d = 1e9
            for c in clusters:
                d = abs(p["pos"] - c["pos_mean"])
                if d < best_d:
                    best_d = d
                    best = c
            if best is not None and best_d <= tol:
                best["items"].append(p)
                best["pos_mean"] = float(np.median([q["pos"] for q in best["items"]]))
            else:
                clusters.append({"pos_mean": p["pos"], "items": [p]})
    out = []
    for c in clusters:
        if len(c["items"]) < min_support:
            continue
        out.append({
            "pos": float(np.median([q["pos"] for q in c["items"]])),
            "amp": float(np.median([q["amp"] for q in c["items"]])),
            "width": float(np.median([q["width"] for q in c["items"]])),
            "area": float(np.median([q["area"] for q in c["items"]])),
            "support": len(c["items"]),
        })
    out.sort(key=lambda x: x["pos"])
    return out


def process(path, dark, thr, prom, min_dist, tol, min_support, rows):
    raw, order, sensors = load_raw(path)
    profiles = raw / ADC_MAX
    if dark is not None and len(dark) == profiles.shape[1]:
        profiles = np.clip(profiles - dark[None, :], 0.0, None)
    meta = {}
    mpath = path.with_suffix(".json")
    if mpath.exists():
        with mpath.open("r", encoding="utf-8") as f:
            meta = json.load(f)
    fps = meta.get("fps", 0.0)
    print(f"\n=== {path.name} ===")
    if meta:
        print(f"phase={meta.get('phase')} x={meta.get('x_mm')} y0={meta.get('y0_mm')} "
              f"theta={meta.get('theta_deg')} dir={meta.get('direction')} rep={meta.get('repeat')} "
              f"frames={len(profiles)} fps={fps}")
    print(f"{'edge':<8} {'support':>10} {'pos(idx)':>9} {'pos(mm)':>8} {'amp':>6} {'width':>6}")
    all_frame_peaks = frame_peaks(profiles, order, sensors, thr, prom, min_dist)
    for e in order:
        cl = cluster(all_frame_peaks[e], tol, min_support)
        if not cl:
            print(f"{e:<8} {'-':>10}")
            continue
        for p in cl:
            print(f"{e:<8} {p['support']:>4}/{len(profiles):<5} {p['pos']:9.2f} {'':>8} "
                  f"{p['amp']:6.3f} {p['width']:6.2f}")
        for p in cl:
            rows.append({
                "file": str(path), "phase": meta.get("phase", ""), "x_mm": meta.get("x_mm", ""),
                "y0_mm": meta.get("y0_mm", ""), "theta_deg": meta.get("theta_deg", ""),
                "direction": meta.get("direction", ""), "repeat": meta.get("repeat", ""),
                "edge": e, "pos_idx": f"{p['pos']:.3f}", "amp": f"{p['amp']:.4f}",
                "width": f"{p['width']:.3f}", "support": p["support"], "frames": len(profiles),
            })


def iter_inputs(paths):
    for p in map(Path, paths):
        if p.is_dir():
            yield from sorted(p.rglob("*.npz"))
        elif p.suffix == ".npz":
            yield p
        else:
            print(f"skip {p}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--dark", default=None)
    ap.add_argument("--thr", type=float, default=0.04)
    ap.add_argument("--prom", type=float, default=0.02)
    ap.add_argument("--mindist", type=int, default=3)
    ap.add_argument("--tol", type=float, default=2.0)
    ap.add_argument("--min-support", type=int, default=3)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    dark = load_dark(args.dark) if args.dark else None
    if args.dark:
        print(f"dark loaded: {args.dark}")
    rows = []
    for path in iter_inputs(args.paths):
        process(path, dark, args.thr, args.prom, args.mindist, args.tol, args.min_support, rows)
    if args.csv and rows:
        p = Path(args.csv)
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nsaved {p} ({len(rows)} peaks)")


if __name__ == "__main__":
    main()
