#!/usr/bin/env python3
"""绘制连续采集的峰轨迹图 + 导出峰表。

用法:
    python plot_trajectory.py test_data/L_diag_60s.npz
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm

from board_layout import EDGE_ORDER, EDGE_SENSORS
import lab_analyze as A

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def track_peaks(peaks_per_frame, t, max_jump=8.0, max_gap=4):
    tracks = []
    for fi, peaks in enumerate(peaks_per_frame):
        used = set()
        for tr in tracks:
            if fi - tr["last_fi"] > max_gap:
                continue
            best = None
            best_d = max_jump + 1.0
            for j, p in enumerate(peaks):
                if j in used:
                    continue
                d = abs(p["pos"] - tr["last_pos"])
                if d < best_d:
                    best_d = d
                    best = j
            if best is not None:
                p = peaks[best]
                used.add(best)
                tr["pts"].append((t[fi], p["pos"], p["amp"]))
                tr["last_fi"] = fi
                tr["last_pos"] = p["pos"]
        for j, p in enumerate(peaks):
            if j not in used:
                tracks.append({"pts": [(t[fi], p["pos"], p["amp"])], "last_fi": fi, "last_pos": p["pos"]})
    return tracks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--out", default=None)
    ap.add_argument("--thr", type=float, default=0.05)
    ap.add_argument("--prom", type=float, default=0.02)
    args = ap.parse_args()

    src = Path(args.npz)
    out = Path(args.out) if args.out else src.with_name(src.stem + "_trajectory.png")
    csv_path = src.with_name(src.stem + "_peaks.csv")

    d = np.load(src)
    raw = d["raw"].astype(np.float32)
    t = d["t"].astype(np.float64)
    order = [str(x) for x in d["edge_order"]]
    sensors = [int(x) for x in d["edge_sensors"]]

    edges = [e for e in ["up", "in_up", "right", "in_right"] if e in order]
    offsets = {}
    off = 0
    for e, n in zip(order, sensors):
        offsets[e] = off
        off += n

    peaks = {}
    profiles = {}
    for e in edges:
        n = EDGE_SENSORS[e]
        prof = raw[:, offsets[e]:offsets[e] + n] / 4095.0
        profiles[e] = prof
        peaks[e] = [A.detect(fr, args.thr, args.prom, 3) for fr in prof]

    fig, axes = plt.subplots(len(edges), 2, figsize=(16, 4.2 * len(edges)),
                             gridspec_kw={"width_ratios": [1.25, 1.0]})
    if len(edges) == 1:
        axes = np.array([axes])
    fig.suptitle(f"连续采集峰轨迹：{src.name}", fontsize=15)

    rows = []
    for r, e in enumerate(edges):
        prof = profiles[e]
        n = prof.shape[1]
        pk = peaks[e]

        ax = axes[r, 0]
        vmax = float(np.percentile(prof, 99.5))
        if vmax <= 0:
            vmax = 1.0
        im = ax.imshow(prof.T, aspect="auto", origin="lower",
                       extent=[t[0], t[-1], 0, n - 1], cmap="viridis",
                       vmin=0, vmax=vmax, interpolation="nearest")
        xs, ys, cs = [], [], []
        for fi, plist in enumerate(pk):
            for p in plist:
                xs.append(t[fi]); ys.append(p["pos"]); cs.append(p["amp"])
                rows.append([f"{t[fi]:.3f}", e, f"{p['pos']:.3f}", f"{p['amp']:.4f}", f"{p['width']:.3f}"])
        ax.scatter(xs, ys, s=5, c="red", alpha=0.55, linewidths=0)
        ax.set_title(f"{e} 原始包络瀑布图（红点=检测峰）")
        ax.set_xlabel("时间 t (s)")
        ax.set_ylabel("传感器索引")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02, label="归一化幅值")

        ax2 = axes[r, 1]
        trs = track_peaks(pk, t)
        for tr in trs:
            if len(tr["pts"]) < 8:
                continue
            tt, pp, aa = zip(*tr["pts"])
            ax2.plot(tt, pp, color="0.65", lw=0.7, alpha=0.7, zorder=1)
        sc = ax2.scatter(xs, ys, c=cs, s=12, cmap="plasma", vmin=0, vmax=max(cs) if cs else 1, zorder=2)
        ax2.set_title(f"{e} 峰轨迹（颜色=幅值）")
        ax2.set_xlabel("时间 t (s)")
        ax2.set_ylabel("峰索引")
        ax2.grid(True, alpha=0.25)
        fig.colorbar(sc, ax=ax2, fraction=0.046, pad=0.02, label="峰幅值")

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160)
    plt.close(fig)

    with csv_path.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "edge", "peak_index", "amplitude", "width"])
        w.writerows(rows)

    print(f"saved {out}")
    print(f"saved {csv_path} ({len(rows)} peaks)")
    for e in edges:
        n = len(peaks[e])
        cnts = [len(p) for p in peaks[e]]
        print(f"{e}: frames={n} peaks_total={sum(cnts)} max_peaks={max(cnts) if cnts else 0} "
              f"frames_with_2+={sum(1 for c in cnts if c >= 2)}")


if __name__ == "__main__":
    main()
