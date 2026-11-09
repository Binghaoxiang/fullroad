#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
edge_check.py — 四条边拆开显示，逐颗核对顺序

布局：四条边各占一个横条，从上到下 E0 / E1 / E2 / E3
      每颗探测器一个柱 + 一个丝印标签 + 数值

数据：K EDGES  →  E0:43 v... / E1:18 v... / E2:7 v... / E3:21 v...
"""

import sys
import time
import threading
import tkinter as tk

try:
    import serial
except ImportError:
    sys.path.insert(0, "tools/_libs")
    import serial

import edge_dsp
import screen_pos

# ---------------- 边定义 ----------------
EDGES = [
    ("E0", "下长边", "D25 → D4F", "左 → 右", 21, 63, False, "#00d1ff"),
    ("E1", "右短边", "D35 → D49", "下 → 上", 22, 39, False, "#25c26e"),
    ("E2", "上长边", "D4F → D35", "左 → 右",  22, 63, True, "#ffb020"),
    ("E3", "左短边", "D35 → D3a", "上 → 下", 22, 42, False, "#ff4d5e"),
]


def silk(i):
    return "D%d%s" % (i % 4 + 1, "%X" % (i // 4))


def edge_silk(e):
    _, _, _, _, i0, i1, rev, _ = EDGES[e]
    idxs = list(range(i1, i0 - 1, -1)) if rev else list(range(i0, i1 + 1))
    return [silk(i) for i in idxs]


W, H = 1480, 900
BAR = 44
FOOT = 70
BODY = H - BAR - FOOT

BG = "#0a0e14"
PANEL = "#111820"
CARD = "#0d1620"
TXT = "#e8eef6"
DIM = "#7d8fa3"
ACC = "#00d1ff"
WARN = "#ffb020"
NOISE = 250
FULL = 4096.0

STRIP_H = 196          # 每边占高
X0 = 30                # 左边界
XW = W - 60            # 绘图宽度


class App:
    def __init__(self, root, port):
        self.root = root
        self.ser = None
        self.ok = 0
        self.bad = 0
        self.data = [None] * 4
        self.proc = [None] * 4
        self.base = [None] * 4
        self.tracker = [edge_dsp.Tracker(0.5, 0.1) for _ in range(4)]
        self.show_raw = True
        self.locator = screen_pos.ScreenLocator(200.0, 100.0)
        self.pos = None
        self.last = 0.0

        root.title("四边探测器顺序核对（拆开显示）")
        root.configure(bg=BG)
        root.geometry("%dx%d" % (W, H))
        root.resizable(False, False)

        bar = tk.Frame(root, bg=PANEL, height=BAR)
        bar.pack(fill="x", side="top")
        bar.pack_propagate(False)
        tk.Label(bar, text="端口", bg=PANEL, fg=DIM,
                 font=("Microsoft YaHei", 10)).pack(side="left", padx=(16, 4))
        self.port_var = tk.StringVar(value=port)
        tk.Entry(bar, textvariable=self.port_var, width=8, bg=CARD, fg=TXT,
                 insertbackground=TXT, relief="flat",
                 font=("Consolas", 11)).pack(side="left", ipady=3)
        self.mk(bar, "连接", self.connect, ACC)
        self.mk(bar, "断开", self.disconnect, DIM)
        self.mk(bar, "原始/处理", self.toggle_raw, WARN)
        self.stat = tk.Label(bar, text="未连接", bg=PANEL, fg=DIM,
                             font=("Microsoft YaHei", 10))
        self.stat.pack(side="right", padx=18)

        self.lb = tk.Label(root, text="", bg=PANEL, fg=TXT,
                           font=("Consolas", 13), anchor="w")
        self.lb.pack(fill="x", side="bottom", ipady=16, padx=18)

        self.cv = tk.Canvas(root, width=W, height=BODY, bg=BG,
                            highlightthickness=0)
        self.cv.pack()

        self.root.after(100, self.tick)

    def mk(self, p, text, cmd, col, w=8):
        b = tk.Button(p, text=text, command=cmd, bg=CARD, fg=col,
                      activebackground=col, activeforeground=BG,
                      relief="flat", bd=0, width=w, cursor="hand2",
                      font=("Microsoft YaHei", 10, "bold"))
        b.pack(side="left", padx=5, pady=8)

    def toggle_raw(self):
        self.show_raw = not self.show_raw
        self.stat.config(text="显示原始数据" if self.show_raw else "显示处理后")

    def connect(self):
        self.disconnect()
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200,
                                     timeout=0.06, write_timeout=0.4)
            time.sleep(0.35)
            self.ser.reset_input_buffer()
            self.stat.config(text="已连接")
            threading.Thread(target=self.poll, daemon=True).start()
        except Exception as e:
            self.ser = None
            self.stat.config(text="失败: %s" % e)

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.stat.config(text="未连接")

    def poll(self):
        while True:
            s = self.ser
            if s is None:
                return
            try:
                s.reset_input_buffer()
                s.write(b"K EDGES\r\n")
                buf = ""
                got = {}
                t0 = time.time()
                while time.time() - t0 < 2.5:
                    c = s.read(1024)
                    if not c:
                        if len(got) >= 4:
                            break
                        continue
                    buf += c.decode("ascii", "ignore")
                    for line in buf.split("\r\n"):
                        for k in range(4):
                            tag = "E%d:" % k
                            if line.startswith(tag):
                                parts = line[len(tag):].strip().split()
                                if parts:
                                    try:
                                        n = int(parts[0])
                                        got[k] = [int(x) for x in parts[1:1 + n]]
                                    except ValueError:
                                        pass
                    if len(got) >= 4:
                        break
                for k, v in got.items():
                    self.data[k] = v
                    n = len(v)
                    if self.base[k] is None or len(self.base[k].base) != n:
                        self.base[k] = edge_dsp.Baseline(n, 0.02)
                    self.proc[k] = edge_dsp.process_edge(
                        v, self.base[k], thr_ratio=0.25,
                        noise_floor=60.0, use_median=True, use_savgol=True)
                if got:
                    self.ok += 1
                else:
                    self.bad += 1
            except Exception:
                pass
            time.sleep(0.05)

    def tick(self):
        now = time.time()
        if now - self.last > 0.05:
            self.last = now
            self.draw()
        self.root.after(30, self.tick)

    def draw(self):
        c = self.cv
        c.delete("all")

        for k in range(4):
            self.draw_strip(c, k, 12 + k * (STRIP_H + 8))

        parts = []
        for k in range(4):
            pr = self.proc[k]
            if pr and pr["valid"]:
                s_ = "峰%4d" % int(pr["peak"])
                if pr["mu"] is not None:
                    s_ += " 位%.1f" % (pr["mu"] + 1)
                parts.append("%s %s" % (EDGES[k][0], s_))
            else:
                parts.append("%s --" % EDGES[k][0])
        if self.pos and self.pos.valid:
            px = self.pos
            qc = GOOD if px.quality > 0.8 else (WARN if px.quality > 0.5 else "#ff4d5e")
            self.lb.config(
                text="位置  X=%6.1f mm   Y=%6.1f mm     一致性 %3.0f%%   质量 %3.0f%%   │   %s   OK %d / BAD %d"
                     % (px.x_mm, px.y_mm, px.consistency * 100, px.quality * 100,
                        "　".join(parts), self.ok, self.bad))
        else:
            self.lb.config(text="位置  --（需要四条边都有信号）　　　" + "　".join(parts)
                                + "　　　OK %d / BAD %d" % (self.ok, self.bad))

    def draw_strip(self, c, k, y):
        tag, cn, rng, direc, i0, i1, rev, col = EDGES[k]
        labels = edge_silk(k)
        n = len(labels)
        pr = self.proc[k]
        raw = self.data[k] or []

        c.create_rectangle(X0 - 10, y, X0 + XW + 10, y + STRIP_H,
                           fill=CARD, outline="#1a2431")

        c.create_text(X0, y + 12, anchor="w", text=tag,
                      fill=col, font=("Microsoft YaHei", 15, "bold"))
        c.create_text(X0 + 46, y + 14, anchor="w",
                      text="%s  %s  (%s)  %d 颗" % (cn, rng, direc, n),
                      fill=TXT, font=("Microsoft YaHei", 10))

        # 右上角指标
        if pr and pr["valid"]:
            pos = pr["mu"]
            info = "峰 %d" % int(pr["peak"])
            if pos is not None:
                info += "  位置 %.2f" % (pos + 1)
            if pr["sigma"]:
                info += "  σ %.2f" % pr["sigma"]
            c.create_text(X0 + XW, y + 14, anchor="e", text=info,
                          fill=col, font=("Consolas", 10, "bold"))
        else:
            c.create_text(X0 + XW, y + 14, anchor="e", text="无信号",
                          fill="#ff4d5e" if n else DIM,
                          font=("Consolas", 10, "bold"))

        gx = X0 + 62
        gw = XW - 84
        gy = y + 36
        gh = 104
        gb = gy + gh

        # ---- 自动缩放：以处理后的最大值为上限，留 20% 余量 ----
        if pr and pr["valid"]:
            data = pr["corr"] if not self.show_raw else [float(x) for x in raw]
            vmax = max(data) if data else 1.0
        else:
            data = [float(x) for x in raw] if raw else [0.0]
            vmax = max(data) if data else 1.0
        if vmax < 120.0:
            vmax = 120.0
        vmax *= 1.2

        # 网格 + 刻度（用自动缩放后的刻度）
        for f in (0.0, 0.25, 0.5, 0.75, 1.0):
            yy = gb - gh * f
            c.create_line(gx, yy, gx + gw, yy,
                          fill="#22303e" if f in (0, 1) else "#161f2a")
            c.create_text(gx - 8, yy, anchor="e", text="%d" % int(vmax * f),
                          fill="#3d4d5e", font=("Consolas", 7))

        step = gw / max(n - 1, 1)

        # ---- 原始曲线（暗色背景，仅处理模式时显示）----
        if not self.show_raw and raw:
            pts = []
            for i in range(min(n, len(raw))):
                px = gx + i * step
                py = gb - min(gh, (float(raw[i]) / vmax) * gh)
                pts.extend([px, py])
            if len(pts) >= 4:
                c.create_line(*pts, fill="#2a3a4a", width=1)

        # ---- 主曲线 ----
        if not raw:
            c.create_text(gx + gw / 2, gy + gh / 2, text="无数据",
                          fill="#3d4d5e", font=("Microsoft YaHei", 13))
        else:
            pts = []
            for i in range(min(n, len(data))):
                px = gx + i * step
                py = gb - min(gh, (float(data[i]) / vmax) * gh)
                pts.extend([px, py])
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=2)
                for i in range(0, len(pts) // 2, 2):
                    pass
            # 数据点
            for i in range(min(n, len(data))):
                if float(data[i]) > vmax * 0.12:
                    px = gx + i * step
                    py = gb - min(gh, (float(data[i]) / vmax) * gh)
                    c.create_oval(px - 2.5, py - 2.5, px + 2.5, py + 2.5,
                                  fill=col, outline="")

        # ---- 阈值线 ----
        if pr and pr["valid"]:
            thr = pr["peak"] * 0.25
            ty = gb - min(gh, (thr / vmax) * gh)
            c.create_line(gx, ty, gx + gw, ty, fill="#5a3a3a", dash=(3, 5))

        # ---- 重心/拟合位置标记 ----
        if pr and pr["valid"] and pr["mu"] is not None:
            mx_ = gx + pr["mu"] * step
            c.create_line(mx_, gy - 4, mx_, gb, fill="#ff5566", width=2)
            c.create_polygon(mx_ - 4, gy - 12, mx_ + 4, gy - 12, mx_, gy - 4,
                             fill="#ff5566", outline="")

        # ---- 丝印 ----
        for i in range(n):
            px = gx + i * step
            c.create_text(px, gb + 5, text=labels[i], angle=90,
                          anchor="n", fill="#4e5f72",
                          font=("Consolas", 6))

        # ---- 首尾与方向 ----
        ay = y + STRIP_H - 14
        c.create_text(X0 + 12, ay, anchor="w", text=labels[0],
                      fill=col, font=("Consolas", 8, "bold"))
        c.create_text(X0 + XW - 12, ay, anchor="e", text=labels[-1],
                      fill=col, font=("Consolas", 8, "bold"))
        c.create_text(X0 + XW / 2, ay, anchor="center", text=direc,
                      fill=DIM, font=("Microsoft YaHei", 8))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    root = tk.Tk()
    App(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
