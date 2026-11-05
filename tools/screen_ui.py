#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
screen_ui.py — 探测器阵列可视化上位机

布局：长边 43 颗（D25 -> D4F，底部横边）
      短边 18 颗（D35 -> D49，右侧竖边）

处理：EMA 平滑 + 阈值过滤 + 重心 + 峰宽

命令：K BOTH（一次扫描返回两边）  K SWEEP（全行程对准）  A <deg>（转舵机）
用法：python screen_ui.py [COM5]
"""

import sys
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
except ImportError:
    sys.path.insert(0, "tools/_libs")
    import serial

LONG_LABELS = [
    "D25", "D35", "D45", "D16", "D26", "D36", "D46",
    "D17", "D27", "D37", "D47",
    "D18", "D28", "D38", "D48",
    "D19", "D29", "D39", "D49",
    "D1A", "D2A", "D3A", "D4A",
    "D1B", "D2B", "D3B", "D4B",
    "D1C", "D2C", "D3C", "D4C",
    "D1D", "D2D", "D3D", "D4D",
    "D1E", "D2E", "D3E", "D4E",
    "D1F", "D2F", "D3F", "D4F",
]
SHORT_LABELS = [
    "D35", "D45", "D16", "D26", "D36", "D46", "D17", "D27", "D37",
    "D47", "D18", "D28", "D38", "D48", "D19", "D29", "D39", "D49",
]

NL = len(LONG_LABELS)
NS = len(SHORT_LABELS)

CELL_W = 16
LONG_Y = 420
LONG_H = 90
SHORT_X = 735
SHORT_W = 80
SHORT_CELL_H = 22
MARGIN_L = 45
CANVAS_W = 1000
CANVAS_H = 640

ALPHA = 0.5
MARGIN = 500
FLOOR = 200


def stats(vals):
    total = sum(vals)
    if total <= 0:
        return None
    pos = sum(i * v for i, v in enumerate(vals)) / total
    var = sum(v * (i - pos) ** 2 for i, v in enumerate(vals)) / total
    return (pos + 1.0, var ** 0.5)


class App:
    def __init__(self, root, port):
        self.root = root
        self.ser = None
        self.ok = 0
        self.bad = 0
        self.long_v = [0] * NL
        self.short_v = [0] * NS
        self.long_s = [0.0] * NL
        self.short_s = [0.0] * NS
        self.long_max = 0
        self.short_max = 0

        root.title("探测器阵列 — 长边 43 / 短边 18")
        root.configure(bg="#101418")

        bar = ttk.Frame(root)
        bar.pack(fill="x", padx=8, pady=4)

        ttk.Label(bar, text="端口").pack(side="left")
        self.port_var = tk.StringVar(value=port)
        ttk.Entry(bar, textvariable=self.port_var, width=8).pack(side="left", padx=4)
        ttk.Button(bar, text="连接", command=self.connect).pack(side="left", padx=2)
        ttk.Button(bar, text="断开", command=self.disconnect).pack(side="left", padx=2)

        self.status = ttk.Label(bar, text="未连接")
        self.status.pack(side="left", padx=10)

        ttk.Label(bar, text="A").pack(side="left")
        self.ang_var = tk.StringVar(value="0")
        ttk.Entry(bar, textvariable=self.ang_var, width=6).pack(side="left", padx=2)
        ttk.Button(bar, text="转", command=self.send_angle).pack(side="left")
        ttk.Button(bar, text="全行程对准", command=self.send_auto).pack(side="left", padx=10)

        self.canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H,
                                bg="#0b0e11", highlightthickness=0)
        self.canvas.pack(padx=8, pady=6)

        self.read1 = ttk.Label(root, text="", font=("Consolas", 14))
        self.read1.pack(fill="x", padx=12)
        self.read2 = ttk.Label(root, text="", font=("Consolas", 14))
        self.read2.pack(fill="x", padx=12, pady=(0, 8))

        self.root.after(400, self.tick)

    def connect(self):
        self.disconnect()
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200,
                                     timeout=0.3, write_timeout=0.5)
            time.sleep(0.4)
            self.ser.reset_input_buffer()
            self.status.config(text="已连接")
        except Exception as exc:
            self.ser = None
            self.status.config(text="失败: %s" % exc)

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.status.config(text="未连接")

    def query_both(self, wait=1.6):
        if not self.ser:
            return (None, None)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"K BOTH\r\n")
        except Exception:
            return (None, None)

        deadline = time.time() + wait
        buf = ""
        lo = None
        sh = None
        while time.time() < deadline:
            try:
                chunk = self.ser.read(512)
            except Exception:
                return (lo, sh)
            if not chunk:
                continue
            buf += chunk.decode("ascii", "ignore")
            for line in buf.split("\r\n"):
                if line.startswith("LONG43"):
                    try:
                        lo = [int(x) for x in line[6:].split()]
                    except ValueError:
                        pass
                elif line.startswith("SHORT18"):
                    try:
                        sh = [int(x) for x in line[7:].split()]
                    except ValueError:
                        pass
            if lo and sh:
                return (lo, sh)
        return (lo, sh)

    def send_angle(self):
        if not self.ser:
            return
        try:
            self.ser.write(("A %s\r\n" % self.ang_var.get()).encode("ascii"))
        except Exception:
            pass

    def send_auto(self):
        if not self.ser:
            return
        try:
            self.ser.write(b"K SWEEP\r\n")
            self.status.config(text="全行程对准中（约 5 秒）...")
        except Exception:
            pass

    def tick(self):
        if self.ser:
            lo, sh = self.query_both()
            if lo and len(lo) == NL:
                self.long_v = lo
                for i in range(NL):
                    self.long_s[i] = self.long_s[i] * (1.0 - ALPHA) + lo[i] * ALPHA
                self.ok += 1
            else:
                self.bad += 1

            if sh and len(sh) == NS:
                self.short_v = sh
                for i in range(NS):
                    self.short_s[i] = self.short_s[i] * (1.0 - ALPHA) + sh[i] * ALPHA
                self.ok += 1
            else:
                self.bad += 1

        self.draw()
        self.root.after(80, self.tick)

    def draw(self):
        c = self.canvas
        c.delete("all")

        lvint = [int(round(x)) for x in self.long_s]
        svint = [int(round(x)) for x in self.short_s]

        self.long_max = max(lvint) if lvint else 0
        self.short_max = max(svint) if svint else 0
        li = lvint.index(self.long_max) if self.long_max else -1
        si = svint.index(self.short_max) if self.short_max else -1

        lt_thr = max(FLOOR, self.long_max - MARGIN)
        st_thr = max(FLOOR, self.short_max - MARGIN)
        lv = [v if v >= lt_thr else 0 for v in lvint]
        sv = [v if v >= st_thr else 0 for v in svint]
        ltop = max(lv) if lv else 0
        stop = max(sv) if sv else 0

        c.create_text(MARGIN_L, 26, anchor="w", fill="#8fa6b8",
                      text="长边 43 颗（左 D25 → 右 D4F）",
                      font=("Microsoft YaHei", 10))
        c.create_text(SHORT_X, 14, anchor="w", fill="#8fa6b8",
                      text="短边 18 颗（下 D35 → 上 D49）",
                      font=("Microsoft YaHei", 9))

        for i in range(NL):
            v = lv[i]
            x1 = MARGIN_L + i * CELL_W
            x2 = x1 + CELL_W - 1
            lit = (i == li) and (v > 0)
            fill = "#ff4757" if lit else ("#2b3a46" if v > 0 else "#161d24")
            c.create_rectangle(x1, LONG_Y, x2, LONG_Y + LONG_H,
                               fill=fill, outline="#33434f")
            c.create_text((x1 + x2) / 2, LONG_Y + 10, text=LONG_LABELS[i],
                          angle=90, fill="#cfe3f0", font=("Consolas", 7))
            c.create_text((x1 + x2) / 2, LONG_Y + LONG_H + 18, text=str(v),
                          angle=90, fill="#ffd166" if lit else "#8fa6b8",
                          font=("Consolas", 7))

        if ltop > 0:
            pts = []
            for i in range(NL):
                cx = MARGIN_L + i * CELL_W + CELL_W / 2
                cy = LONG_Y - (lv[i] / ltop) * 200
                pts.extend([cx, cy])
            c.create_line(*pts, fill="#00d1ff", width=2)

        for i in range(NS):
            v = sv[i]
            y2 = LONG_Y - 10 - i * SHORT_CELL_H
            y1 = y2 - SHORT_CELL_H + 3
            lit = (i == si) and (v > 0)
            fill = "#ff4757" if lit else ("#2b3a46" if v > 0 else "#161d24")
            c.create_rectangle(SHORT_X, y1, SHORT_X + SHORT_W, y2,
                               fill=fill, outline="#33434f")
            c.create_text(SHORT_X + 10, (y1 + y2) / 2, anchor="w",
                          text=SHORT_LABELS[i], fill="#cfe3f0",
                          font=("Consolas", 8))
            c.create_text(SHORT_X + SHORT_W - 6, (y1 + y2) / 2, anchor="e",
                          text=str(v), fill="#ffd166" if lit else "#8fa6b8",
                          font=("Consolas", 8))

        if stop > 0:
            pts = []
            for i in range(NS):
                y2 = LONG_Y - 10 - i * SHORT_CELL_H
                cy = y2 - SHORT_CELL_H / 2
                cx = SHORT_X + SHORT_W + 10 + (sv[i] / stop) * 95
                pts.extend([cx, cy])
            c.create_line(*pts, fill="#ffd166", width=2)

        c.create_rectangle(MARGIN_L, 50, MARGIN_L + NL * CELL_W, LONG_Y,
                           outline="#1d2a35", width=2)

        ls = stats(lv)
        ss = stats(sv)
        lsum = sum(lvint)
        ssum = sum(svint)
        lconc = (self.long_max * 43.0 / lsum) if lsum > 0 else 0.0
        sconc = (self.short_max * 18.0 / ssum) if ssum > 0 else 0.0

        if ls:
            t1 = "长边 峰=%4d @%-4s 重心=%5.2f/43 宽=%4.2f 集中=%4.1f" % (
                self.long_max, LONG_LABELS[li], ls[0], ls[1], lconc)
        else:
            t1 = "长边 无信号"

        if ss:
            t2 = "短边 峰=%4d @%-4s 重心=%5.2f/18 宽=%4.2f 集中=%4.1f" % (
                self.short_max, SHORT_LABELS[si], ss[0], ss[1], sconc)
        else:
            t2 = "短边 无信号"

        self.read1.config(text=t1)
        self.read2.config(text="%s        OK=%d BAD=%d" % (t2, self.ok, self.bad))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    root = tk.Tk()
    App(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
