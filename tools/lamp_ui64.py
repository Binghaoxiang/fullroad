#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
探测器灯排 · 64 灯（上位机）

    探测器板物理上是一条直线：64 只二极管，间距 4.8mm
    电气上是 4 行 x 16 列，物理顺序 = 列优先：
        D10 D20 D30 D40 | D11 D21 D31 D41 | ... | D1F D2F D3F D4F

    板 A（长边）：64 灯
    板 B（短边）：64 灯

用法:  python lamp_ui64.py
"""
import os
import sys
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

_h = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_libs")
if os.path.isdir(_h):
    sys.path.insert(0, _h)
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial:  pip install pyserial")
    raise

ROWS, COLS = 4, 16
N = ROWS * COLS                     # 64

# 物理顺序：从 D4F 到 D10（PCB 从左到右的反序）
#   D4F D3F D2F D1F | D4E D3E D2E D1E | ... | D40 D30 D20 D10
ORDER = [(r, c) for c in range(COLS - 1, -1, -1) for r in range(ROWS - 1, -1, -1)]


def label_of(i):
    r, c = ORDER[i]
    return f"D{r + 1}{c:X}"


class App:
    def __init__(self, root):
        self.root = root
        root.title("探测器灯排 64 · 实时")
        root.configure(bg="#0b0f14")

        self.ser = None
        self.running = False
        self.q = queue.Queue()
        self.data_a = None
        self.data_b = None
        self.cells_a = [[0] * COLS for _ in range(ROWS)]
        self.cells_b = [[0] * COLS for _ in range(ROWS)]
        self.stat = {"A": 0, "B": 0, "S": 0, "P": 0}

        self._build()
        self._scan_ports()
        self._tick()

    def _build(self):
        top = tk.Frame(self.root, bg="#121a24", pady=8, padx=12)
        top.pack(fill="x")
        tk.Label(top, text="串口", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.cb = ttk.Combobox(top, width=8, state="readonly")
        self.cb.pack(side="left", padx=6)
        tk.Button(top, text="刷新", command=self._scan_ports).pack(side="left")
        self.btn = tk.Button(top, text="连接", command=self._toggle,
                             bg="#1c72d4", fg="white", width=6)
        self.btn.pack(side="left", padx=8)
        self.lbl = tk.Label(top, text="A=0 B=0 S=0 P=0", fg="#4fd1c5",
                            bg="#121a24", font=("Consolas", 11, "bold"))
        self.lbl.pack(side="left", padx=16)
        tk.Label(top, text="阈值", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.th = tk.IntVar(value=400)
        tk.Scale(top, from_=0, to=4000, orient="horizontal", variable=self.th,
                 length=140, bg="#121a24", fg="#d7e3ee",
                 troughcolor="#1d2a39", highlightthickness=0).pack(side="left")

        self.W = 1300
        self.cv = tk.Canvas(self.root, width=self.W, height=260, bg="#0b0f14",
                            highlightthickness=0)
        self.cv.pack(padx=8, pady=6)

        self.peak = tk.Label(self.root, text="", fg="#ffd166", bg="#0b0f14",
                             font=("Consolas", 12, "bold"))
        self.peak.pack()
        self.log = tk.Text(self.root, height=3, bg="#080c11", fg="#48627d",
                           font=("Consolas", 9))
        self.log.pack(fill="x", padx=8, pady=(2, 6))

        self._draw()

    def _draw(self):
        self.cv.delete("all")
        x0, bw, gap = 46, 16, 2
        self.items = {"A": [], "B": []}
        self.vtext = {"A": [], "B": []}
        for tag, y in (("A", 40), ("B", 170)):
            self.cv.create_text(x0 - 26, y + 26, text=tag, fill="#5f7c98",
                                font=("Consolas", 14, "bold"))
            for i in range(N):
                r, c = ORDER[i]
                x = x0 + i * (bw + gap)
                self.items[tag].append(self.cv.create_rectangle(
                    x, y, x + bw, y + 52, fill="#13202d", outline="#1b2836"))
                # 每 4 个（一列）标一次位号
                if r == 1:
                    self.cv.create_text(x + bw * 2, y + 62, text=f"{c:X}",
                                        fill="#44607d", font=("Consolas", 8))
            # 顶部/底部标尺
            for i in range(0, N, 4):
                x = x0 + i * (bw + gap)
                self.cv.create_text(x + bw * 2, y - 12, text=str(i),
                                    fill="#2f455c", font=("Consolas", 7))

    def _scan_ports(self):
        ps = [p.device for p in list_ports.comports()]
        self.cb["values"] = ps
        if ps:
            self.cb.set(ps[0])
        self._log("串口: " + (", ".join(ps) if ps else "无"))

    def _toggle(self):
        if self.running:
            self._stop()
            return
        dev = self.cb.get()
        if not dev:
            messagebox.showwarning("提示", "请选择串口")
            return
        try:
            self.ser = serial.Serial(dev, 115200, timeout=0.2)
        except Exception as e:
            messagebox.showerror("打开失败", f"{dev}\n{e}\n（可能被占用）")
            return
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.running = True
        self.btn.config(text="断开", bg="#8c2b2b")
        self._log(f"已连接 {dev}")
        threading.Thread(target=self._worker, daemon=True).start()

    def _stop(self):
        self.running = False
        time.sleep(0.25)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.btn.config(text="连接", bg="#1c72d4")
        self._log("已断开")

    def _worker(self):
        while self.running:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b"K RAW\r\n")
                t0 = time.time()
                while time.time() - t0 < 0.55:
                    n = self.ser.in_waiting
                    if n:
                        for ln in self.ser.read(n).decode("utf-8", "ignore").splitlines():
                            self.q.put(ln)
                    else:
                        time.sleep(0.006)
            except Exception as e:
                self.q.put(f"__ERR__ {e}")
                break
            time.sleep(0.22)

    def _parse(self, line):
        s = line.strip()
        if not s:
            return
        if s.startswith("# A"):
            self.data_a = []
            return
        if s.startswith("# B"):
            self.data_b = []
            return
        if s[:2] in ("R1", "R2", "R3", "R4") and len(s) > 2 and s[2] == " ":
            try:
                nums = [int(x) for x in s[3:].split()][:COLS]
            except ValueError:
                return
            if len(nums) < COLS:
                return
            if self.data_a is not None and len(self.data_a) < ROWS:
                self.data_a.append(nums)
            elif self.data_b is not None and len(self.data_b) < ROWS:
                self.data_b.append(nums)
            return
        if s.startswith("A=") and " B=" in s:
            try:
                d = dict(x.split("=") for x in s.split() if "=" in x)
                for k in ("A", "B", "S", "P"):
                    if k in d:
                        self.stat[k] = int(d[k])
            except Exception:
                pass
            if self.data_a is not None and len(self.data_a) == ROWS:
                self.cells_a = self.data_a
            if self.data_b is not None and len(self.data_b) == ROWS:
                self.cells_b = self.data_b
            self.data_a = self.data_b = None

    def _tick(self):
        try:
            while True:
                ln = self.q.get_nowait()
                if ln.startswith("__ERR__"):
                    self._log(ln)
                    self._stop()
                else:
                    self._parse(ln)
        except queue.Empty:
            pass

        thr = self.th.get()
        info = []
        for tag, cells in (("A", self.cells_a), ("B", self.cells_b)):
            vals = [cells[r][c] for (r, c) in ORDER]
            vmax = max(1, max(vals))
            peak_i = vals.index(max(vals))
            for i, v in enumerate(vals):
                k = min(1.0, v / vmax)
                col = "#%02x%02x%02x" % (int(20 + k * 240), int(30 + k * 200),
                                         int(40 + k * 50))
                lit = v >= thr
                self.cv.itemconfig(self.items[tag][i], fill=col,
                                   outline="#ffd166" if lit else "#1b2836",
                                   width=2 if lit else 1)
            if max(vals) >= thr:
                info.append(f"板{tag}最亮 {label_of(peak_i)}={max(vals)}")
        self.peak.config(text="    ".join(info))
        st = self.stat
        self.lbl.config(text=f"A={st['A']} B={st['B']} S={st['S']} P={st['P']}")
        self.root.after(110, self._tick)

    def _log(self, m):
        self.log.insert("end", m + "\n")
        self.log.see("end")


if __name__ == "__main__":
    r = tk.Tk()
    App(r)
    r.mainloop()
