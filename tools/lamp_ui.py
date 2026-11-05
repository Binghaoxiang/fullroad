#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
探测器实时灯排显示（上位机）

    板 A（长边）
    ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐
    │  │  │██│  │  │  │  │  │  │  │  │  │  │  │  │  │   ← 谁被照到谁亮
    └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘
    ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐
    │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │  │   ← 板 B（短边）
    └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘

用法:  python lamp_ui.py
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


class LampApp:
    def __init__(self, root):
        self.root = root
        root.title("探测器灯排 · 实时")
        root.configure(bg="#0b0f14")

        self.ser = None
        self.running = False
        self.q = queue.Queue()
        self.data_a = None
        self.data_b = None
        self.cells_a = [[0] * COLS for _ in range(ROWS)]
        self.cells_b = [[0] * COLS for _ in range(ROWS)]
        self.stat = {"A": 0, "B": 0, "S": 0, "P": 0}
        self.row_max = [0] * ROWS

        self._build()
        self._scan_ports()
        self._tick()

    # ---------- UI ----------
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

        tk.Label(top, text="行选择", fg="#8fb3d6", bg="#121a24").pack(side="left", padx=(12, 2))
        self.row_sel = tk.IntVar(value=-1)      # -1 = 四行叠加
        for i, t in enumerate(["R1", "R2", "R3", "R4", "全部"]):
            tk.Radiobutton(top, text=t, value=(i if i < 4 else -1),
                           variable=self.row_sel, bg="#121a24", fg="#d7e3ee",
                           selectcolor="#1c72d4", activebackground="#121a24",
                           highlightthickness=0).pack(side="left")

        self.lbl = tk.Label(top, text="A=0  B=0  S=0  P=0", fg="#4fd1c5",
                            bg="#121a24", font=("Consolas", 11, "bold"))
        self.lbl.pack(side="left", padx=16)

        tk.Label(top, text="阈值", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.th = tk.IntVar(value=400)
        tk.Scale(top, from_=0, to=4000, orient="horizontal", variable=self.th,
                 length=130, bg="#121a24", fg="#d7e3ee",
                 troughcolor="#1d2a39", highlightthickness=0).pack(side="left")

        # 灯排画布
        self.W = 1040
        self.cv = tk.Canvas(self.root, width=self.W, height=300, bg="#0b0f14",
                            highlightthickness=0)
        self.cv.pack(padx=10, pady=8)

        self.peak = tk.Label(self.root, text="", fg="#ffd166", bg="#0b0f14",
                             font=("Consolas", 12, "bold"))
        self.peak.pack()

        self.log = tk.Text(self.root, height=4, bg="#080c11", fg="#48627d",
                           font=("Consolas", 9))
        self.log.pack(fill="x", padx=10, pady=(4, 8))

        self._draw()

    def _draw(self):
        self.cv.delete("all")
        x0, y0 = 40, 30
        bw, bh, gap = 56, 76, 4
        self.items = {"A": [], "B": []}
        # 板 A
        self.cv.create_text(x0 - 22, y0 + bh / 2, text="A", fill="#5f7c98",
                            font=("Consolas", 14, "bold"))
        for c in range(COLS):
            x = x0 + c * (bw + gap)
            self.items["A"].append(self.cv.create_rectangle(
                x, y0, x + bw, y0 + bh, fill="#13202d", outline="#1b2836", width=2))
            self.cv.create_text(x + bw / 2, y0 - 10, text=f"{c:X}", fill="#44607d",
                                font=("Consolas", 9))
            self.cv.create_text(x + bw / 2, y0 + bh / 2, text="", fill="#8098b0",
                                font=("Consolas", 10), tags=(f"va{c}",))
        # 板 B
        y1 = y0 + bh + 44
        self.cv.create_text(x0 - 22, y1 + bh / 2, text="B", fill="#5f7c98",
                            font=("Consolas", 14, "bold"))
        for c in range(COLS):
            x = x0 + c * (bw + gap)
            self.items["B"].append(self.cv.create_rectangle(
                x, y1, x + bw, y1 + bh, fill="#13202d", outline="#1b2836", width=2))
            self.cv.create_text(x + bw / 2, y1 + bh / 2, text="", fill="#8098b0",
                                font=("Consolas", 10), tags=(f"vb{c}",))

    # ---------- 串口 ----------
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
            messagebox.showerror("打开失败", f"{dev}\n{e}\n（可能被串口助手占用）")
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
                got = 0
                while time.time() - t0 < 0.6:
                    n = self.ser.in_waiting
                    if n:
                        for ln in self.ser.read(n).decode("utf-8", "ignore").splitlines():
                            self.q.put(ln)
                            got += 1
                    else:
                        time.sleep(0.008)
            except Exception as e:
                self.q.put(f"__ERR__ {e}")
                break
            time.sleep(0.25)

    # ---------- 解析（严格按顺序） ----------
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

    # ---------- 刷新 ----------
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

        sel = self.row_sel.get()
        thr = self.th.get()

        for tag, cells, items, vtag in (("A", self.cells_a, self.items["A"], "va"),
                                        ("B", self.cells_b, self.items["B"], "vb")):
            # 按行选择取数
            vals = []
            for c in range(COLS):
                if sel < 0:
                    v = max(cells[r][c] for r in range(ROWS))
                else:
                    v = cells[sel][c]
                vals.append(v)
            vmax = max(1, max(vals))
            peak_c = vals.index(max(vals))
            for c, v in enumerate(vals):
                k = min(1.0, v / vmax)
                r = int(20 + k * 240)
                g = int(30 + k * 200)
                b = int(40 + k * 50)
                lit = v >= thr
                self.cv.itemconfig(items[c], fill="#%02x%02x%02x" % (r, g, b),
                                   outline="#ffd166" if lit else "#1b2836",
                                   width=3 if lit else 2)
                self.cv.itemconfig(f"{vtag}{c}", text=str(v),
                                   fill="#101010" if k > 0.55 else "#8098b0")
            if max(vals) > thr:
                self.peak.config(text=f"板 {tag} 最亮：CH{peak_c:X}  = {max(vals)}")

        st = self.stat
        self.lbl.config(text=f"A={st['A']}  B={st['B']}  S={st['S']}  P={st['P']}")
        self.root.after(120, self._tick)

    def _log(self, m):
        self.log.insert("end", m + "\n")
        self.log.see("end")


if __name__ == "__main__":
    r = tk.Tk()
    LampApp(r)
    r.mainloop()
