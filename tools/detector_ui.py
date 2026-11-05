#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
探测器实时监视 UI —— 按实际安装位置显示

    ┌──────────────────────────────────────┐
    │ ████████████████████████████████████  │ ← 长边 43 只 (D25~D4F)
    │ █                                     │
    │ █                                     │
    │ █   50 寸光学膜                        │
    │ █                                     │
    │ █  短边 21 只 (D35~D49)                │
    └──────────────────────────────────────┘

用法:  python detector_ui.py
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
POLL_MS = 350


def designators(a, b):
    """'D25'..'D4F' -> [(row 1..4, col 0..15), ...]"""
    def val(s):
        return (int(s[1], 16) << 4) | int(s[2], 16)
    lo, hi = val(a), val(b)
    if lo > hi:
        lo, hi = hi, lo
    return [((d >> 4), (d & 0x0F)) for d in range(lo, hi + 1)]


LONG_SEQ = designators("D25", "D4F")     # 长边 43 只（板 A）
SHORT_SEQ = designators("D35", "D49")    # 短边 21 只（板 B）


class App:
    def __init__(self, root):
        self.root = root
        root.title("探测器实时监视 · 按实际安装位置")
        root.configure(bg="#0b0f14")

        self.ser = None
        self.running = False
        self.q = queue.Queue()
        self.data_a = None
        self.data_b = None
        self.cells_a = [[0] * COLS for _ in range(ROWS)]
        self.cells_b = [[0] * COLS for _ in range(ROWS)]
        self.stat = {"A": 0, "B": 0, "S": 0, "P": 0}

        self._build_ui()
        self._scan_ports()
        self._tick()

    def _build_ui(self):
        top = tk.Frame(self.root, bg="#121a24", pady=8, padx=12)
        top.pack(fill="x")
        tk.Label(top, text="串口", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.cb = ttk.Combobox(top, width=10, state="readonly")
        self.cb.pack(side="left", padx=6)
        tk.Button(top, text="刷新", command=self._scan_ports).pack(side="left")
        self.btn = tk.Button(top, text="连接", command=self._toggle,
                             bg="#1c72d4", fg="white", width=7)
        self.btn.pack(side="left", padx=8)
        self.lbl = tk.Label(top, text="A=0  B=0  S=0  P=0", fg="#4fd1c5",
                            bg="#121a24", font=("Consolas", 11, "bold"))
        self.lbl.pack(side="left", padx=14)
        tk.Label(top, text="阈值", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.th = tk.IntVar(value=600)
        tk.Scale(top, from_=0, to=4000, orient="horizontal", variable=self.th,
                 length=150, bg="#121a24", fg="#d7e3ee",
                 troughcolor="#1d2a39", highlightthickness=0).pack(side="left")

        self.W, self.H = 980, 560
        self.cv = tk.Canvas(self.root, width=self.W, height=self.H,
                            bg="#0b0f14", highlightthickness=0)
        self.cv.pack(padx=10, pady=8)

        self.mx, self.my = 60, 46
        sw, sh = self.W - 2 * self.mx, self.H - 2 * self.my
        self.sw, self.sh = sw, sh
        self.cv.create_rectangle(self.mx, self.my, self.mx + sw, self.my + sh,
                                 fill="#0e1620", outline="#2a3d52", width=2)
        self.cv.create_text(self.mx + sw / 2, self.my + sh / 2,
                            text="50 寸光学膜", fill="#1e3044",
                            font=("Microsoft YaHei", 22, "bold"))

        n = len(LONG_SEQ)
        cw = sw / n
        self.long_items = [
            self.cv.create_rectangle(self.mx + i * cw + 0.5, self.my - 18,
                                     self.mx + (i + 1) * cw - 0.5, self.my - 3,
                                     fill="#16222f", outline="#1b2836")
            for i in range(n)]

        m = len(SHORT_SEQ)
        ch = sh / m
        self.short_items = [
            self.cv.create_rectangle(self.mx - 18, self.my + i * ch + 0.5,
                                     self.mx - 3, self.my + (i + 1) * ch - 0.5,
                                     fill="#16222f", outline="#1b2836")
            for i in range(m)]

        self.cv.create_text(self.mx + sw / 2, self.my - 28,
                            text=f"长边 {n} 只", fill="#5f7c98",
                            font=("Consolas", 9))
        self.cv.create_text(self.mx - 34, self.my + sh / 2, text=f"短边 {m} 只",
                            fill="#5f7c98", font=("Consolas", 9), angle=90)

        self.txt = tk.Label(self.root, text="", fg="#ffd166", bg="#0b0f14",
                            font=("Consolas", 10))
        self.txt.pack()

        self.log = tk.Text(self.root, height=4, bg="#080c11", fg="#48627d",
                           font=("Consolas", 9))
        self.log.pack(fill="x", padx=10, pady=(4, 8))

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
        self._log(f"已连接 {dev} @115200")
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
                self.ser.write(b"K RAW\r\n")
                t0 = time.time()
                while time.time() - t0 < 0.5:
                    n = self.ser.in_waiting
                    if n:
                        for ln in self.ser.read(n).decode("utf-8", "ignore").splitlines():
                            self.q.put(ln)
                    else:
                        time.sleep(0.01)
            except Exception as e:
                self.q.put(f"__ERR__ {e}")
                break
            time.sleep(POLL_MS / 1000.0)

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
        if s.startswith("A=") and " B=" in s and " P=" in s:
            try:
                d = dict(x.split("=") for x in s.split() if "=" in x)
                for k in ("A", "B", "S", "P"):
                    self.stat[k] = int(d.get(k, 0))
            except Exception:
                pass
            if self.data_a and len(self.data_a) == ROWS:
                self.cells_a = self.data_a
            if self.data_b and len(self.data_b) == ROWS:
                self.cells_b = self.data_b
            self.data_a = self.data_b = None

    def _color(self, v, vmax):
        k = min(1.0, v / max(1, vmax))
        return "#%02x%02x%02x" % (int(22 + k * 233), int(34 + k * 190),
                                  int(47 + k * 60))

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
        vmax = 1
        for seq, cells in ((LONG_SEQ, self.cells_a), (SHORT_SEQ, self.cells_b)):
            for (r, c) in seq:
                vmax = max(vmax, cells[r - 1][c])

        hits = []
        for i, (r, c) in enumerate(LONG_SEQ):
            v = self.cells_a[r - 1][c]
            lit = v >= thr
            self.cv.itemconfig(self.long_items[i], fill=self._color(v, vmax),
                               outline="#ffd166" if lit else "#1b2836",
                               width=2 if lit else 1)
            if lit:
                hits.append(f"长D{r}{c:X}")
        for i, (r, c) in enumerate(SHORT_SEQ):
            v = self.cells_b[r - 1][c]
            lit = v >= thr
            self.cv.itemconfig(self.short_items[i], fill=self._color(v, vmax),
                               outline="#ffd166" if lit else "#1b2836",
                               width=2 if lit else 1)
            if lit:
                hits.append(f"短D{r}{c:X}")

        st = self.stat
        self.lbl.config(text=f"A={st['A']}  B={st['B']}  S={st['S']}  P={st['P']}")
        self.txt.config(text=("命中: " + "  ".join(hits[:12])) if hits else "")
        self.root.after(120, self._tick)

    def _log(self, m):
        self.log.insert("end", m + "\n")
        self.log.see("end")


if __name__ == "__main__":
    r = tk.Tk()
    App(r)
    r.mainloop()
