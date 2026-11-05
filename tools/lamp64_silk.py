#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
探测器 64 灯 · PCB 顺序 · 每灯带丝印位号

物理顺序（PCB 从左到右）：D4F D3F D2F D1F | D4E D3E D2E D1E | ... | D40 D30 D20 D10

用法:  python lamp64_silk.py
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
    print("缺少 pyserial")
    raise

ROWS, COLS = 4, 16
N = ROWS * COLS

# PCB 顺序：CH0 在物理最左（D4F 端），CH15 在最右（D10 端）
# 屏幕左 → 右：CH0,CH1,...,CH15；每列内行 4→1
ORDER = [(r, c) for c in range(COLS) for r in range(ROWS - 1, -1, -1)]


def silk(i):
    r, c = ORDER[i]
    return "D%d%X" % (r + 1, (COLS - 1) - c)   # CH0 ↔ 丝印 F 列


class App:
    def __init__(self, root):
        self.root = root
        root.title("探测器 64 · PCB 顺序 · 带丝印")
        root.configure(bg="#0b0f14")

        self.ser = None
        self.running = False
        self.q = queue.Queue()
        self.da = None
        self.db = None
        self.ca = [[0] * COLS for _ in range(ROWS)]
        self.cb = [[0] * COLS for _ in range(ROWS)]
        self.stat = {"A": 0, "B": 0, "S": 0, "P": 0}
        self.stage = "IDLE"
        self.buf_a = None
        self.buf_b = None
        self.frames_ok = 0
        self.frames_bad = 0
        self.lock = threading.Lock()

        self._build()
        self._scan_ports()
        self._tick()

    def _build(self):
        top = tk.Frame(self.root, bg="#121a24", pady=6, padx=10)
        top.pack(fill="x")
        tk.Label(top, text="串口", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.cb_port = ttk.Combobox(top, width=7, state="readonly")
        self.cb_port.pack(side="left", padx=5)
        tk.Button(top, text="刷新", command=self._scan_ports).pack(side="left")
        self.btn = tk.Button(top, text="连接", command=self._toggle,
                             bg="#1c72d4", fg="white", width=5)
        self.btn.pack(side="left", padx=6)
        self.lbl = tk.Label(top, text="A=0 B=0 S=0 P=0", fg="#4fd1c5",
                            bg="#121a24", font=("Consolas", 10, "bold"))
        self.lbl.pack(side="left", padx=10)
        tk.Label(top, text="阈值", fg="#8fb3d6", bg="#121a24").pack(side="left")
        self.th = tk.IntVar(value=1)
        tk.Scale(top, from_=0, to=500, orient="horizontal", variable=self.th,
                 length=120, bg="#121a24", fg="#d7e3ee",
                 troughcolor="#1d2a39", highlightthickness=0).pack(side="left")

        # 画布
        self.CW, self.CH = 14, 86         # 每格宽/高
        W = 60 + N * (self.CW + 2)
        self.cv = tk.Canvas(self.root, width=W, height=300, bg="#0b0f14",
                            highlightthickness=0)
        self.cv.pack(padx=6, pady=4)

        self.peak = tk.Label(self.root, text="", fg="#ffd166", bg="#0b0f14",
                             font=("Consolas", 20, "bold"))
        self.peak.pack()
        self.log = tk.Text(self.root, height=3, bg="#080c11", fg="#48627d",
                           font=("Consolas", 8))
        self.log.pack(fill="x", padx=6, pady=(2, 5))

        self._draw()

    def _draw(self):
        self.cv.delete("all")
        x0 = 52
        self.rect = {"A": [], "B": []}
        self.val = {"A": [], "B": []}

        for tag, y in (("A", 34), ("B", 176)):
            self.cv.create_text(24, y + self.CH / 2, text=tag, fill="#5f7c98",
                                font=("Consolas", 13, "bold"))
            for i in range(N):
                x = x0 + i * (self.CW + 2)
                self.rect[tag].append(self.cv.create_rectangle(
                    x, y, x + self.CW, y + self.CH,
                    fill="#101a24", outline="#1b2836"))
                # 数值（格内上方，竖排）
                self.val[tag].append(self.cv.create_text(
                    x + self.CW / 2, y + 16, text="",
                    fill="#90a8c0", font=("Consolas", 6), angle=90))
                # 丝印位号（格内下方，竖排）
                self.cv.create_text(x + self.CW / 2, y + self.CH - 16,
                                    text=silk(i), fill="#3f5a75",
                                    font=("Consolas", 6), angle=90)

    def _scan_ports(self):
        ps = [p.device for p in list_ports.comports()]
        self.cb_port["values"] = ps
        if ps:
            self.cb_port.set(ps[0] if "COM5" not in ps else "COM5")
        self._log("串口: " + (", ".join(ps) if ps else "无"))

    def _toggle(self):
        if self.running:
            self._stop()
            return
        dev = self.cb_port.get()
        if not dev:
            messagebox.showwarning("提示", "选择串口")
            return
        try:
            self.ser = serial.Serial(dev, 115200, timeout=0.2)
        except Exception as e:
            messagebox.showerror("打开失败", "%s\n%s" % (dev, e))
            return
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.running = True
        self.btn.config(text="断开", bg="#8c2b2b")
        self._log("已连接 " + dev)
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
        """连续读取，不清缓冲；按需发送 K RAW，避免帧被截断"""
        buf = ""
        need_cmd = True
        while self.running:
            try:
                if need_cmd:
                    self.ser.write(b"K RAW\r\n")
                    need_cmd = False
                n = self.ser.in_waiting
                if n:
                    buf += self.ser.read(n).decode("utf-8", "ignore")
                    while "\n" in buf:
                        ln, buf = buf.split("\n", 1)
                        self.q.put(ln.rstrip("\r"))
                else:
                    time.sleep(0.004)
            except Exception as ex:
                self.q.put("__ERR__ " + str(ex))
                break
            # 解析器空闲时再发下一条命令
            if self.stage == "IDLE":
                need_cmd = True
    def _parse(self, s):
        """严格状态机：只接受 #A -> R1..R4 -> #B -> R1..R4 -> A= 的完整帧"""
        s = s.strip()
        if not s:
            return
        if s.startswith("# A"):
            self.stage = "A"
            self.buf_a = []
            self.buf_b = None
            return
        if s.startswith("# B"):
            if self.stage == "A" and self.buf_a is not None and len(self.buf_a) == ROWS:
                self.stage = "B"
                self.buf_b = []
            else:
                self.stage = "BAD"
            return
        if s[:2] in ("R1", "R2", "R3", "R4") and len(s) > 2 and s[2] == " ":
            try:
                nums = [int(x) for x in s[3:].split()][:COLS]
            except ValueError:
                self.stage = "BAD"
                return
            if len(nums) < COLS:
                self.stage = "BAD"
                return
            if self.stage == "A" and len(self.buf_a) < ROWS:
                self.buf_a.append(nums)
            elif self.stage == "B" and self.buf_b is not None and len(self.buf_b) < ROWS:
                self.buf_b.append(nums)
            else:
                self.stage = "BAD"
            return
        if s.startswith("A=") and " B=" in s:
            if (self.stage == "B" and self.buf_a is not None and len(self.buf_a) == ROWS
                    and self.buf_b is not None and len(self.buf_b) == ROWS):
                self.ca = self.buf_a
                self.cb = self.buf_b
                self.frames_ok += 1
            else:
                self.frames_bad += 1
            for kv in s.split():
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    try:
                        self.stat[k] = int(v)
                    except ValueError:
                        pass
            self.stage = "IDLE"
            self.buf_a = None
            self.buf_b = None
            return
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
        for tag, cells in (("A", self.ca),):
            vals = []
            for (r, c) in ORDER:
                v = cells[r][c]
                vals.append(v)
            # 每行独立归一化（四行灵敏度差异很大，不归一化会呈条纹）
            rowmax = [max(1, max(cells[r])) for r in range(ROWS)]
            vmax = max(1, max(vals))
            pi = vals.index(max(vals))
            for i, v in enumerate(vals):
                r, c = ORDER[i]
                if v <= 0:
                    self.cv.itemconfig(self.rect[tag][i], fill="#0e161f",
                                       outline="#16202b")
                    self.cv.itemconfig(self.val[tag][i], text="")
                    continue
                k = min(1.0, v / rowmax[r])
                col = "#%02x%02x%02x" % (int(20 + k * 235),
                                         int(40 + k * 180),
                                         int(60 + k * 40))
                lit = (i == pi) and (v > 0)
                self.cv.itemconfig(self.rect[tag][i], fill=col,
                                   outline="#ffd166" if lit else "#243447",
                                   width=2 if lit else 1)
                self.cv.itemconfig(self.val[tag][i], text=str(v),
                                   fill="#0a0a0a" if k > 0.5 else "#c8d8e8")
            if max(vals) > 0:
                info.append("%s 最亮 %s = %d" % (tag, silk(pi), max(vals)))
        self.peak.config(text="    ".join(info))
        st = self.stat
        self.lbl.config(text="A=%d  S=%d  P=%d   帧 OK=%d BAD=%d" % (st["A"], st["S"], st["P"], self.frames_ok, self.frames_bad))
        self.root.after(110, self._tick)

    def _log(self, m):
        self.log.insert("end", m + "\n")
        self.log.see("end")


if __name__ == "__main__":
    r = tk.Tk()
    App(r)
    r.mainloop()
