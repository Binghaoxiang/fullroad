#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spot_ui.py — 激光光斑位置 / 入射角 / 双边原始分布  实时监视

架构：
    后台线程：串口轮询（K BOTH 每周期，K INFO 每 5 周期）→ 写入共享状态
    主线程：  60 Hz 重绘，永不阻塞

算法：
    EMA 平滑 → 相对阈值 → 加权重心 → 抛物线峰值细化 → 亚探测器坐标
    入射角 = 舵机角 − 峰值舵机角（由 K INFO 提供）
"""

import sys
import time
import math
import threading
import tkinter as tk

try:
    import serial
except ImportError:
    sys.path.insert(0, "tools/_libs")
    import serial

PITCH_MM = 4.8
N_LONG = 43
N_SHORT = 18
SPAN_X = (N_LONG - 1) * PITCH_MM
SPAN_Y = (N_SHORT - 1) * PITCH_MM

W_TOTAL, H_TOTAL = 1360, 720
LEFT_W = 780
PANEL_W = 540
BAR_H = 46
FOOT_H = 96
BODY_H = H_TOTAL - BAR_H - FOOT_H

BG = "#0a0e14"
PANEL = "#111820"
CARD = "#0d1620"
EDGE = "#1e2a38"
TXT = "#e8eef6"
DIM = "#7d8fa3"
ACC = "#00d1ff"
WARN = "#ffb020"
GOOD = "#25c26e"
BAD = "#ff4d5e"

ALPHA = 0.55
THR_RATIO = 0.35
NOISE = 250
L_PARAM = 260.0      # 等效力臂 mm（用于 atan(Δx/L) 换算角度）
SERVO_PER_DEG = 8    # 每度舵机脉冲毫秒（与固件一致）
ADC_FULL = 4096.0

LONG_SILK = ["D25", "D35", "D45", "D16", "D26", "D36", "D46", "D17", "D27", "D37", "D47",
             "D18", "D28", "D38", "D48", "D19", "D29", "D39", "D49",
             "D1A", "D2A", "D3A", "D4A", "D1B", "D2B", "D3B", "D4B",
             "D1C", "D2C", "D3C", "D4C", "D1D", "D2D", "D3D", "D4D",
             "D1E", "D2E", "D3E", "D4E", "D1F", "D2F", "D3F", "D4F"]
SHORT_SILK = ["D35", "D45", "D16", "D26", "D36", "D46", "D17", "D27", "D37",
              "D47", "D18", "D28", "D38", "D48", "D19", "D29", "D39", "D49"]


class DSP:
    def __init__(self):
        self.ls = [0.0] * N_LONG
        self.ss = [0.0] * N_SHORT
        self.max_l = 0
        self.max_s = 0
        self.x_idx = None
        self.y_idx = None
        self.valid_l = False
        self.valid_s = False
        self.peak_ref = 0
        self.x_peak = None
        self.y_peak = None

    def update(self, raw_l, raw_s):
        for i in range(N_LONG):
            self.ls[i] = self.ls[i] * (1 - ALPHA) + raw_l[i] * ALPHA
        for i in range(N_SHORT):
            self.ss[i] = self.ss[i] * (1 - ALPHA) + raw_s[i] * ALPHA
        self.max_l, self.x_idx, self.valid_l = self._solve(self.ls)
        self.max_s, self.y_idx, self.valid_s = self._solve(self.ss)

    @staticmethod
    def _solve(arr):
        vals = [int(round(v)) for v in arr]
        n = len(vals)
        mx = max(vals) if vals else 0
        if mx < NOISE:
            return mx, None, False
        thr = max(mx * THR_RATIO, NOISE)
        s0 = 0.0
        s1 = 0.0
        for i, v in enumerate(vals):
            if v >= thr:
                s0 += v
                s1 += v * i
        if s0 <= 0:
            return mx, None, False
        c = s1 / s0

        k = vals.index(mx)
        if 0 < k < n - 1:
            y0, y1, y2 = vals[k - 1], vals[k], vals[k + 1]
            den = y0 - 2 * y1 + y2
            if den != 0:
                off = 0.5 * (y0 - y2) / den
                if abs(off) <= 0.5:
                    c = c * 0.6 + (k + off) * 0.4
        return mx, c, True

    def x_mm(self):
        return None if self.x_idx is None else self.x_idx * PITCH_MM

    def y_mm(self):
        return None if self.y_idx is None else self.y_idx * PITCH_MM

    def perpendicular(self):
        if self.peak_ref <= 0 or self.max_l <= 0:
            return None
        return min(1.0, self.max_l / self.peak_ref)

    def angle_from_pos(self):
        """几何法入射角：θ = atan((X − X_peak) / L)"""
        if self.x_peak is None or self.x_idx is None:
            return None
        dx = (self.x_idx - self.x_peak) * PITCH_MM
        return math.degrees(math.atan2(dx, L_PARAM))

    @staticmethod
    def width(arr):
        vals = [int(round(v)) for v in arr]
        s0 = sum(vals)
        if s0 <= 0:
            return 0.0
        m = sum(i * v for i, v in enumerate(vals)) / s0
        var = sum(v * (i - m) ** 2 for i, v in enumerate(vals)) / s0
        return math.sqrt(max(var, 0.0))


class Poller(threading.Thread):
    def __init__(self, app):
        super().__init__(daemon=True)
        self.app = app
        self.stop_flag = False
        self.n = 0

    def run(self):
        while not self.stop_flag:
            ser = self.app.ser
            if ser is None:
                time.sleep(0.15)
                continue
            ok = False
            try:
                ser.reset_input_buffer()
                ser.write(b"K BOTH\r\n")
                buf = ""
                lo = sh = None
                t0 = time.time()
                while time.time() - t0 < 0.5:
                    chunk = ser.read(512)
                    if not chunk:
                        if lo and sh:
                            break
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
                        break
                if lo and len(lo) == N_LONG and sh and len(sh) == N_SHORT:
                    self.app.dsp.update(lo, sh)
                    self.app.ok += 1
                    ok = True
            except Exception:
                pass
            if not ok:
                self.app.bad += 1

            self.n += 1
            if self.n % 5 == 0:
                try:
                    ser.write(b"K INFO\r\n")
                    buf = ""
                    t0 = time.time()
                    while time.time() - t0 < 1.2:
                        chunk = ser.read(256)
                        if not chunk:
                            continue
                        buf += chunk.decode("ascii", "ignore")
                        for line in buf.split("\r\n"):
                            if line.startswith("INFO"):
                                d = {}
                                for tok in line[4:].split():
                                    if "=" in tok:
                                        k, v = tok.split("=", 1)
                                        try:
                                            d[k] = int(v)
                                        except ValueError:
                                            pass
                                if d.get("C", 0):
                                    self.app.ang = d.get("ANG", 0) / 10.0
                                break
                        break
                except Exception:
                    pass
            time.sleep(0.02)


class App:
    def __init__(self, root, port):
        self.root = root
        self.ser = None
        self.dsp = DSP()
        self.ok = 0
        self.bad = 0
        self.ang = None
        self.poller = None
        self._last = 0.0

        root.title("激光光斑位置 · 入射角 · 双边原始分布")
        root.configure(bg=BG)
        root.geometry("%dx%d" % (W_TOTAL, H_TOTAL))
        root.resizable(False, False)

        bar = tk.Frame(root, bg=PANEL, height=BAR_H)
        bar.pack(fill="x", side="top")
        bar.pack_propagate(False)
        tk.Label(bar, text="端口", bg=PANEL, fg=DIM,
                 font=("Microsoft YaHei", 10)).pack(side="left", padx=(16, 4))
        self.port_var = tk.StringVar(value=port)
        tk.Entry(bar, textvariable=self.port_var, width=8, bg=CARD, fg=TXT,
                 insertbackground=TXT, relief="flat",
                 font=("Consolas", 11)).pack(side="left", ipady=3)
        self._btn(bar, "连接", self.connect, ACC)
        self._btn(bar, "断开", self.disconnect, DIM)
        self._btn(bar, "标定基准", self.do_cal, WARN)
        self.stat = tk.Label(bar, text="未连接", bg=PANEL, fg=DIM,
                             font=("Microsoft YaHei", 10))
        self.stat.pack(side="right", padx=18)

        foot = tk.Frame(root, bg=PANEL, height=FOOT_H)
        foot.pack(fill="x", side="bottom")
        foot.pack_propagate(False)
        self.lb_pos = tk.Label(foot, text="位置  --", bg=PANEL, fg=TXT,
                               font=("Microsoft YaHei", 19, "bold"))
        self.lb_pos.pack(side="left", padx=22)
        self.lb_ang = tk.Label(foot, text="入射角  --", bg=PANEL, fg=ACC,
                               font=("Microsoft YaHei", 24, "bold"))
        self.lb_ang.pack(side="left", padx=30)
        self.lb_extra = tk.Label(foot, text="", bg=PANEL, fg=DIM,
                                 font=("Consolas", 11))
        self.lb_extra.pack(side="right", padx=20)

        body = tk.Frame(root, bg=BG)
        body.pack(fill="both", expand=True)
        self.cv = tk.Canvas(body, width=LEFT_W, height=BODY_H,
                            bg=BG, highlightthickness=0)
        self.cv.pack(side="left")
        self.pv = tk.Canvas(body, width=PANEL_W, height=BODY_H,
                            bg=BG, highlightthickness=0)
        self.pv.pack(side="right")

        self.root.after(60, self.tick)

    def _btn(self, parent, text, cmd, color, w=8):
        b = tk.Button(parent, text=text, command=cmd, bg=CARD, fg=color,
                      activebackground=color, activeforeground=BG,
                      relief="flat", bd=0, width=w, cursor="hand2",
                      font=("Microsoft YaHei", 10, "bold"))
        b.pack(side="left", padx=5, pady=9)
        return b

    def connect(self):
        self.disconnect()
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200,
                                     timeout=0.06, write_timeout=0.4)
            time.sleep(0.35)
            self.ser.reset_input_buffer()
            self.stat.config(text="已连接")
            self.poller = Poller(self)
            self.poller.start()
        except Exception as exc:
            self.ser = None
            self.stat.config(text="失败: %s" % exc)

    def disconnect(self):
        if self.poller:
            self.poller.stop_flag = True
            self.poller = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.stat.config(text="未连接")

    def do_cal(self):
        m = max(self.dsp.max_l, self.dsp.max_s)
        if m > 0 and self.dsp.x_idx is not None:
            self.dsp.peak_ref = m
            self.dsp.x_peak = self.dsp.x_idx
            self.dsp.y_peak = self.dsp.y_idx
            self.stat.config(text="垂直基准已锁定：峰值 %d  X=%.1f" %
                             (m, self.dsp.x_peak * PITCH_MM))

    def tick(self):
        now = time.time()
        if now - self._last >= 0.033:
            self._last = now
            self.draw_left()
            self.draw_right()
            self.draw_foot()
        self.root.after(16, self.tick)

    def draw_left(self):
        c = self.cv
        c.delete("all")
        pad = 58
        x0, y0 = pad, 44
        w, h = LEFT_W - 2 * pad, BODY_H - y0 - 76
        c.create_text(22, 18, text="屏幕视野", fill=DIM, anchor="w",
                      font=("Microsoft YaHei", 10, "bold"))
        for i in range(3):
            c.create_rectangle(x0 - i * 3, y0 - i * 3, x0 + w + i * 3,
                               y0 + h + i * 3, outline=EDGE, width=1)
        for i in range(1, 8):
            gx = x0 + w * i / 8
            c.create_line(gx, y0, gx, y0 + h, fill="#141c26")
        for j in range(1, 4):
            gy = y0 + h * j / 4
            c.create_line(x0, gy, x0 + w, gy, fill="#141c26")
        c.create_text(x0, y0 + h + 16, text="0", fill=DIM, font=("Consolas", 9), anchor="w")
        c.create_text(x0 + w, y0 + h + 16, text="%.0f mm" % SPAN_X, fill=DIM,
                      font=("Consolas", 9), anchor="e")
        c.create_text(x0 - 10, y0 + h, text="0", fill=DIM, font=("Consolas", 9), anchor="e")
        c.create_text(x0 - 10, y0, text="%.0f" % SPAN_Y, fill=DIM,
                      font=("Consolas", 9), anchor="e")
        c.create_text(x0 + w / 2, y0 + h + 40, text="长边(X) → 水平　短边(Y) → 垂直",
                      fill="#3c4a5a", font=("Microsoft YaHei", 9))

        xm, ym = self.dsp.x_mm(), self.dsp.y_mm()
        if xm is None and ym is None:
            c.create_text(x0 + w / 2, y0 + h / 2, text="未检测到激光",
                          fill="#33404f", font=("Microsoft YaHei", 24, "bold"))
            return

        px = x0 + (xm / SPAN_X) * w if xm is not None else x0 + w / 2
        py = y0 + h - (ym / SPAN_Y) * h if ym is not None else y0 + h / 2
        px = max(x0 + 5, min(x0 + w - 5, px))
        py = max(y0 + 5, min(y0 + h - 5, py))

        for r, col in ((74, "#06222e"), (48, "#0a3747"), (27, "#0e5266")):
            c.create_oval(px - r, py - r, px + r, py + r, fill=col, outline="")
        if xm is not None:
            c.create_line(px, y0, px, y0 + h, fill="#123a4a", dash=(3, 7))
        if ym is not None:
            c.create_line(x0, py, x0 + w, py, fill="#123a4a", dash=(3, 7))
        c.create_oval(px - 15, py - 15, px + 15, py + 15,
                      fill=ACC, outline="#9beeff", width=2)
        c.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#ffffff", outline="")

        ang = self.dsp.angle_from_pos()
        if ang is None:
            ang = self.ang
        if ang is not None:
            txt = "θ = %+.1f°" % ang
            col = GOOD if abs(ang) < 3 else (WARN if abs(ang) < 8 else BAD)
        else:
            p = self.dsp.perpendicular()
            if p is not None and self.dsp.peak_ref > 0:
                txt = "垂直度 %.0f%%" % (p * 100)
                col = GOOD if p > 0.95 else (WARN if p > 0.85 else BAD)
            else:
                txt, col = '点击「标定基准」', DIM

        bx = px + 26
        if bx + 168 > x0 + w:
            bx = px - 26 - 168
        by = max(y0 + 4, py - 54)
        c.create_rectangle(bx, by, bx + 168, by + 50, fill=CARD, outline=EDGE)
        c.create_text(bx + 10, by + 8, text=txt, fill=col, anchor="nw",
                      font=("Microsoft YaHei", 16, "bold"))
        sub = ""
        if xm is not None:
            sub += "X=%.1f " % xm
        if ym is not None:
            sub += "Y=%.1f" % ym
        c.create_text(bx + 10, by + 31, text=sub + " mm", fill=DIM, anchor="nw",
                      font=("Consolas", 9))

    def draw_right(self):
        c = self.pv
        c.delete("all")
        pad = 22
        w = PANEL_W - 2 * pad
        self._profile(c, pad, 52, w, 148, self.dsp.ls, self.dsp.max_l,
                      "长边 43 点（X 分布，按 PCB 顺序）", ACC,
                      self.dsp.width(self.dsp.ls), self.dsp.x_idx, LONG_SILK)
        self._profile(c, pad, 302, w, 108, self.dsp.ss, self.dsp.max_s,
                      "短边 18 点（Y 分布，按 PCB 顺序）", WARN,
                      self.dsp.width(self.dsp.ss), self.dsp.y_idx, SHORT_SILK)

        sy = 500
        c.create_rectangle(pad, sy, pad + w, sy + 56, fill=CARD, outline=EDGE)
        p = self.dsp.perpendicular()
        if p is not None:
            col = GOOD if p > 0.95 else WARN if p > 0.85 else BAD
            c.create_text(pad + 12, sy + 10, text="垂直度", fill=DIM, anchor="nw",
                          font=("Microsoft YaHei", 9))
            c.create_text(pad + 12, sy + 27, text="%.0f%%" % (p * 100), fill=col,
                          anchor="nw", font=("Microsoft YaHei", 18, "bold"))
            bw = w - 150
            c.create_rectangle(pad + 110, sy + 30, pad + 110 + bw, sy + 40,
                               fill="#16202c", outline="")
            c.create_rectangle(pad + 110, sy + 30,
                               pad + 110 + int(bw * p), sy + 40, fill=col, outline="")
        else:
            c.create_text(pad + 12, sy + 16, text="点「标定基准」记录垂直状态",
                          fill=DIM, anchor="nw", font=("Microsoft YaHei", 10))

    def _profile(self, c, x, y, w, h, arr, maxv, title, color, width, center, labels):
        c.create_text(x, y - 26, text=title, fill=DIM, anchor="w",
                      font=("Microsoft YaHei", 10, "bold"))
        n = len(arr)
        gap = 1 if n > 20 else 3
        bw = (w - gap * (n - 1)) / n
        base = y + h
        for f in (0.25, 0.5, 0.75, 1.0):
            gy = base - h * f
            c.create_line(x, gy, x + w, gy, fill="#141c26")
            c.create_text(x + w + 4, gy, text="%d" % int(ADC_FULL * f),
                          fill="#2c3745", anchor="w", font=("Consolas", 7))
        c.create_line(x, base, x + w, base, fill=EDGE)

        lit_any = False
        for i, v in enumerate(arr):
            bh = (v / ADC_FULL) * h
            if bh < 0.4:
                continue
            bx = x + i * (bw + gap)
            col = color if v >= NOISE else "#243140"
            c.create_rectangle(bx, base - bh, bx + bw, base, fill=col, outline="")
            if v >= NOISE:
                lit_any = True

        for i, lb in enumerate(labels):
            if i >= n:
                break
            bx = x + i * (bw + gap) + bw / 2
            c.create_text(bx, base + 3, text=lb, fill="#44566a", angle=90,
                          anchor="n", font=("Consolas", 6))

        if center is not None and lit_any:
            cx = x + (center + 0.5) * (bw + gap)
            c.create_line(cx, y - 8, cx, base, fill="#ff5566", width=2)
            c.create_polygon(cx - 5, y - 14, cx + 5, y - 14, cx, y - 6,
                             fill="#ff5566", outline="")

        if not lit_any:
            c.create_text(x, base + 46,
                          text="无有效光：峰值 %d（阈值 %d）" % (maxv, NOISE),
                          fill="#4a5a6c", anchor="w", font=("Consolas", 9))
            return

        info = "峰值 %d　展宽 %.2f" % (maxv, width)
        if center is not None:
            info += "　重心 %.2f" % (center + 1)
        c.create_text(x, base + 46, text=info, fill="#6e8092", anchor="w",
                      font=("Consolas", 9))
        order = sorted(range(n), key=lambda k: arr[k], reverse=True)[:3]
        parts = ["%s=%d" % (labels[k], int(arr[k]))
                 for k in order if k < len(labels) and arr[k] >= NOISE]
        if parts:
            c.create_text(x, base + 62, text="　".join(parts), fill=color,
                          anchor="w", font=("Consolas", 9))

    def draw_foot(self):
        xm, ym = self.dsp.x_mm(), self.dsp.y_mm()
        if xm is not None and ym is not None:
            self.lb_pos.config(text="位置  X=%6.1f  Y=%5.1f mm" % (xm, ym))
        else:
            self.lb_pos.config(text="位置  --")
        ang = self.dsp.angle_from_pos()
        if ang is None:
            ang = self.ang
        if ang is not None:
            self.lb_ang.config(text="入射角  %+.1f°" % ang,
                               fg=GOOD if abs(ang) < 3 else
                               (WARN if abs(ang) < 8 else BAD))
        else:
            self.lb_ang.config(text="入射角  --（需先标定）", fg=DIM)
        self.lb_extra.config(text="长边峰 %d　短边峰 %d　OK %d / BAD %d"
                                  % (self.dsp.max_l, self.dsp.max_s, self.ok, self.bad))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    root = tk.Tk()
    App(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
