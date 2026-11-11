#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""collect_ui.py — 标定数据采集界面

流程：
  屏幕上依次高亮 N 个目标点（默认 5x5 = 25 点，也可切成 3x3）
  每一步：
    ① 大字提示「请把激光移到 ★ 左下角」
    ② 等待信号稳定（连续若干帧波形稳定）
    ③ 自动采集 M 帧
    ④ 进入下一个点
  采完保存 data/calib.json，内含：
    每点 (x_mm, y_mm) 真实坐标
    每次采样的四条边原始波形
"""

import json
import os
import sys
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial
import edge_dsp

W_MM, H_MM = 200.0, 100.0

BG = "#0d1117"
FG = "#e6edf3"
GOOD = "#3fb950"
WARN = "#d29922"
BAD = "#f85149"
GRID = "#30363d"
DIM = "#8b949e"

EDGE_COLORS = ["#58a6ff", "#3fb950", "#f0c674", "#ff7b72"]

# 5x5 网格（含四角与中心），坐标按屏幕 mm
GRID_N = 3
FRAMES_PER_POINT = 12


def build_points(n):
    pts = []
    for j in range(n):                      # y: 上->下
        for i in range(n):                  # x: 左->右
            x = 20.0 + (W_MM - 40.0) * i / (n - 1)
            y = 20.0 + (H_MM - 40.0) * j / (n - 1)
            pts.append((x, y))
    return pts


def name_of(x, y, n):
    hs = []
    if x < W_MM / 3:
        hs.append("左")
    elif x > W_MM * 2 / 3:
        hs.append("右")
    else:
        hs.append("中")
    if y < H_MM / 3:
        hs.append("上")
    elif y > H_MM * 2 / 3:
        hs.append("下")
    hs.append("部" if len(hs) > 1 else "")
    return "".join(hs)


class Collector:
    def __init__(self, root, port):
        self.root = root
        self.port = port
        self.ser = None
        self.points = build_points(GRID_N)
        self.idx = 0
        self.frames = []
        self.all = []
        self.state = "idle"        # idle | wait | collect | done
        self.n_got = 0

        root.title("探测器标定数据采集  %d x %d 点" % (GRID_N, GRID_N))
        root.geometry("1100x760")
        root.configure(bg=BG)

        top = tk.Frame(root, bg=BG)
        top.pack(fill="x", padx=14, pady=8)
        tk.Label(top, text="端口", bg=BG, fg=DIM).pack(side="left")
        self.e_port = tk.Entry(top, width=8, bg="#161b22", fg=FG,
                               insertbackground=FG, relief="flat")
        self.e_port.insert(0, port)
        self.e_port.pack(side="left", padx=6)
        self.b_conn = tk.Button(top, text="连接", bg="#238636", fg="white",
                                relief="flat", command=self.connect)
        self.b_conn.pack(side="left", padx=6)
        self.b_start = tk.Button(top, text="开始标定", bg="#1f6feb", fg="white",
                                 relief="flat", state="disabled", command=self.start)
        self.b_start.pack(side="left", padx=6)
        self.b_save = tk.Button(top, text="保存", bg="#6e40c9", fg="white",
                                relief="flat", command=self.save)
        self.b_save.pack(side="left", padx=6)

        tk.Label(top, text="采集进度", bg=BG, fg=DIM).pack(side="left", padx=(24, 4))
        self.lb_prog = tk.Label(top, text="0 / %d" % len(self.points),
                                bg=BG, fg=FG, font=("Consolas", 11, "bold"))
        self.lb_prog.pack(side="left")

        # 主提示
        self.lb_hint = tk.Label(root, text="点【连接】，然后点【开始标定】",
                                bg=BG, fg=WARN, font=("Microsoft YaHei", 24, "bold"))
        self.lb_hint.pack(pady=10)

        self.lb_sub = tk.Label(root, text="", bg=BG, fg=DIM,
                               font=("Microsoft YaHei", 12))
        self.lb_sub.pack()

        # 画布
        self.cv = tk.Canvas(root, bg="#010409", highlightthickness=0)
        self.cv.pack(fill="both", expand=True, padx=14, pady=10)
        self.cv.bind("<Configure>", lambda e: self.draw())

        self.lb_foot = tk.Label(root, text="等待开始", bg=BG, fg=DIM,
                                font=("Consolas", 10), anchor="w")
        self.lb_foot.pack(fill="x", padx=14, pady=(0, 8))

        self.cur = None            # 当前帧的四边原始值
        self.root.after(120, self.tick)

    # ---------------------------------------------------------- 串口
    def connect(self):
        try:
            self.ser = serial.Serial(self.e_port.get().strip(), 115200, timeout=0.1)
            time.sleep(0.8)
            self.ser.reset_input_buffer()
            self.ser.write(b"K OFF\r\n")
            time.sleep(0.2)
            self.ser.write(b"X1\r\n")
            self.b_start.config(state="normal")
            self.b_conn.config(text="已连接", bg="#30363d", state="disabled")
            self.lb_hint.config(text="已连接，点【开始标定】", fg=GOOD)
        except Exception as ex:
            self.lb_hint.config(text="连接失败: %s" % ex, fg=BAD)

    # ---------------------------------------------------------- 采集流程
    def start(self):
        self.idx = 0
        self.all = []
        self.frames = []
        self.state = "wait"
        self.wait_t0 = time.time()
        self.lb_hint.config(text="准备…", fg=WARN)

    def read_frame(self):
        s = self.ser
        if s is None:
            return None
        s.reset_input_buffer()
        s.write(b"K EDGES\r\n")
        buf = ""
        t0 = time.time()
        while time.time() - t0 < 6.0:
            c = s.read(256)
            if c:
                buf += c.decode("ascii", "ignore")
                if buf.count("\r\n") >= 4:
                    break
            elif buf.count("\r\n") >= 4:
                break
            else:
                time.sleep(0.004)
        out = {}
        for line in buf.split("\r\n"):
            line = line.strip()
            if len(line) < 3 or line[0] != "E" or line[2] != ":":
                continue
            try:
                tag = int(line[1])
                parts = line[3:].split()
                cnt = int(parts[0])
                v = [int(x) for x in parts[1:1 + cnt]]
                if len(v) == cnt:
                    out[tag] = v
            except Exception:
                pass
        return out if len(out) == 4 else None

    def stability(self, hist):
        """用最近几帧的重心变化判断光点是否稳定"""
        if len(hist) < 3:
            return 99.0
        cur = hist[-1]
        prev = hist[-3]
        d = 0.0
        c = 0
        for e in range(4):
            a, b = cur[e], prev[e]
            if a is not None and b is not None:
                d += abs(a - b)
                c += 1
        return d / c if c else 99.0

    def tick(self):
        try:
            if self.state == "wait":
                f = self.read_frame()
                if f is not None:
                    self.cur = f
                    self.draw()
                # 提示阶段：显示目标点，等操作者移动
                if time.time() - self.wait_t0 > 3.5:
                    self.state = "collect"
                    self.frames = []
                    self.n_got = 0

            elif self.state == "collect":
                f = self.read_frame()
                if f is not None:
                    self.cur = f
                    self.frames.append(f)
                    self.n_got = len(self.frames)
                    self.draw()
                if self.n_got >= FRAMES_PER_POINT:
                    x, y = self.points[self.idx]
                    self.all.append({"x_mm": x, "y_mm": y,
                                     "point": self.idx,
                                     "frames": self.frames})
                    self.save(silent=True)
                    self.idx += 1
                    self.frames = []
                    if self.idx >= len(self.points):
                        self.state = "done"
                    else:
                        self.state = "wait"
                        self.wait_t0 = time.time()

            elif self.state == "done":
                pass
        except Exception as ex:
            self.lb_foot.config(text="错误: %s" % ex)
        self.root.after(60, self.tick)

    # ---------------------------------------------------------- 绘图
    def geom(self):
        w = self.cv.winfo_width()
        h = self.cv.winfo_height()
        m = 46
        if w < 200 or h < 120:
            return None
        s = min((w - 2 * m) / W_MM, (h - 2 * m) / H_MM)
        ox = (w - W_MM * s) / 2
        oy = (h - H_MM * s) / 2
        return (s, ox, oy)

    def to_px(self, x, y):
        g = self.geom()
        if not g:
            return None
        s, ox, oy = g
        return (ox + x * s, oy + (H_MM - y) * s)

    def draw(self):
        c = self.cv
        c.delete("all")
        g = self.geom()
        if not g:
            return
        s, ox, oy = g
        w = self.cv.winfo_width()

        # 屏幕框
        c.create_rectangle(ox, oy, ox + W_MM * s, oy + H_MM * s,
                           outline="white", width=2)
        for i in range(1, 10):
            xx = ox + W_MM * s * i / 10.0
            c.create_line(xx, oy, xx, oy + H_MM * s,
                          fill=GRID if i % 5 else GRID, width=1)
        for i in range(1, 5):
            yy = oy + H_MM * s * i / 5.0
            c.create_line(ox, yy, ox + W_MM * s, yy, fill=GRID, width=1)

        # 所有目标点
        for k, (x, y) in enumerate(self.points):
            p = self.to_px(x, y)
            if not p:
                continue
            done = k < self.idx
            cur = (k == self.idx and self.state in ("wait", "collect"))
            col = GOOD if done else (WARN if cur else DIM)
            r = 11 if cur else 5
            c.create_oval(p[0] - r, p[1] - r, p[0] + r, p[1] + r,
                          outline=col, width=3 if cur else 1,
                          fill=col if cur else "")
            c.create_text(p[0], p[1] - 20, text=str(k + 1),
                          fill=col, font=("Consolas", 9, "bold"))

        # 当前目标高亮
        if self.state in ("wait", "collect") and self.idx < len(self.points):
            x, y = self.points[self.idx]
            p = self.to_px(x, y)
            if p:
                r = 22
                c.create_oval(p[0] - r, p[1] - r, p[0] + r, p[1] + r,
                              outline=WARN, width=3)

        # 光点位置（用四条边重心粗算，仅作参考）
        if self.cur:
            xs, ys = [], []
            for e in range(4):
                pr = edge_dsp.process_edge(self.cur.get(e, []), None)
                if not pr["valid"] or pr["mu"] is None:
                    continue
                n = len(self.cur.get(e, []))
                u = pr["mu"] / max(1, n - 1)
                if e in (0, 2):
                    xs.append(u * W_MM)
                else:
                    ys.append(u * H_MM)
            if xs:
                x0 = sum(xs) / len(xs)
                y0 = sum(ys) / len(ys) if ys else H_MM / 2
                p = self.to_px(x0, y0)
                if p:
                    c.create_oval(p[0] - 9, p[1] - 9, p[0] + 9, p[1] + 9,
                                  outline="#39d353" if False else "#58a6ff", width=2)
                    c.create_text(p[0], p[1] + 22, text="光点(粗算)",
                                  fill=DIM, font=("Microsoft YaHei", 8))

        # 右侧波形
        self.draw_waves(w)

        # 文案
        if self.state == "idle":
            self.lb_hint.config(text="点【连接】，然后点【开始标定】", fg=WARN)
            self.lb_sub.config(text="")
        elif self.state == "wait":
            x, y = self.points[self.idx]
            self.lb_hint.config(
                text="%d/%d  请把激光移到【%s】" % (
                    self.idx + 1, len(self.points), name_of(x, y, GRID_N)),
                fg=WARN)
            self.lb_sub.config(text="目标坐标  X=%.0f mm  Y=%.0f mm     保持不动，即将开始采集…"
                                    % (x, y))
        elif self.state == "collect":
            x, y = self.points[self.idx]
            self.lb_hint.config(
                text="%d/%d  正在采集  %d/%d" % (
                    self.idx + 1, len(self.points), self.n_got, FRAMES_PER_POINT),
                fg=GOOD)
            self.lb_sub.config(text="目标  X=%.0f mm  Y=%.0f mm   请保持激光不动" % (x, y))
        else:
            self.lb_hint.config(text="✅ 采集完成  共 %d 点" % len(self.all), fg=GOOD)
            self.lb_sub.config(text="点下方按钮保存")

        self.lb_prog.config(text="%d / %d" % (min(self.idx, len(self.points)),
                                              len(self.points)))
        if self.state == "done":
            if not hasattr(self, "_saved"):
                self._saved = True
                self.save()
        self.lb_foot.config(text="状态: %s   当前帧: %s"
                                 % (self.state, "有" if self.cur else "无"))

    def draw_waves(self, w):
        c = self.cv
        if not self.cur:
            return
        x0 = w - 330
        y0 = 14
        box_h = 96
        for e in range(4):
            v = self.cur.get(e, [])
            if not v:
                continue
            yy = y0 + e * (box_h + 8)
            c.create_rectangle(x0, yy, w - 14, yy + box_h,
                               outline=GRID, width=1)
            mx = max(v) if max(v) > 0 else 1
            n = len(v)
            pts = []
            for i, val in enumerate(v):
                px = x0 + 6 + (w - 20 - x0 - 12) * i / max(1, n - 1)
                py = yy + box_h - 8 - (box_h - 16) * val / mx
                pts += [px, py]
            if len(pts) >= 4:
                c.create_line(*pts, fill=EDGE_COLORS[e], width=2)
            c.create_text(x0 + 8, yy + 10, anchor="w",
                          text="E%d  峰 %d" % (e, max(v)),
                          fill=EDGE_COLORS[e], font=("Consolas", 9))

    # ---------------------------------------------------------- 保存
    def save(self, silent=False):
        os.makedirs("data", exist_ok=True)
        fn = "data/calib.json"
        with open(fn, "w", encoding="utf-8") as f:
            json.dump({"grid": GRID_N, "w_mm": W_MM, "h_mm": H_MM,
                       "frames_per_point": FRAMES_PER_POINT,
                       "points": self.all}, f, ensure_ascii=False)
        if not silent:
            self.lb_foot.config(text="已保存到 %s   共 %d 点 %d 帧"
                                     % (fn, len(self.all),
                                        sum(len(p["frames"]) for p in self.all)))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    root = tk.Tk()
    Collector(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
