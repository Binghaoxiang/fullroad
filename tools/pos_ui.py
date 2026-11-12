#!/usr/bin/env python3

# -*- coding: utf-8 -*-

"""

pos_ui.py — 屏幕坐标定位可视化



左侧：200 × 100 mm 屏幕视图，实时显示光斑位置 + 十字准线 + 坐标标注

右侧：四条边的强度分布（紧凑显示）+ 一致性/质量指标

底部：X / Y / 质量 大字读数



数据来源：K EDGES（四条边的原始 ADC 数组）

处理链：edge_dsp（基线/中值/平滑/重心/高斯）+ screen_pos（融合定位）

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



# ---------------- 屏幕物理尺寸 ----------------

SCREEN_W_MM = 200.0

SCREEN_H_MM = 100.0



# ---------------- 布局 ----------------

W, H = 1420, 820

BAR_H = 46

FOOT_H = 96



# ---------------- 配色 ----------------

BG = "#0a0e14"

PANEL = "#111820"

CARD = "#0d1620"

EDGE_C = "#1e2a38"

TXT = "#e8eef6"

DIM = "#7d8fa3"

ACC = "#00d1ff"

WARN = "#ffb020"

GOOD = "#25c26e"

BAD = "#ff4d5e"



EDGE_COLORS = ["#00d1ff", "#25c26e", "#ffb020", "#ff4d5e"]






# 四条边的坐标校准（offset_mm, scale）
# 依据实测：E0 比 E2 偏小 11.1mm，E1 比 E3 偏大 13.7mm
# scale=1.0 表示只做平移修正；后续可用两点法精调
CAL_TABLE = {
    0: (0.0, 1.0),     # 下边：映射已修正，无需偏移
    1: (0.0, 1.0),     # 右边：映射已修正，无需偏移
    2: (0.0, 1.0),     # 上长边：基准
    3: (0.0, 1.0),     # 左短边：基准
}
class App:

    def __init__(self, root, port):

        screen_pos.CAL.update(CAL_TABLE)
        self.root = root

        self.ser = None

        self.ok = 0

        self.bad = 0

        self.raw = [None] * 4

        self.proc = [None] * 4

        self.base = [None] * 4

        self.pos = None
        self.sm_x = None        # 时域平滑后的 X
        self.sm_y = None        # 时域平滑后的 Y

        self.last = 0.0

        self.locator = screen_pos.ScreenLocator(SCREEN_W_MM, SCREEN_H_MM)

        # ---- 跟随控制 ----

        self.tracking = False
        self.fw_x = None
        self.fw_y = None
        self.servo_deg = 90.0   # 中位（可向两侧各调 ±85 度）

        self.dir_sign = 1.0
        self.kp_ms = 7.0        # 每 mm 误差 → 多少毫秒转动
        self.min_ms = 100.0      # 最短转动
        self.max_ms = 400.0     # 最长转动
        self.dev_us = 500.0     # 脉宽偏离量（越大转越快）
        self.last_moved = False
        self.worse_count = 0
        self.db_mm = 3.0        # 死区
        self.kp = 0.50          # 每 mm 误差对应多少度（放大以便可见）
        self.ki = 0.010
        self.kd = 0.030

        self.last_cmd = 0.0

        self.calibrating = 0
        self.prev_err = None
        self.prev_step_sign = 0
        self.worse_count = 0
        self.max_deg = 175.0
        self.min_deg = 5.0
        self.fast_x = None        # 快速 X（mm）
        self.fast_peak = 0
        self.fast_age = 0.0



        root.title("屏幕坐标定位监视  200 × 100 mm")

        root.configure(bg=BG)

        root.geometry("%dx%d" % (W, H))

        root.resizable(False, False)



        # ---- 顶栏 ----

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

        self.btn_track = tk.Button(bar, text="开始跟随", command=self.toggle_track,

                                   bg=CARD, fg=GOOD, activebackground=GOOD,

                                   activeforeground=BG, relief="flat", bd=0,

                                   width=10, cursor="hand2",

                                   font=("Microsoft YaHei", 10, "bold"))

        self.btn_track.pack(side="left", padx=5, pady=9)


        self.stat = tk.Label(bar, text="未连接", bg=PANEL, fg=DIM,

                             font=("Microsoft YaHei", 10))

        self.stat.pack(side="right", padx=18)



        # ---- 底部读数 ----

        foot = tk.Frame(root, bg=PANEL, height=FOOT_H)

        foot.pack(fill="x", side="bottom")

        foot.pack_propagate(False)

        self.lb_pos = tk.Label(foot, text="X = --    Y = --",

                               bg=PANEL, fg=TXT,

                               font=("Consolas", 30, "bold"))

        self.lb_pos.pack(side="left", padx=24)

        self.lb_meta = tk.Label(foot, text="", bg=PANEL, fg=DIM,

                                font=("Consolas", 12))

        self.lb_meta.pack(side="right", padx=24)



        # ---- 主体 ----

        body = tk.Frame(root, bg=BG)

        body.pack(fill="both", expand=True)



        self.cv = tk.Canvas(body, width=1000, height=H - BAR_H - FOOT_H,

                            bg=BG, highlightthickness=0)

        self.cv.pack(side="left")



        self.pv = tk.Canvas(body, width=W - 1000, height=H - BAR_H - FOOT_H,

                            bg=BG, highlightthickness=0)

        self.pv.pack(side="right")



        self.root.after(100, self.tick)



    def _btn(self, p, text, cmd, color, w=8):

        b = tk.Button(p, text=text, command=cmd, bg=CARD, fg=color,

                      activebackground=color, activeforeground=BG,

                      relief="flat", bd=0, width=w, cursor="hand2",

                      font=("Microsoft YaHei", 10, "bold"))

        b.pack(side="left", padx=5, pady=9)



    # ---------------- 串口 ----------------

    def connect(self):

        self.disconnect()

        try:

            self.ser = serial.Serial(self.port_var.get(), 115200,

                                     timeout=0.06, write_timeout=0.4)

            time.sleep(0.35)

            self.ser.reset_input_buffer()

            self.stat.config(text="已连接")
            try:
                self.ser.write(b"K OFF\r\n")
                time.sleep(0.15)
                self.ser.write(b"X1\r\n")     # 使能舵机 PWM
                time.sleep(0.15)
                time.sleep(0.2)
                self.ser.reset_input_buffer()
            except Exception:
                pass

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

            got = {}


            # ---- 跟随模式：高频读 K X（约 60ms/次）----
            if self.tracking:
                try:
                    s.reset_input_buffer()
                    s.write(b"K X\r\n")
                    fb = ""
                    t0f = time.time()
                    while time.time() - t0f < 0.20:
                        cc = s.read(128)
                        if not cc:
                            if "X " in fb:
                                break
                            continue
                        fb += cc.decode("ascii", "ignore")
                        if "\r\n" in fb:
                            break
                    for line in fb.split("\r\n"):
                        if line.startswith("X "):
                            pp = line[2:].split()
                            if len(pp) >= 4:
                                pos10 = int(pp[1])
                                self.fast_peak = int(pp[2])
                                if pos10 > 0:
                                    idx = pos10 / 10.0
                                    vx = (63.0 - idx) / (63.0 - 22.0) * SCREEN_W_MM
                                    self.fast_x = max(0.0, min(SCREEN_W_MM, vx))
                                    self.fast_age = time.time()
                            break
                except Exception:
                    pass
            try:

                s.reset_input_buffer()

                s.write(b"K EDGES\r\n")

                buf = ""

                t0 = time.time()

                while time.time() - t0 < 2.5:

                    c = s.read(1024)

                    if not c:

                        if len(got) >= 4:

                            break

                        continue

                    buf += c.decode("ascii", "ignore")

                    for _pl in buf.split("\r\n"):

                        if _pl.startswith("POS "):

                            _p2 = _pl[4:].split()

                            if len(_p2) >= 2:

                                try:

                                    _fx = float(_p2[0]) / 10.0

                                    _fy = SCREEN_H_MM - float(_p2[1]) / 10.0   # 屏幕坐标 Y 向下为正，固件 Y 向上为正，此处翻转

                                    self.fw_x = _fx

                                    self.fw_y = _fy

                                    if self.pos is None:

                                        self.pos = screen_pos.PosResult()

                                    self.pos.x_mm = _fx

                                    self.pos.y_mm = _fy

                                    self.pos.valid = True

                                except Exception:

                                    pass


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

                # 数据已由固件处理完毕，UI 只做显示（不再调用 edge_dsp / screen_pos）
                for k, v in got.items():

                    self.raw[k] = v

                    self.proc[k] = {

                        "valid": len(v) > 0,

                        "corr": [float(x) for x in v],

                        "peak": max(v) if v else 0,

                        "peak_idx": (v.index(max(v)) if v else 0),

                        "peak_width": 0, "centroid": 0.0, "sigma": None,

                        "fit": None, "u": 0.0, "mu": 0.0, "sigma": 0.0, "centroid": 0.0,
                        "peak_width": 0, "corr": [float(x) for x in v],

                    }

                # 位置由固件的 POS 行给出（上方已解析到 self.pos）


                if got:

                    self.ok += 1

                else:

                    self.bad += 1

            except Exception:

                pass

            time.sleep(0.04)



    # ---------------- 刷新 ----------------

    # ---------------- 跟随控制 ----------------

    def toggle_track(self):
        self.tracking = not self.tracking

        if self.tracking:

            self.btn_track.config(text="停止跟随", fg=BAD)

            self.stat.config(text="跟随中")

        else:

            self.btn_track.config(text="开始跟随", fg=GOOD)

            self.stat.config(text="已停止")



    def start_calib(self):

        """校准：记录当前 X，正向转 2 度，看 X 往哪边动"""

        if not (self.pos and self.pos.valid):

            self.stat.config(text="无有效定位，无法校准")

            return

        self.calibrating = 1

        self.calib_x0 = self.fast_x if self.fast_x is not None else self.pos.x_mm

        self.calib_deg0 = self.servo_deg

        self.servo_deg += 2.0

        self._send("A %.1f" % self.servo_deg)

        self.stat.config(text="校准中：已转 +2°")



    def finish_calib(self):

        self.calibrating = 0
        self.fast_x = None        # 快速 X（mm）
        self.fast_peak = 0
        self.fast_age = 0.0

        if not (self.pos and self.pos.valid):

            self.stat.config(text="校准失败：无有效定位")

            return

        dx = self.pos.x_mm - self.calib_x0

        if abs(dx) < 1.0:

            self.dir_sign = 1.0

            self.stat.config(text="校准失败：X 变化不足（%.1fmm）" % dx)

            return

        # dx>0 表示角度增大使 X 增大；我们需要 angle += sign*err

        # 若 err = 100 - x > 0 需要 X 增大 -> angle 增大 -> sign = +1

        self.dir_sign = 1.0 if dx > 0 else -1.0

        self.stat.config(text="校准完成：X 每度变化 %.2fmm  方向 %s"

                              % (dx / 2.0, "+" if dx > 0 else "-"))



    def _pulse(self, us):
        """发送脉宽命令"""
        us = int(round(us))
        if us < 1000:
            us = 1000
        if us > 2000:
            us = 2000
        self._send("P%d" % us)
        self.last_cmd = time.time()
        self.last_pulse = us

    def e2_x_mm(self):
        """从上长边 E2 的重心算 X 坐标（mm）

        E2 索引 22~63 逆序（D4F 左 → D35 右），
        数组位置 0 = 最左 X=0，位置 N-1 = 最右 X=200。
        """
        pr = self.proc[2] if len(self.proc) > 2 else None
        if not pr or not pr.get("valid"):
            return None, 0
        pos = pr.get("mu")
        if pos is None:
            pos = pr.get("pos")
        if pos is None:
            return None, 0
        n = screen_pos.EDGES[2][2]
        u = pos / float(n - 1)
        u = max(0.0, min(1.0, u))
        return u * SCREEN_W_MM, float(pr.get("peak", 0.0))

    def _burst(self, pulse, ms):
        """X1 -> P<pulse> -> 保持 ms -> X0"""
        self._send("X1")
        time.sleep(0.02)
        self._send("P%d" % int(pulse))
        t0 = time.time()
        while (time.time() - t0) * 1000.0 < ms:
            if not self.tracking:
                break
            time.sleep(0.02)
        self._send("X0")          # 断电停止

    def track_step(self):
        """E2 单边偏差 -> 方向 + 时长 -> 三段式脉冲驱动"""
        if not self.tracking:
            return

        x, peak = self.e2_x_mm()
        if x is None:
            self._send("X0")
            self.prev_e = None
            self.stat.config(text="E2 无有效信号，等待…")
            return

        err = (SCREEN_W_MM / 2.0) - x
        aerr = abs(err)

        # ---- 死区 ----
        if aerr < self.db_mm:
            self._send("X0")
            self.prev_e = aerr
            self.worse_count = 0
            self.last_moved = False
            self.stat.config(text="已对准  X=%.1f  err=%+.1f  E2峰=%d"
                                  % (x, err, int(peak)))
            return

        # ---- 方向自学习 ----
        if self.prev_e is not None and getattr(self, "last_moved", False):
            if aerr > self.prev_e + 1.5:
                self.worse_count = getattr(self, "worse_count", 0) + 1
                if self.worse_count >= 2:
                    self.dir_sign = -self.dir_sign
                    self.worse_count = 0
                    self.last_moved = False
                    self.prev_e = aerr
                    self.stat.config(text="方向翻转 -> %s"
                                          % ("+" if self.dir_sign > 0 else "-"))
                    return
            else:
                self.worse_count = 0
        self.prev_e = aerr
        self.last_moved = False

        # ---- 转动时长（比例） ----
        ms = self.kp_ms * aerr
        ms = max(self.min_ms, min(self.max_ms, ms))

        # ---- 方向：err>0 表示光在左，需要往右推 ----
        direction = 1.0 if err > 0 else -1.0
        pulse = 1375 - int(self.dev_us * direction * self.dir_sign)

        self._burst(pulse, ms)
        self.last_moved = True

        self.stat.config(
            text="移动  X=%.1f  err=%+.1f  P=%d  %dms  dir=%s"
                 % (x, err, pulse, int(ms),
                    "+" if self.dir_sign > 0 else "-"))

    def _send(self, cmd):

        s = self.ser

        if s is None:

            return

        try:

            s.write((cmd + "\r\n").encode("ascii"))

        except Exception:

            pass



    def tick(self):

        now = time.time()

        if now - self.last > 0.04:

            self.last = now

            self.draw_screen()

            self.draw_edges()

            self.draw_foot()

        self.root.after(25, self.tick)



    # ---------------- 左：屏幕视图 ----------------

    def _smooth_pos(self):
        """时域平滑 + 跳变抑制，抑制远端弱信号造成的漂移"""
        p = self.pos
        if not p or not p.valid:
            return
        if self.sm_x is None or self.sm_y is None:
            self.sm_x = p.x_mm
            self.sm_y = p.y_mm
            return
        dx = p.x_mm - self.sm_x
        dy = p.y_mm - self.sm_y
        # 跳变过大 -> 只走一小部分（抑制突发跳变）
        # 静止时重平均，移动时快速跟随
        def _alpha(d):
            ad = abs(d)
            if ad < 3.0:
                return 0.12      # 静止：重平均，压制跳动
            if ad < 15.0:
                return 0.35      # 缓慢移动
            if ad < 25.0:
                return 0.50
            return 0.60          # 快速移动：跟上
        ax = _alpha(dx)
        ay = _alpha(dy)
        self.sm_x += dx * ax
        self.sm_y += dy * ay
        p.x_mm = self.sm_x
        p.y_mm = self.sm_y

    def draw_screen(self):

        c = self.cv

        c.delete("all")

        Hh = H - BAR_H - FOOT_H



        c.create_text(24, 18, text="屏幕视野  %.0f × %.0f mm"

                      % (SCREEN_W_MM, SCREEN_H_MM),

                      fill=DIM, anchor="w",

                      font=("Microsoft YaHei", 11, "bold"))



        # 屏幕矩形：2:1 比例

        mx, my = 70, 60

        sw = 1000 - 2 * mx

        sh = sw * SCREEN_H_MM / SCREEN_W_MM

        if my + sh > Hh - 70:

            sh = Hh - 70 - my

            sw = sh * SCREEN_W_MM / SCREEN_H_MM

            mx = (1000 - sw) / 2

        x0, y0 = mx, my



        # 外框

        for i in range(3):

            c.create_rectangle(x0 - i * 3, y0 - i * 3,

                               x0 + sw + i * 3, y0 + sh + i * 3,

                               outline=EDGE_C, width=1)



        # 网格 20mm × 10mm

        for gx in range(0, int(SCREEN_W_MM) + 1, 20):

            px = x0 + sw * gx / SCREEN_W_MM

            c.create_line(px, y0, px, y0 + sh,

                          fill="#1a2431" if gx % 100 else "#25384a")

        for gy in range(0, int(SCREEN_H_MM) + 1, 10):

            py = y0 + sh * gy / SCREEN_H_MM

            c.create_line(x0, py, x0 + sw, py,

                          fill="#1a2431" if gy % 50 else "#25384a")



        # 标尺

        for gx in range(0, int(SCREEN_W_MM) + 1, 50):

            px = x0 + sw * gx / SCREEN_W_MM

            c.create_text(px, y0 + sh + 14, text="%d" % gx, fill="#5a6b7e",

                          font=("Consolas", 8))

        for gy in range(0, int(SCREEN_H_MM) + 1, 25):

            py = y0 + sh * gy / SCREEN_H_MM

            c.create_text(x0 - 18, py, text="%d" % gy, fill="#5a6b7e",

                          font=("Consolas", 8), anchor="e")

        c.create_text(x0 + sw / 2, y0 + sh + 32, text="X (mm)", fill=DIM,

                      font=("Microsoft YaHei", 9))

        c.create_text(x0 - 46, y0 + sh / 2, text="Y\n(mm)", fill=DIM,

                      font=("Microsoft YaHei", 9), justify="center")



        # 四边标注

        c.create_text(x0 + sw / 2, y0 - 20, text="E2 上长边",

                      fill=EDGE_COLORS[2], font=("Microsoft YaHei", 9))

        c.create_text(x0 + sw / 2, y0 + sh + 50, text="E0 下长边",

                      fill=EDGE_COLORS[0], font=("Microsoft YaHei", 9))

        c.create_text(x0 + sw + 30, y0 + sh / 2, text="E1\n右\n短\n边",

                      fill=EDGE_COLORS[1], font=("Microsoft YaHei", 9),

                      justify="center")

        c.create_text(x0 - 74, y0 + sh / 2, text="E3\n左\n短\n边",

                      fill=EDGE_COLORS[3], font=("Microsoft YaHei", 9),

                      justify="center")



        if not (self.pos and self.pos.valid):

            c.create_text(x0 + sw / 2, y0 + sh / 2,

                          text="未定位到光斑",

                          fill="#33404f",

                          font=("Microsoft YaHei", 26, "bold"))

            c.create_text(x0 + sw / 2, y0 + sh / 2 + 40,

                          text="需要四条边都有有效信号",

                          fill="#2a3542", font=("Microsoft YaHei", 11))

            return



        p = self.pos

        # 坐标 → 画布（Y 轴翻转：物理 Y 向上，画布 Y 向下）

        px = x0 + sw * (p.x_mm / SCREEN_W_MM)

        py = y0 + 14 + (sh - 28) * (p.y_mm / SCREEN_H_MM)   # Y 大 -> 下方（与 E3 标记一致）

        px = max(x0 + 4, min(x0 + sw - 4, px))

        py = max(y0 + 4, min(y0 + sh - 4, py))



        # 光晕

        for r, col in ((72, "#06222e"), (48, "#0a3747"), (28, "#0e5266")):

            c.create_oval(px - r, py - r, px + r, py + r,

                          fill=col, outline="")



        # 十字准线

        c.create_line(px, y0, px, y0 + sh, fill="#12495e", dash=(3, 6))

        c.create_line(x0, py, x0 + sw, py, fill="#12495e", dash=(3, 6))



        # 光点

        c.create_oval(px - 16, py - 16, px + 16, py + 16,

                      fill=ACC, outline="#9beeff", width=2)

        c.create_oval(px - 6, py - 6, px + 6, py + 6,

                      fill="#ffffff", outline="")



        # 坐标标签

        qc = GOOD if p.quality > 0.8 else (WARN if p.quality > 0.5 else BAD)

        bx = px + 30

        if bx + 190 > x0 + sw:

            bx = px - 30 - 190

        by = py - 60

        if by < y0 + 4:

            by = y0 + 4

        c.create_rectangle(bx, by, bx + 190, by + 56,

                           fill=CARD, outline=EDGE_C)

        c.create_text(bx + 10, by + 8,

                      text="X=%.1f  Y=%.1f mm" % (p.x_mm, p.y_mm),

                      fill=qc, anchor="nw",

                      font=("Consolas", 13, "bold"))

        c.create_text(bx + 10, by + 32,

                      text="一致性 %.0f%%  质量 %.0f%%"

                           % (p.consistency * 100, p.quality * 100),

                      fill=DIM, anchor="nw", font=("Consolas", 9))



        # 两条边各自估计的位置（辅助核对）

        if p.x0 is not None:

            lx = x0 + sw * (p.x0 / SCREEN_W_MM)

            c.create_line(lx, y0 + sh, lx, y0 + sh + 8,

                          fill=EDGE_COLORS[0], width=3)

        if p.x2 is not None:

            lx = x0 + sw * (p.x2 / SCREEN_W_MM)

            c.create_line(lx, y0 - 8, lx, y0,

                          fill=EDGE_COLORS[2], width=3)

        if p.y1 is not None:

            ly = y0 + sh * (p.y1 / SCREEN_H_MM)   # E1 绿标常规绘制

            c.create_line(x0 + sw, ly, x0 + sw + 8, ly,

                          fill=EDGE_COLORS[1], width=3)

        if p.y3 is not None:

            ly = y0 + sh * (p.y3 / SCREEN_H_MM)

            c.create_line(x0 - 8, ly, x0, ly,

                          fill=EDGE_COLORS[3], width=3)



    # ---------------- 右：四边信号 ----------------

    def draw_edges(self):

        c = self.pv

        c.delete("all")

        Hh = H - BAR_H - FOOT_H

        pad = 16

        w = (W - 1000) - 2 * pad

        top = 44

        per = (Hh - top - 130) / 4.0



        c.create_text(pad, 18, text="四条边信号", fill=DIM, anchor="w",

                      font=("Microsoft YaHei", 11, "bold"))



        for k in range(4):

            y = top + k * per

            tag = screen_pos.EDGES[k][1]

            col = EDGE_COLORS[k]

            pr = self.proc[k]



            c.create_text(pad, y, text="E%d %s" % (k, tag),

                          fill=col, anchor="w",

                          font=("Microsoft YaHei", 9, "bold"))

            if pr and pr["valid"]:

                c.create_text(pad + w, y, text="峰 %d" % int(pr["peak"]),

                              fill=col, anchor="e",

                              font=("Consolas", 9))

            else:

                c.create_text(pad + w, y, text="--", fill=DIM, anchor="e",

                              font=("Consolas", 9))



            gy = y + 16

            gh = per - 40

            gb = gy + gh

            raw = self.raw[k] or []

            n = max(len(raw), 1)



            data = [float(x) for x in raw] if raw else [0.0]
            # 显示方向统一：垂直边（E1/E3）让「屏幕顶部」都落在波形左边
            #   E3 索引 0 = 顶部 -> 不翻
            #   E1 索引 0 = 底部 -> 翻转
            # 右短边（E1）波形显示方向翻转：使其与左短边一致
            if False:   # 波形翻转已撤销
                pass

            vmax = max(data) if data else 1.0

            if vmax < 120:

                vmax = 120.0

            vmax *= 1.25



            c.create_line(pad, gb, pad + w, gb, fill="#22303e")



            if raw and len(raw) >= 2:

                step = w / float(n - 1)

                pts = []

                for i in range(n):

                    pxx = pad + i * step

                    pyy = gb - min(gh, (data[i] / vmax) * gh)

                    pts.extend([pxx, pyy])

                c.create_line(*pts, fill=col, width=1.5)



                # 重心标记

                if pr and pr["valid"] and pr["mu"] is not None:

                    mxx = pad + pr["mu"] * step

                    c.create_line(mxx, gy - 2, mxx, gb, fill="#ffffff",

                                  width=1)



        # 指标区

        by = Hh - 110

        c.create_rectangle(pad, by, pad + w, by + 96,

                           fill=CARD, outline=EDGE_C)

        if self.pos and self.pos.valid:

            p = self.pos

            c.create_text(pad + 12, by + 10, text="一致性",

                          fill=DIM, anchor="nw",

                          font=("Microsoft YaHei", 9))

            cw = w - 90

            c.create_rectangle(pad + 70, by + 12, pad + 70 + cw, by + 22,

                               fill="#16202c", outline="")

            c.create_rectangle(pad + 70, by + 12,

                               pad + 70 + int(cw * p.consistency), by + 22,

                               fill=GOOD if p.consistency > 0.9 else WARN,

                               outline="")

            c.create_text(pad + 12, by + 32, text="质量",

                          fill=DIM, anchor="nw",

                          font=("Microsoft YaHei", 9))

            c.create_rectangle(pad + 70, by + 34, pad + 70 + cw, by + 44,

                               fill="#16202c", outline="")

            c.create_rectangle(pad + 70, by + 34,

                               pad + 70 + int(cw * p.quality), by + 44,

                               fill=GOOD if p.quality > 0.8 else WARN,

                               outline="")

            c.create_text(pad + 12, by + 60,

                          text="E0 X=%.1f   E2 X=%.1f"

                               % (p.x0 or 0, p.x2 or 0),

                          fill="#6e8092", anchor="nw",

                          font=("Consolas", 8))

            c.create_text(pad + 12, by + 76,

                          text="E1 Y=%.1f   E3 Y=%.1f"

                               % (p.y1 or 0, p.y3 or 0),

                          fill="#6e8092", anchor="nw",

                          font=("Consolas", 8))

        else:

            c.create_text(pad + w / 2, by + 44,

                          text="等待四条边数据…", fill=DIM,

                          font=("Microsoft YaHei", 10))



    # ---------------- 底部 ----------------

    def draw_foot(self):

        if self.pos and self.pos.valid:

            p = self.pos

            self.lb_pos.config(text="X = %6.1f    Y = %6.1f  mm"

                                    % (p.x_mm, p.y_mm))

            self.lb_meta.config(

                text="一致性 %3.0f%%   质量 %3.0f%%      OK %d / BAD %d"

                     % (p.consistency * 100, p.quality * 100,

                        self.ok, self.bad))

        else:

            self.lb_pos.config(text="X =   --      Y =   --  mm")

            self.lb_meta.config(text="OK %d / BAD %d" % (self.ok, self.bad))





def main():

    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"

    root = tk.Tk()

    App(root, port)

    root.mainloop()





if __name__ == "__main__":

    main()


