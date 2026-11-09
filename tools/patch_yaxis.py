#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_yaxis.py — 把 pos_ui.py 的控制轴从 X 改成 Y（用两个短边），并实现 PID"""

import io
import re

F = "tools/pos_ui.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 参数：PID + 舵机中位 ----------
s = re.sub(r"\n\s*self\.kp_us = [^\n]*", "\n        self.kp = 0.06          # PID: 每 mm 误差对应多少度", s, count=1)
s = re.sub(r"\n\s*self\.max_dev = [^\n]*", "\n        self.ki = 0.008         # 积分项", s, count=1)
s = re.sub(r"\n\s*self\.min_dev = [^\n]*", "\n        self.kd = 0.02          # 微分项", s, count=1)
s = re.sub(r"\n\s*self\.max_deg = [^\n]*", "\n        self.max_deg = 175.0", s, count=1)
s = re.sub(r"\n\s*self\.min_deg = [^\n]*", "\n        self.min_deg = 5.0", s, count=1)
s = re.sub(r"\n\s*self\.db_mm = [^\n]*", "\n        self.db_mm = 2.0        # 死区 mm", s, count=1)
s = re.sub(r"\n\s*self\.servo_deg = [^\n]*", "\n        self.servo_deg = 90.0   # 上电中位 = 屏幕中心", s, count=1)

# ---------- 2) 增加 PID 状态 ----------
if "self.integral" not in s:
    s = s.replace("        self.last_pulse = 1500",
                  "        self.last_pulse = 1500\n"
                  "        self.integral = 0.0\n"
                  "        self.prev_e = None\n"
                  "        self.out_max = 8.0      # 单次最大角度变化")

# ---------- 3) 重写 track_step 为 Y 轴 PID ----------
start = s.find("    def track_step(self):")
end = s.find("    def _send(self, cmd):")
if start > 0 and end > start:
    new = '''    def track_step(self):
        """Y 轴单轴 PID 位置控制

        用两个短边（E1 右 + E3 左）融合出 Y，目标 Y = 50mm（屏幕中心）。
        误差 -> PID -> 舵机角度增量 -> 发 A 命令。
        舵机上电 90 度 = 屏幕中心。
        """
        if not self.tracking:
            return

        # ---- 取 Y（只用两个短边）----
        y = None
        if self.pos and self.pos.valid:
            ys = []
            ws = []
            if self.pos.y1 is not None:
                ys.append(self.pos.y1)
                ws.append(self.pos.wy1)
            if self.pos.y3 is not None:
                ys.append(self.pos.y3)
                ws.append(self.pos.wy3)
            if ys and sum(ws) > 0:
                y = sum(a * b for a, b in zip(ys, ws)) / sum(ws)
            elif ys:
                y = sum(ys) / len(ys)

        if y is None:
            self.integral = 0.0
            self.prev_e = None
            self.stat.config(text="等待 Y 数据（需要短边有信号）")
            return

        # ---- 误差 ----
        err = (SCREEN_H_MM / 2.0) - y      # 目标 Y = 50mm
        aerr = abs(err)

        if aerr < self.db_mm:
            self.integral = 0.0
            self.prev_e = None
            self.stat.config(text="已对准  Y=%.1f  err=%+.1f" % (y, err))
            return

        # ---- PID ----
        self.integral += err * 0.15
        self.integral = max(-200.0, min(200.0, self.integral))
        deriv = 0.0 if self.prev_e is None else (err - self.prev_e)
        self.prev_e = err

        u = (self.kp * err) + (self.ki * self.integral) + (self.kd * deriv)
        u = max(-self.out_max, min(self.out_max, u))

        # ---- 方向 + 限幅 ----
        delta = u * self.dir_sign
        new_deg = self.servo_deg + delta
        if new_deg > self.max_deg:
            new_deg = self.max_deg
        elif new_deg < self.min_deg:
            new_deg = self.min_deg
        delta = new_deg - self.servo_deg
        if abs(delta) < 0.05:
            return

        self.servo_deg = new_deg
        self._send("A %.2f" % self.servo_deg)
        self.last_cmd = time.time()

        self.stat.config(
            text="Y轴跟随  Y=%.1f  err=%+.1f  A=%.1f°  dir=%s"
                 % (y, err, self.servo_deg,
                    "+" if self.dir_sign > 0 else "-"))

'''
    s = s[:start] + new + s[end:]

# ---------- 4) 方向自动学习：改成跟踪 |err| 的 3 次滑动 ----------
if "self.hist_err" not in s:
    s = s.replace("        self.integral = 0.0",
                  "        self.integral = 0.0\n        self.hist_err = []")

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
