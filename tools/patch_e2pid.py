#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_e2pid.py — 用上长边 E2 单独做舵机 PID 控制"""

import io

F = "tools/pos_ui.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 替换 track_step：用 E2 ----------
start = s.find("    def track_step(self):")
end = s.find("    def _send(self, cmd):")
if start > 0 and end > start:
    new = '''    def e2_x_mm(self):
        """从上长边 E2 的重心算 X 坐标（mm）

        E2 的索引范围 22~63 逆序（D4F 左 → D35 右），
        所以数组位置 0 = 最左（X=0），位置 N-1 = 最右（X=200）。
        """
        pr = self.proc[2] if len(self.proc) > 2 else None
        if not pr or not pr.get("valid"):
            return None, 0
        pos = pr.get("mu")
        if pos is None:
            pos = pr.get("pos")
        if pos is None:
            return None, 0
        n = screen_pos.EDGES[2][2]          # 42
        u = pos / float(n - 1)
        u = max(0.0, min(1.0, u))
        return u * SCREEN_W_MM, float(pr.get("peak", 0.0))

    def track_step(self):
        """用上长边 E2 做单轴 PID 位置控制

        E2 信号最强最干净，用它的重心算 X。
        舵机上电 0 度 = 光在屏幕中心（X=100mm）。
        偏差 -> PID -> 角度增量 -> 发 A 命令。
        """
        if not self.tracking:
            return

        x, peak = self.e2_x_mm()
        if x is None:
            self.integral = 0.0
            self.prev_e = None
            self.stat.config(text="E2 无有效信号，等待…")
            return

        err = (SCREEN_W_MM / 2.0) - x        # 目标中心 X=100mm
        aerr = abs(err)

        # 死区
        if aerr < self.db_mm:
            self.integral = 0.0
            self.prev_e = None
            self.stat.config(text="已对准  X=%.1f  err=%+.1f  E2峰=%d"
                                  % (x, err, int(peak)))
            return

        # PID
        self.integral += err * 0.15
        self.integral = max(-300.0, min(300.0, self.integral))
        deriv = 0.0 if self.prev_e is None else (err - self.prev_e)
        self.prev_e = err

        u = (self.kp * err) + (self.ki * self.integral) + (self.kd * deriv)
        u = max(-self.out_max, min(self.out_max, u))

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
            text="E2-PID  X=%.1f  err=%+.1f  A=%.1f°  P=%d  dir=%s"
                 % (x, err, self.servo_deg, int(peak),
                    "+" if self.dir_sign > 0 else "-"))

'''
    s = s[:start] + new + s[end:]

# ---------- 2) 参数调整为适合 E2 控制 ----------
import re
s = re.sub(r"\n\s*self\.kp = [^\n]*", "\n        self.kp = 0.08          # 每 mm 误差对应多少度", s, count=1)
s = re.sub(r"\n\s*self\.ki = [^\n]*", "\n        self.ki = 0.010", s, count=1)
s = re.sub(r"\n\s*self\.kd = [^\n]*", "\n        self.kd = 0.030", s, count=1)
s = re.sub(r"\n\s*self\.out_max = [^\n]*", "\n        self.out_max = 6.0      # 单次最大角度变化", s, count=1)
s = re.sub(r"\n\s*self\.servo_deg = [^\n]*", "\n        self.servo_deg = 0.0    # 上电 0 度 = 屏幕中心", s, count=1)
s = re.sub(r"\n\s*self\.min_deg = [^\n]*", "\n        self.min_deg = -175.0", s, count=1)
s = re.sub(r"\n\s*self\.max_deg = [^\n]*", "\n        self.max_deg = 175.0", s, count=1)

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
