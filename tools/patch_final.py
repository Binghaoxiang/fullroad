#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_final.py — E2 单边偏差控制，用 X1/P/X0 三段式驱动

每个控制周期：
  X1         使能 PWM
  P<pulse>   以偏离中点的脉宽起转（方向由误差符号决定）
  wait       ms = kp_ms * |err|，限幅
  X0         关闭 PWM -> 舵机停止

用 X0 断电停止，不需要知道舵机的精确中点脉宽。
"""

import io
import re

F = "tools/pos_ui.py"
s = io.open(F, "r", encoding="utf-8").read()

start = s.find("    def e2_x_mm(self):")
end = s.find("    def _send(self, cmd):")
if start > 0 and end > start:
    new = '''    def e2_x_mm(self):
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

'''
    s = s[:start] + new + s[end:]

# 参数
s = re.sub(r"self\.kp_ms = [0-9.]+", "self.kp_ms = 7.0", s)
s = re.sub(r"self\.min_ms = [0-9.]+", "self.min_ms = 100.0", s)
s = re.sub(r"self\.max_ms = [0-9.]+", "self.max_ms = 400.0", s)
s = re.sub(r"self\.dev_us = [0-9.]+", "self.dev_us = 500.0", s)
s = re.sub(r"self\.db_mm = [0-9.]+", "self.db_mm = 3.0", s)

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
