#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_ctrl.py — 舵机「移动-测量」闭环控制

适配连续旋转舵机 + 约 1 秒的测量周期：
  1. 读上长边 E2 算 X
  2. err = 100 - X
  3. |err| < 死区 → P1500 停住
  4. 否则按 |err| 决定转动时长（PID 的比例项 → 时间）
  5. P<偏离> 起转 → 等 duration → P1500 停
  6. 下一轮重新测量

方向自学习：若移动后误差变大两次，翻转方向。
"""

import io
import re

F = "tools/pos_ui.py"
s = io.open(F, "r", encoding="utf-8").read()

start = s.find("    def track_step(self):")
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

    def _pulse_and_wait(self, us, ms):
        """发脉宽，保持 ms 毫秒，然后回中停住"""
        self._send("P%d" % int(us))
        t0 = time.time()
        while time.time() - t0 < ms / 1000.0:
            if not self.tracking:
                break
            time.sleep(0.02)
        self._send("P1500")

    def track_step(self):
        """移动-测量闭环控制"""
        if not self.tracking:
            return

        x, peak = self.e2_x_mm()
        if x is None:
            self._send("P1500")
            self.prev_e = None
            self.stat.config(text="E2 无有效信号，等待…")
            return

        err = (SCREEN_W_MM / 2.0) - x
        aerr = abs(err)

        # ---- 死区：停住 ----
        if aerr < self.db_mm:
            self._send("P1500")
            self.stat.config(text="已对准  X=%.1f  err=%+.1f  E2峰=%d"
                                  % (x, err, int(peak)))
            self.prev_e = aerr
            self.worse_count = 0
            return

        # ---- 方向自学习：上一轮移动后误差反而变大 ----
        if self.prev_e is not None and self.last_moved:
            if aerr > self.prev_e + 1.0:
                self.worse_count = getattr(self, "worse_count", 0) + 1
                if self.worse_count >= 2:
                    self.dir_sign = -self.dir_sign
                    self.worse_count = 0
                    self.stat.config(text="方向翻转 -> %s"
                                          % ("+" if self.dir_sign > 0 else "-"))
                    self.prev_e = aerr
                    self.last_moved = False
                    return
            else:
                self.worse_count = 0
        self.prev_e = aerr
        self.last_moved = False

        # ---- 转动时长：比例项（限幅） ----
        ms = self.kp_ms * aerr
        ms = max(self.min_ms, min(self.max_ms, ms))

        # 偏离中点的脉宽量（越大转越快）
        dev = int(self.dev_us)
        direction = 1.0 if err > 0 else -1.0
        pulse = 1500 - int(dev * direction * self.dir_sign)

        self._pulse_and_wait(pulse, ms)
        self.last_moved = True

        self.stat.config(
            text="移动  X=%.1f  err=%+.1f  P=%d  %dms  dir=%s"
                 % (x, err, pulse, int(ms),
                    "+" if self.dir_sign > 0 else "-"))

'''
    s = s[:start] + new + s[end:]

# 参数：改成移动-测量所需的
for pat, rep in [
    (r"\n\s*self\.kp_ms = [^\n]*", "\n        self.kp_ms = 6.0        # 每 mm 误差对应多少毫秒转动"),
    (r"\n\s*self\.min_ms = [^\n]*", "\n        self.min_ms = 40.0      # 最短转动"),
    (r"\n\s*self\.max_ms = [^\n]*", "\n        self.max_ms = 500.0     # 最长转动"),
    (r"\n\s*self\.dev_us = [^\n]*", "\n        self.dev_us = 400.0     # 脉宽偏离量（越大转越快）"),
    (r"\n\s*self\.db_mm = [^\n]*", "\n        self.db_mm = 2.0        # 死区"),
]:
    s = re.sub(pat, rep, s, count=1)

# 补充状态变量
if "self.last_moved" not in s:
    s = s.replace("        self.integral = 0.0",
                  "        self.integral = 0.0\n        self.last_moved = False\n        self.worse_count = 0")

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
