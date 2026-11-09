#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_track.py — 把 pos_ui.py 的跟随改成连续旋转舵机的速度控制"""

import io
import re

F = "tools/pos_ui.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 参数区：替换控制参数 ----------
old_params = """self.tracking = False"""
new_params = """self.tracking = False"""
# 保留

# 把 kp/max_step/min_step 换成脉宽参数
s = re.sub(r"\n\s*self\.kp = [^\n]*", "\n        self.kp_us = 6.0        # 比例：每 mm 误差对应多少 us 偏离", s, count=1)
s = re.sub(r"\n\s*self\.max_step = [^\n]*", "\n        self.max_dev = 350      # 最大脉宽偏离（us）", s, count=1)
s = re.sub(r"\n\s*self\.min_step = [^\n]*", "\n        self.min_dev = 40       # 最小脉宽偏离（us）", s, count=1)

# ---------- 2) 替换 track_step ----------
start = s.find("    def track_step(self):")
end = s.find("    def _send(self, cmd):")
if start > 0 and end > start:
    new = '''    def _pulse(self, us):
        """发送脉宽命令"""
        us = int(round(us))
        if us < 1000:
            us = 1000
        if us > 2000:
            us = 2000
        self._send("P%d" % us)
        self.last_cmd = time.time()
        self.last_pulse = us

    def track_step(self):
        """连续旋转舵机的速度控制

        P1500 = 停止；偏离 1500 越多转得越快。
        误差 > 死区 -> 按比例给偏离；误差 < 死区 -> P1500 停住。
        方向自动学习：若连续两次误差变大，翻转方向。
        """
        if not self.tracking:
            return

        # ---- 取当前 X ----
        x = None
        if self.fast_x is not None and (time.time() - self.fast_age) < 0.6:
            x = self.fast_x
        elif self.pos and self.pos.valid:
            x = self.pos.x_mm

        # 无有效信号 -> 停住，避免乱转
        if x is None:
            if getattr(self, "last_pulse", 1500) != 1500:
                self._pulse(1500)
            self.prev_err = None
            return

        err = (SCREEN_W_MM / 2.0) - x        # 目标 X = 100mm
        aerr = abs(err)

        # ---- 方向自动学习 ----
        if self.prev_err is not None and getattr(self, "last_pulse", 1500) != 1500:
            if aerr > self.prev_err + 0.6:
                self.worse_count += 1
                if self.worse_count >= 2:
                    self.dir_sign = -self.dir_sign
                    self.worse_count = 0
                    self._pulse(1500)        # 先停一下再换向
                    self.stat.config(
                        text="方向已翻转 -> %s    X=%.1f"
                             % ("+" if self.dir_sign > 0 else "-", x))
                    return
            else:
                self.worse_count = 0
        self.prev_err = aerr

        # ---- 死区：停住 ----
        if aerr < self.db_mm:
            if getattr(self, "last_pulse", 1500) != 1500:
                self._pulse(1500)
            return

        # ---- 限位保护 ----
        # 用累计的等效角度做软限位（P1500 为 90 度）
        if self.servo_deg >= self.max_deg and err * self.dir_sign > 0:
            self._pulse(1500)
            self.stat.config(text="已达正限位")
            return
        if self.servo_deg <= self.min_deg and err * self.dir_sign < 0:
            self._pulse(1500)
            self.stat.config(text="已达负限位")
            return

        # ---- 比例速度控制 ----
        dev = self.kp_us * aerr
        dev = max(self.min_dev, min(self.max_dev, dev))

        # err>0 表示 X 偏小，需要往正方向拉
        sgn = 1 if err > 0 else -1
        pulse = 1500 + self.dir_sign * dev * sgn

        self._pulse(pulse)

        # 估算等效角度（用于软限位）
        self.servo_deg += (pulse - 1500) * 0.02

        self.stat.config(text="跟随中  X=%.1f  err=%+.1f  P=%d  dir=%s"
                              % (x, err, int(round(pulse)),
                                 "+" if self.dir_sign > 0 else "-"))

'''
    s = s[:start] + new + s[end:]

# ---------- 3) toggle_track 停止时回 P1500 ----------
old_stop = '''        else:
            self.btn_track.config(text="开始跟随", fg=GOOD)
            self.stat.config(text="已停止")'''
new_stop = '''        else:
            self.btn_track.config(text="开始跟随", fg=GOOD)
            self._pulse(1500)          # 停止时回中，避免舵机空转
            self.stat.config(text="已停止")'''
if old_stop in s:
    s = s.replace(old_stop, new_stop)

# ---------- 4) 初始化补齐 ----------
if "self.last_pulse" not in s:
    s = s.replace("self.last_cmd = 0.0",
                  "self.last_cmd = 0.0\n        self.last_pulse = 1500")

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
