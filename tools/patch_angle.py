#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_angle.py — 给 spot_ui.py 加入几何法入射角 + 局部舵机角跟踪"""

import io

F = "tools/spot_ui.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 增加全局参数 ----------
if "L_PARAM" not in s:
    s = s.replace(
        "NOISE = 250\n",
        "NOISE = 250\nL_PARAM = 260.0      # 等效力臂 mm（用于 atan(Δx/L) 换算角度）\n"
        "SERVO_PER_DEG = 8    # 每度舵机脉冲毫秒（与固件一致）\n"
    )

# ---------- 2) DSP 增加垂直基准位置 ----------
s = s.replace(
    "        self.peak_ref = 0\n",
    "        self.peak_ref = 0\n        self.x_peak = None\n        self.y_peak = None\n"
)

# ---------- 3) DSP 增加角度方法 ----------
if "def angle_from_pos" not in s:
    s = s.replace(
        "    @staticmethod\n    def width(arr):",
        "    def angle_from_pos(self):\n"
        "        \"\"\"几何法入射角：θ = atan((X − X_peak) / L)\"\"\"\n"
        "        if self.x_peak is None or self.x_idx is None:\n"
        "            return None\n"
        "        dx = (self.x_idx - self.x_peak) * PITCH_MM\n"
        "        return math.degrees(math.atan2(dx, L_PARAM))\n"
        "\n"
        "    @staticmethod\n    def width(arr):"
    )

# ---------- 4) 标定按钮同时记录 X 位置 ----------
s = s.replace(
    '    def do_cal(self):\n'
    '        m = max(self.dsp.max_l, self.dsp.max_s)\n'
    '        if m > 0:\n'
    '            self.dsp.peak_ref = m\n'
    '            self.stat.config(text="基准已设定 %d" % m)',
    '    def do_cal(self):\n'
    '        m = max(self.dsp.max_l, self.dsp.max_s)\n'
    '        if m > 0 and self.dsp.x_idx is not None:\n'
    '            self.dsp.peak_ref = m\n'
    '            self.dsp.x_peak = self.dsp.x_idx\n'
    '            self.dsp.y_peak = self.dsp.y_idx\n'
    '            self.stat.config(text="垂直基准已锁定：峰值 %d  X=%.1f" %\n'
    '                             (m, self.dsp.x_peak * PITCH_MM))'
)

# ---------- 5) draw_left 里改用几何角 ----------
old_ang = '''        if self.ang is not None:
            txt = "θ = %+.1f°" % self.ang
            col = GOOD if abs(self.ang) < 3 else (WARN if abs(self.ang) < 8 else BAD)
        else:
            p = self.dsp.perpendicular()
            if p is not None and self.dsp.peak_ref > 0:
                txt = "垂直度 %.0f%%" % (p * 100)
                col = GOOD if p > 0.95 else (WARN if p > 0.85 else BAD)
            else:
                txt, col = "θ = --", DIM'''
new_ang = '''        ang = self.dsp.angle_from_pos()
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
                txt, col = '点击「标定基准」', DIM'''
if old_ang in s:
    s = s.replace(old_ang, new_ang)
    print("角度显示: OK")
else:
    print("角度显示: 未匹配")

# ---------- 6) 底部读数改用几何角 ----------
old_foot = '''        if self.ang is not None:
            self.lb_ang.config(text="入射角  %+.1f°" % self.ang,
                               fg=GOOD if abs(self.ang) < 3 else
                               (WARN if abs(self.ang) < 8 else BAD))
        else:
            self.lb_ang.config(text="入射角  --", fg=DIM)'''
new_foot = '''        ang = self.dsp.angle_from_pos()
        if ang is None:
            ang = self.ang
        if ang is not None:
            self.lb_ang.config(text="入射角  %+.1f°" % ang,
                               fg=GOOD if abs(ang) < 3 else
                               (WARN if abs(ang) < 8 else BAD))
        else:
            self.lb_ang.config(text="入射角  --（需先标定）", fg=DIM)'''
if old_foot in s:
    s = s.replace(old_foot, new_foot)
    print("底部读数: OK")
else:
    print("底部读数: 未匹配")

io.open(F, "w", encoding="utf-8").write(s)
print("written")
