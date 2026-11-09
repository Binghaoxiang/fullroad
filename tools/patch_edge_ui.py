#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_edge_ui.py — 给 edge_check.py 接入 edge_dsp 处理链 + 自动缩放"""

import io
import re

F = "tools/edge_check.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 导入 DSP ----------
if "import edge_dsp" not in s:
    s = s.replace("import serial\n",
                  "import serial\nimport edge_dsp\n", 1)

# ---------- 2) App.__init__ 增加处理状态 ----------
if "self.proc = " not in s:
    s = s.replace(
        "        self.data = [None] * 4\n",
        "        self.data = [None] * 4\n"
        "        self.proc = [None] * 4\n"
        "        self.base = [None] * 4\n"
        "        self.tracker = [edge_dsp.Tracker(0.5, 0.1) for _ in range(4)]\n"
        "        self.show_raw = True\n"
    )

# ---------- 3) 顶栏加「原始/处理」切换 ----------
if 'self.mk(bar, "原始"' not in s:
    s = s.replace(
        '        self.mk(bar, "断开", self.disconnect, DIM)\n',
        '        self.mk(bar, "断开", self.disconnect, DIM)\n'
        '        self.mk(bar, "原始/处理", self.toggle_raw, WARN)\n'
    )

# ---------- 4) 增加 toggle 方法 ----------
if "def toggle_raw" not in s:
    s = s.replace(
        "    def connect(self):",
        "    def toggle_raw(self):\n"
        "        self.show_raw = not self.show_raw\n"
        "        self.stat.config(text=\"显示原始数据\" if self.show_raw else \"显示处理后\")\n"
        "\n"
        "    def connect(self):"
    )

# ---------- 5) poll 里加入处理 ----------
old_poll = """                for k, v in got.items():
                    self.data[k] = v"""
new_poll = """                for k, v in got.items():
                    self.data[k] = v
                    n = len(v)
                    if self.base[k] is None or len(self.base[k].base) != n:
                        self.base[k] = edge_dsp.Baseline(n, 0.02)
                    self.proc[k] = edge_dsp.process_edge(
                        v, self.base[k], thr_ratio=0.25,
                        noise_floor=60.0, use_median=True, use_savgol=True)"""
if old_poll in s:
    s = s.replace(old_poll, new_poll)
    print("poll 处理链: OK")
else:
    print("poll 处理链: 未匹配")

# ---------- 6) 替换 draw_strip 为带处理与自动缩放的版本 ----------
start = s.find("    def draw_strip(self, c, k, y):")
end = s.find("def main():")
if start > 0 and end > start:
    new_fn = '''    def draw_strip(self, c, k, y):
        tag, cn, rng, direc, i0, i1, rev, col = EDGES[k]
        labels = edge_silk(k)
        n = len(labels)
        pr = self.proc[k]
        raw = self.data[k] or []

        c.create_rectangle(X0 - 10, y, X0 + XW + 10, y + STRIP_H,
                           fill=CARD, outline="#1a2431")

        c.create_text(X0, y + 12, anchor="w", text=tag,
                      fill=col, font=("Microsoft YaHei", 15, "bold"))
        c.create_text(X0 + 46, y + 14, anchor="w",
                      text="%s  %s  (%s)  %d 颗" % (cn, rng, direc, n),
                      fill=TXT, font=("Microsoft YaHei", 10))

        # 右上角指标
        if pr and pr["valid"]:
            pos = pr["mu"]
            info = "峰 %d" % int(pr["peak"])
            if pos is not None:
                info += "  位置 %.2f" % (pos + 1)
            if pr["sigma"]:
                info += "  σ %.2f" % pr["sigma"]
            c.create_text(X0 + XW, y + 14, anchor="e", text=info,
                          fill=col, font=("Consolas", 10, "bold"))
        else:
            c.create_text(X0 + XW, y + 14, anchor="e", text="无信号",
                          fill="#ff4d5e" if n else DIM,
                          font=("Consolas", 10, "bold"))

        gx = X0 + 62
        gw = XW - 84
        gy = y + 36
        gh = 104
        gb = gy + gh

        # ---- 自动缩放：以处理后的最大值为上限，留 20% 余量 ----
        if pr and pr["valid"]:
            data = pr["corr"] if not self.show_raw else [float(x) for x in raw]
            vmax = max(data) if data else 1.0
        else:
            data = [float(x) for x in raw] if raw else [0.0]
            vmax = max(data) if data else 1.0
        if vmax < 120.0:
            vmax = 120.0
        vmax *= 1.2

        # 网格 + 刻度（用自动缩放后的刻度）
        for f in (0.0, 0.25, 0.5, 0.75, 1.0):
            yy = gb - gh * f
            c.create_line(gx, yy, gx + gw, yy,
                          fill="#22303e" if f in (0, 1) else "#161f2a")
            c.create_text(gx - 8, yy, anchor="e", text="%d" % int(vmax * f),
                          fill="#3d4d5e", font=("Consolas", 7))

        step = gw / max(n - 1, 1)

        # ---- 原始曲线（暗色背景，仅处理模式时显示）----
        if not self.show_raw and raw:
            pts = []
            for i in range(min(n, len(raw))):
                px = gx + i * step
                py = gb - min(gh, (float(raw[i]) / vmax) * gh)
                pts.extend([px, py])
            if len(pts) >= 4:
                c.create_line(*pts, fill="#2a3a4a", width=1)

        # ---- 主曲线 ----
        if not raw:
            c.create_text(gx + gw / 2, gy + gh / 2, text="无数据",
                          fill="#3d4d5e", font=("Microsoft YaHei", 13))
        else:
            pts = []
            for i in range(min(n, len(data))):
                px = gx + i * step
                py = gb - min(gh, (float(data[i]) / vmax) * gh)
                pts.extend([px, py])
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=2)
                for i in range(0, len(pts) // 2, 2):
                    pass
            # 数据点
            for i in range(min(n, len(data))):
                if float(data[i]) > vmax * 0.12:
                    px = gx + i * step
                    py = gb - min(gh, (float(data[i]) / vmax) * gh)
                    c.create_oval(px - 2.5, py - 2.5, px + 2.5, py + 2.5,
                                  fill=col, outline="")

        # ---- 阈值线 ----
        if pr and pr["valid"]:
            thr = pr["peak"] * 0.25
            ty = gb - min(gh, (thr / vmax) * gh)
            c.create_line(gx, ty, gx + gw, ty, fill="#5a3a3a", dash=(3, 5))

        # ---- 重心/拟合位置标记 ----
        if pr and pr["valid"] and pr["mu"] is not None:
            mx_ = gx + pr["mu"] * step
            c.create_line(mx_, gy - 4, mx_, gb, fill="#ff5566", width=2)
            c.create_polygon(mx_ - 4, gy - 12, mx_ + 4, gy - 12, mx_, gy - 4,
                             fill="#ff5566", outline="")

        # ---- 丝印 ----
        for i in range(n):
            px = gx + i * step
            c.create_text(px, gb + 5, text=labels[i], angle=90,
                          anchor="n", fill="#4e5f72",
                          font=("Consolas", 6))

        # ---- 首尾与方向 ----
        ay = y + STRIP_H - 14
        c.create_text(X0 + 12, ay, anchor="w", text=labels[0],
                      fill=col, font=("Consolas", 8, "bold"))
        c.create_text(X0 + XW - 12, ay, anchor="e", text=labels[-1],
                      fill=col, font=("Consolas", 8, "bold"))
        c.create_text(X0 + XW / 2, ay, anchor="center", text=direc,
                      fill=DIM, font=("Microsoft YaHei", 8))


'''
    s = s[:start] + new_fn + s[end:]
    print("draw_strip 替换: OK")
else:
    print("draw_strip 替换: 未匹配")

# ---------- 7) 底栏显示处理信息 ----------
old_lb = '''        parts = []
        for k in range(4):
            v = self.data[k]
            if v:
                parts.append("%s 峰%4d" % (EDGES[k][0], max(v)))
            else:
                parts.append("%s --" % EDGES[k][0])'''
new_lb = '''        parts = []
        for k in range(4):
            pr = self.proc[k]
            if pr and pr["valid"]:
                s_ = "峰%4d" % int(pr["peak"])
                if pr["mu"] is not None:
                    s_ += " 位%.1f" % (pr["mu"] + 1)
                parts.append("%s %s" % (EDGES[k][0], s_))
            else:
                parts.append("%s --" % EDGES[k][0])'''
if old_lb in s:
    s = s.replace(old_lb, new_lb)
    print("底栏: OK")
else:
    print("底栏: 未匹配")

io.open(F, "w", encoding="utf-8").write(s)
print("written", F)
