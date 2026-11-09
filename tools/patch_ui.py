#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""patch_ui.py — 用 Python 精确修改 spot_ui.py 的数值显示"""

import io
import re

F = "tools/spot_ui.py"
src = io.open(F, "r", encoding="utf-8").read()

# ---- 1) 删除柱顶竖排数字 ----
pat = re.compile(
    r"\n( +)if v >= NOISE:\n"
    r" +lit_any = True\n"
    r" +if v > NOISE \* 2:\n"
    r" +c\.create_text\(bx \+ bw / 2, base - bh - 7, text=str\(v\),\n"
    r" +fill=color, angle=90,\n"
    r" +font=\(\"Consolas\", 7\)\)"
)
src2, n1 = pat.subn(lambda m: "\n%slit_any = True" % m.group(1), src)
print("柱顶数字移除:", n1)

# ---- 2) 信息行替换为 峰值/展宽/重心 + 前三强 ----
old = '''        if not lit_any:
            info = "无有效光：峰值 %d（阈值 %d）" % (maxv, NOISE)
            col_t = "#4a5a6c"
        else:
            info = "峰值 %d　展宽 %.2f" % (maxv, width)
            if center is not None:
                info += "　重心 %.2f" % (center + 1)
            col_t = "#6e8092"
        c.create_text(x, base + 46, text=info, fill=col_t, anchor="w",
                      font=("Consolas", 9))'''

new = '''        if not lit_any:
            c.create_text(x, base + 46,
                          text="无有效光：峰值 %d（阈值 %d）" % (maxv, NOISE),
                          fill="#4a5a6c", anchor="w", font=("Consolas", 9))
            return
        info = "峰值 %d　展宽 %.2f" % (maxv, width)
        if center is not None:
            info += "　重心 %.2f" % (center + 1)
        c.create_text(x, base + 46, text=info, fill="#6e8092", anchor="w",
                      font=("Consolas", 9))

        # 前三强：丝印 + 原始值
        order = sorted(range(n), key=lambda k: arr[k], reverse=True)[:3]
        parts = []
        for k in order:
            if k < len(labels) and arr[k] >= NOISE:
                parts.append("%s=%d" % (labels[k], int(arr[k])))
        if parts:
            c.create_text(x, base + 62, text="　".join(parts),
                          fill=color, anchor="w", font=("Consolas", 9))'''

if old in src2:
    src2 = src2.replace(old, new)
    print("信息行替换: OK")
else:
    print("信息行替换: 未匹配（保持原样）")

io.open(F, "w", encoding="utf-8").write(src2)
print("written", F)
