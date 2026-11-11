#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dsp_v2.py — 重心改为「最大连通区」，并按边区分参数

解决：
  1. 白线不对准波峰 —— 重心落在多个峰之间
  2. 杂峰多 —— 阈值以上的散点被一起平均
  3. 长短边跨度不同 —— 短边远端可能无信号，需要单独参数
"""

import io

F = "tools/edge_dsp.py"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 在 process_edge 里把重心改成最大连通区 ----------
old = """    # ---- ⑥ 重心（用自适应阈值）----
    s0 = 0.0
    s1 = 0.0
    cnt = 0
    for i in range(n):
        if corr[i] >= thr:
            s0 += corr[i]
            s1 += corr[i] * i
            cnt += 1
    if s0 > 0.0:
        pos = s1 / s0
        total = s0
    else:
        pos = None
        total = 0.0"""

new = """    # ---- ⑥ 重心：只在「最大的连续过阈值区段」内计算 ----
    # 这样多峰时不会把重心拉到峰之间，白线自然对齐主峰
    runs = []
    i = 0
    while i < n:
        if corr[i] >= thr:
            j = i
            acc = 0.0
            while j < n and corr[j] >= thr:
                acc += corr[j]
                j += 1
            runs.append((acc, i, j - 1))
            i = j
        else:
            i += 1

    pos = None
    total = 0.0
    cnt = 0
    peak_idx = None

    if runs:
        # 取累积量最大的那一段（= 主峰）
        runs.sort(key=lambda r: r[0], reverse=True)
        acc_best, i0, i1 = runs[0]
        s0 = 0.0
        s1 = 0.0
        for k in range(i0, i1 + 1):
            s0 += corr[k]
            s1 += corr[k] * k
            cnt += 1
        if s0 > 0.0:
            pos = s1 / s0
            total = s0
            # 主峰段内的最大值位置
            peak_idx = max(range(i0, i1 + 1), key=lambda k: corr[k])
            # 主峰段内不允许"全场最大值"在段外
            global_max = max(corr)
            if corr[peak_idx] < global_max * 0.6:
                # 主峰段与全场最大峰不是同一个 → 用全场最大峰所在段
                for acc2, j0, j1 in runs:
                    if j0 <= corr.index(global_max) <= j1:
                        i0, i1 = j0, j1
                        break
                s0 = 0.0
                s1 = 0.0
                cnt = 0
                for k in range(i0, i1 + 1):
                    s0 += corr[k]
                    s1 += corr[k] * k
                    cnt += 1
                if s0 > 0.0:
                    pos = s1 / s0
                    total = s0
                    peak_idx = max(range(i0, i1 + 1), key=lambda k: corr[k])

    # 主峰宽度（探测器数），用于质量评估
    peak_width = cnt"""

if old in s:
    s = s.replace(old, new, 1)
    print("重心已改为最大连通区")
else:
    print("重心锚点未匹配")

# ---------- 返回值里加入 peak_idx / peak_width ----------
s = s.replace(
    '"mu": (fit[0] if fit else pos),',
    '"mu": (fit[0] if fit else pos),\n        "peak_idx": peak_idx,\n        "peak_width": peak_width,')

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
