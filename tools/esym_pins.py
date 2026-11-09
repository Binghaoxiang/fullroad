#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""esym_pins.py — 从 Altium 符号提取 MCU 引脚名与编号"""

import os
import re
import sys

root = sys.argv[1] if len(sys.argv) > 1 else "tools/_epro"

best = None
bestn = 0
for dp, _, fns in os.walk(root):
    for fn in fns:
        if not fn.endswith(".esym"):
            continue
        p = os.path.join(dp, fn)
        try:
            d = open(p, "rb").read().decode("latin-1")
        except Exception:
            continue
        n = d.count('"PIN"')
        if n > bestn:
            bestn = n
            best = (p, d)

if not best:
    print("未找到符号文件")
    sys.exit(1)

print("MCU 符号:", os.path.basename(best[0]), " PIN 记录数:", bestn)
d = best[1]

# Altium esym 里 PIN 记录的典型形式：
#   ["PIN","e12","...", ..., "NAME","PC13", ... , "DESIGNATOR","7", ...]
# 用宽松方式：截取每条 ["PIN", ... ] 直到下一个顶层 "["
recs = re.findall(r'\["PIN","(e\d+)".*?(?=\["PIN"|\Z)', d, re.S)
print("解析到 PIN 条目:", len(recs))

want = {"PC13", "PC14", "PC15", "PI9", "PI10", "PI11", "PI8", "PH4",
        "PF1", "PC0", "PC1", "PC2", "PC3"}
found = {}
for r in recs:
    mn = re.search(r'"NAME","([^"]+)"', r)
    md = re.search(r'"DESIGNATOR","([^"]+)"', r)
    if mn:
        nm = mn.group(1)
        num = md.group(1) if md else "?"
        if nm in want:
            found[nm] = num

if found:
    print("\n目标引脚:")
    for k in sorted(found):
        print("  %-6s -> 编号 %s" % (k, found[k]))
else:
    print("\n未匹配到目标引脚，列出前 30 个 NAME:")
    cnt = 0
    for r in recs:
        mn = re.search(r'"NAME","([^"]+)"', r)
        md = re.search(r'"DESIGNATOR","([^"]+)"', r)
        if mn:
            print("  %-8s %s" % (mn.group(1), md.group(1) if md else "?"))
            cnt += 1
            if cnt >= 30:
                break
