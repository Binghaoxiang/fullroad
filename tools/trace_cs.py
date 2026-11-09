#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""trace_cs.py — 从 PCB 数据追踪 SPI4_CS3 / SPI4_CS2 网络的所有焊盘，定位 MCU 引脚"""

import os
import re
import sys

EPCB = r"tools/_epro/PCB/a0f137a2df6944d6b0dc32f16d56264a.epcb"

d = open(EPCB, "rb").read().decode("latin-1")
print("PCB 文件大小: %d" % len(d))

TARGETS = ["SPI4_CS3", "SPI4_CS2", "SPI4_CS0", "SPI4_CS4"]

# 焊盘记录格式: ["PAD","e672",<layer>,"NET",<netid>,"<padname>",x,y,...
pad_re = re.compile(
    r'\["PAD","(e\d+)",(-?\d+),"([^"]*)",(\d+),"([^"]*)",'
    r'(-?[\d.]+),(-?[\d.]+)')

allpads = {}
for m in pad_re.finditer(d):
    net = m.group(3)
    allpads.setdefault(net, []).append({
        "id": m.group(1),
        "layer": m.group(2),
        "pad": m.group(5),
        "x": float(m.group(6)),
        "y": float(m.group(7)),
    })

print("\n=== 目标网络的全部焊盘 ===")
for net in TARGETS:
    pads = allpads.get(net, [])
    print("\n%s  共 %d 个焊盘" % (net, len(pads)))
    for p in pads:
        print("   pad=%-4s layer=%-3s  pos=(%.1f, %.1f)"
              % (p["pad"], p["layer"], p["x"], p["y"]))

# 统计所有 pad 数量最多的网络（MCU 的 GND 等会很多）
print("\n=== 焊盘数最多的 12 个网络（用于定位 MCU 区域）===")
rank = sorted(allpads.items(), key=lambda kv: -len(kv[1]))[:12]
for net, pads in rank:
    xs = [p["x"] for p in pads]
    ys = [p["y"] for p in pads]
    print("  %-18s %3d 个   x:[%.0f,%.0f] y:[%.0f,%.0f]"
          % (net or "(空)", len(pads), min(xs), max(xs), min(ys), max(ys)))
