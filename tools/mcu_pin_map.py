#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mcu_pin_map.py — 建立 MCU 物理引脚号 <-> 端口名 的对应表"""

import os
import re
import sys

ESYM = r"tools/_epro/SYMBOL/7fde2d7e7b87477bad1b222feb301e0b.esym"

d = open(ESYM, "rb").read().decode("latin-1")

# 每条 PIN 记录：["PIN","e4",...] 后面紧跟
#   ["ATTR","e1","e4","NAME","PE2",...]
#   ["ATTR","e2","e4","NUMBER","1",...]
pat = re.compile(
    r'\["PIN","(e\d+)"(.*?)(?=\["PIN"|\Z)', re.S)

pins = {}
for m in pat.finditer(d):
    eid = m.group(1)
    body = m.group(2)
    mn = re.search(r'\["ATTR","(e\d+)","%s","NAME","([^"]+)"' % eid, body)
    nu = re.search(r'\["ATTR","(e\d+)","%s","NUMBER","([^"]+)"' % eid, body)
    if mn and nu:
        pins[nu.group(2)] = mn.group(2)

print("解析引脚数:", len(pins))
if not pins:
    sys.exit(1)

print("\n=== 物理引脚号 -> 端口名（前 20）===")
for i in range(1, 21):
    print("  %-3d -> %s" % (i, pins.get(str(i), "?")))

inv = {}
for num, name in pins.items():
    inv.setdefault(name, num)

print("\n=== 关键端口 -> 物理引脚号 ===")
for nm in ["PE2", "PE5", "PE6", "PH4", "PI8", "PI9", "PI10", "PI11",
           "PC13", "PC14", "PC15", "PF1", "PG6", "PG7", "PB0"]:
    print("  %-5s -> %s" % (nm, inv.get(nm, "?")))

# 反查用户说的 8 和 11
print("\n=== 关键物理脚反查 ===")
for n in ["7", "8", "9", "10", "11", "12"]:
    print("  物理脚 %-3s -> %s" % (n, pins.get(n, "?")))
