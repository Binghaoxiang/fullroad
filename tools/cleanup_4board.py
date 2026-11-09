#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cleanup_4board.py — 剥离四板实验代码，恢复到可用的 2 板状态"""

import io
import re
import sys

ADC = "Core/Src/adc64.c"
CMD = "Core/Src/cmd.c"
CFG = "Core/Inc/config.h"

# ---------------- adc64.c ----------------
s = io.open(ADC, "r", encoding="utf-8", errors="ignore").read()

# 1) 删除 adc64_probe_cs 及其前面的注释块、gpio_clock_on、pin_set_output
start = s.find("/*----------------------------------------------------------------------------\n * 临时片选探测")
if start < 0:
    start = s.find("static void gpio_clock_on")
if start > 0:
    # 找到 adc64_probe_cs 函数结束
    p = s.find("uint32_t adc64_probe_cs", start)
    if p > 0:
        # 从 p 开始找到匹配的收尾 "}\n"
        depth = 0
        i = s.find("{", p)
        end = -1
        while i < len(s):
            if s[i] == "{":
                depth += 1
            elif s[i] == "}":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
            i += 1
        if end > 0:
            s = s[:start] + s[end:]
            print("已删除 adc64_probe_cs 及辅助函数")

# 2) 删除 adc64_read_edges_all
p = s.find("void adc64_read_edges_all")
if p > 0:
    depth = 0
    i = s.find("{", p)
    end = -1
    while i < len(s):
        if s[i] == "{":
            depth += 1
        elif s[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    if end > 0:
        s = s[:p] + s[end:]
        print("已删除 adc64_read_edges_all")

# 3) 删除 adc64_read_edge（单边读取，也依赖 k_edges 但无害，先保留）
# 4) GPIOI 配置恢复为仅 PI8
s = re.sub(
    r"GPIOI->MODER = \(GPIOI->MODER[^;]*;",
    "GPIOI->MODER = (GPIOI->MODER & ~(3UL << 16)) | (1UL << 16);",
    s, count=1)
s = re.sub(r"GPIOI->BSRR\s*=\s*\(1UL << 5\)[^;]*;", "GPIOI->BSRR  = (1UL << 8);", s)
s = re.sub(r"GPIOI->BSRR\s*=\s*\(1UL << 8\) \| \(1UL << 9\)[^;]*;",
           "GPIOI->BSRR  = (1UL << 8);", s)

io.open(ADC, "w", encoding="utf-8").write(s)

# ---------------- cmd.c ----------------
c = io.open(CMD, "r", encoding="utf-8", errors="ignore").read()

for name in ("CSCAN", "EDGES"):
    m = re.search(r'\n    if \(eqi\(tok, "%s"\) != 0U\) \{' % name, c)
    if not m:
        print("%s 分支未找到" % name)
        continue
    start = m.start()
    i = c.find("{", m.end() - 2)
    depth = 0
    end = -1
    while i < len(c):
        if c[i] == "{":
            depth += 1
        elif c[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    if end > 0:
        # 跳过后面的换行
        while end < len(c) and c[end] in " \t\r\n":
            end += 1
        c = c[:start] + "\n" + c[end:]
        print("已删除 K %s 分支" % name)

io.open(CMD, "w", encoding="utf-8").write(c)

# ---------------- config.h ----------------
g = io.open(CFG, "r", encoding="utf-8", errors="ignore").read()
g = re.sub(r"#define ADC64_BOARD_COUNT\s+\d+U", "#define ADC64_BOARD_COUNT        2U", g)
io.open(CFG, "w", encoding="utf-8").write(g)
print("BOARD_COUNT 已设为 2")
