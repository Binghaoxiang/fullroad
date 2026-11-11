#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""en_group.py — 每次扫描只驱动目标板的那一组 EN，其余全部拉低

对应关系：
  E0 下边 = FPC12 = 网络17-20 = k_en_a
  E1 右边 = FPC16 = 网络01-04 = k_en_b
  E2 上边 = FPC13 = 网络13-16 = k_en_c
  E3 左边 = FPC14 = 网络09-12 = k_en_d
"""

import io
import re

F = "Core/Src/adc64.c"
s = io.open(F, "r", encoding="utf-8").read()

# ---------- 1) 加入 s_en_group 变量 ----------
if "s_en_group" not in s:
    s = s.replace("static uint8_t  s_en_hold   = 0U;",
                  "static uint8_t  s_en_hold   = 0U;\n"
                  "static uint8_t  s_en_group  = 0xFFU;   /* 0xFF=驱动全部；0..3=只驱动该组 */")

# ---------- 2) 重写 en_on ----------
m = re.search(r"static void en_on\(uint8_t row\)\s*\{", s)
if m:
    i = s.index("{", m.start())
    depth = 0
    while i < len(s):
        if s[i] == "{":
            depth += 1
        elif s[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    new = '''static void en_on(uint8_t row)
{
    uint8_t r;
    uint8_t g = s_en_group;

    /* 只驱动目标板那一组；其余三组全部拉低，彻底消除 FPC 之间的串扰 */
    for (r = 0U; r < ADC64_ROWS; r++) {
        uint8_t hi = (r == row) ? 1U : 0U;

        if ((g == 0xFFU) || (g == 0U)) {
            if (hi != 0U) { en_high(&k_en_a[r]); } else { en_low(&k_en_a[r]); }
        } else {
            en_low(&k_en_a[r]);
        }

        if ((g == 0xFFU) || (g == 1U)) {
            if (hi != 0U) { en_high(&k_en_b[r]); } else { en_low(&k_en_b[r]); }
        } else {
            en_low(&k_en_b[r]);
        }

        if ((g == 0xFFU) || (g == 2U)) {
            if (hi != 0U) { en_high(&k_en_c[r]); } else { en_low(&k_en_c[r]); }
        } else {
            en_low(&k_en_c[r]);
        }

        if ((g == 0xFFU) || (g == 3U)) {
            if (hi != 0U) { en_high(&k_en_d[r]); } else { en_low(&k_en_d[r]); }
        } else {
            en_low(&k_en_d[r]);
        }
    }
}'''
    s = s[:m.start()] + new + s[end:]

# ---------- 3) 提供设置接口 ----------
if "adc64_set_en_group" not in s:
    s = s.replace("void adc64_set_en_hold(uint8_t v)",
                  "void adc64_set_en_group(uint8_t g) { s_en_group = (g > 3U) ? 0xFFU : g; }\n\n"
                  "void adc64_set_en_hold(uint8_t v)")

io.open(F, "w", encoding="utf-8").write(s)
print("patched", F)
